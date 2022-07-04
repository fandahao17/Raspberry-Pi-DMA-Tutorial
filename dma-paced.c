/*
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>
*/

#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <signal.h>
#include <bcm_host.h>

#include "mailbox.h"

/*
 * Check more about Raspberry Pi's register mapping at:
 * https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 * https://elinux.org/BCM2835_registers
 */
#define PAGE_SIZE 4096

#define PERI_BUS_BASE 0x7E000000
#define PERI_PHYS_BASE 0x3F000000
#define BUS_TO_PHYS(x) ((x) & ~0xC0000000)

#define CM_BASE 0x00101000
#define CM_LEN 0xA8
#define CM_PWM 0xA0
#define CLK_CTL_BUSY (1 << 7)
#define CLK_CTL_KILL (1 << 5)
#define CLK_CTL_ENAB (1 << 4)
#define CLK_CTL_SRC(x) ((x) << 0)

#define CLK_SRCS 2

#define CLK_CTL_SRC_OSC 1
#define CLK_CTL_SRC_PLLD 6

#define CLK_OSC_FREQ 19200000
#define CLK_OSC_FREQ_2711 54000000
#define CLK_PLLD_FREQ 500000000
#define CLK_PLLD_FREQ_2711 750000000

#define CLK_DIV_DIVI(x) ((x) << 12)

#define BCM_PASSWD (0x5A << 24)

#define PWM_BASE 0x0020C000
#define PWM_LEN 0x28
#define PWM_FIFO 0x18

/* PWM control bits */
#define PWM_CTL 0
#define PWM_STA 1
#define PWM_DMAC 2
#define PWM_RNG1 4
#define PWM_DAT1 5
#define PWM_RNG2 8
#define PWM_DAT2 9

#define PWM_CTL_MSEN2 (1 << 15)
#define PWM_CTL_PWEN2 (1 << 8)
#define PWM_CTL_MSEN1 (1 << 7)
#define PWM_CTL_CLRF1 (1 << 6)
#define PWM_CTL_USEF1 (1 << 5)
#define PWM_CTL_MODE1 (1 << 1)
#define PWM_CTL_PWEN1 (1 << 0)

#define PWM_DMAC_ENAB (1 << 31)
#define PWM_DMAC_PANIC(x) ((x) << 8)
#define PWM_DMAC_DREQ(x) (x)

#define SYST_BASE 0x3000
#define SYST_LEN 0x1C
#define SYST_CLO 0x04

#define DMA_BASE 0x00007000
#define DMA_CHANNEL 6
#define DMA_OFFSET 0x100
#define DMA_ADDR (DMA_BASE + DMA_OFFSET * (DMA_CHANNEL >> 2))

/* DMA CS Control and Status bits */
#define DMA_ENABLE (0xFF0 / 4)
#define DMA_CHANNEL_RESET (1 << 31)
#define DMA_CHANNEL_ABORT (1 << 30)
#define DMA_WAIT_ON_WRITES (1 << 28)
#define DMA_PANIC_PRIORITY(x) ((x) << 20)
#define DMA_PRIORITY(x) ((x) << 16)
#define DMA_INTERRUPT_STATUS (1 << 2)
#define DMA_END_FLAG (1 << 1)
#define DMA_ACTIVE (1 << 0)
#define DMA_DISDEBUG (1 << 28)

/* DMA control block "info" field bits */
#define DMA_NO_WIDE_BURSTS (1 << 26)
#define DMA_PERIPHERAL_MAPPING(x) ((x) << 16)
#define DMA_BURST_LENGTH(x) ((x) << 12)
#define DMA_SRC_IGNORE (1 << 11)
#define DMA_SRC_DREQ (1 << 10)
#define DMA_SRC_WIDTH (1 << 9)
#define DMA_SRC_INC (1 << 8)
#define DMA_DEST_IGNORE (1 << 7)
#define DMA_DEST_DREQ (1 << 6)
#define DMA_DEST_WIDTH (1 << 5)
#define DMA_DEST_INC (1 << 4)
#define DMA_WAIT_RESP (1 << 3)

// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
#define MEM_FLAG_DIRECT (1 << 2)
#define MEM_FLAG_COHERENT (2 << 2)
#define MEM_FLAG_L1_NONALLOCATING (MEM_FLAG_DIRECT | MEM_FLAG_COHERENT)

#define TICK_CNT 20
#define CB_CNT (TICK_CNT * 2)

#define CLK_DIVI 5
#define CLK_MICROS 1

typedef struct DMACtrlReg
{
    uint32_t cs;      // DMA Channel Control and Status register
    uint32_t cb_addr; // DMA Channel Control Block Address
} DMACtrlReg;

typedef struct DMAControlBlock
{
    uint32_t tx_info;    // Transfer information
    uint32_t src;        // Source (bus) address
    uint32_t dest;       // Destination (bus) address
    uint32_t tx_len;     // Transfer length (in bytes)
    uint32_t stride;     // 2D stride
    uint32_t next_cb;    // Next DMAControlBlock (bus) address
    uint32_t padding[2]; // 2-word padding
} DMAControlBlock;

typedef struct DMAMemHandle
{
    void *virtual_addr; // Virutal base address of the page
    uint32_t bus_addr;  // Bus adress of the page, this is not a pointer because it does not point to valid virtual address
    uint32_t mb_handle; // Used by mailbox property interface
    uint32_t size;
} DMAMemHandle;

typedef struct CLKCtrlReg
{
    // See https://elinux.org/BCM2835_registers#CM
    uint32_t ctrl;
    uint32_t div;
} CLKCtrlReg;

typedef struct PWMCtrlReg
{
    uint32_t ctrl;     // 0x0, Control
    uint32_t status;   // 0x4, Status
    uint32_t dma_cfg;  // 0x8, DMA configuration
    uint32_t padding1; // 0xC, 4-byte padding
    uint32_t range1;   // 0x10, Channel 1 range
    uint32_t data1;    // 0x14, Channel 1 data
    uint32_t fifo_in;  // 0x18, FIFO input
    uint32_t padding2; // 0x1C, 4-byte padding again
    uint32_t range2;   // 0x20, Channel 2 range
    uint32_t data2;    // 0x24, Channel 2 data
} PWMCtrlReg;

int mailbox_fd = -1;
DMAMemHandle *dma_cbs;
DMAMemHandle *dma_ticks;
volatile DMACtrlReg *dma_reg;
volatile PWMCtrlReg *pwm_reg;
volatile CLKCtrlReg *clk_reg;

DMAMemHandle *dma_malloc(unsigned int size)
{
    if (mailbox_fd < 0)
    {
        mailbox_fd = mbox_open();
        assert(mailbox_fd >= 0);
    }

    // Make `size` a multiple of PAGE_SIZE
    size = ((size + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;

    DMAMemHandle *mem = (DMAMemHandle *)malloc(sizeof(DMAMemHandle));
    // Documentation: https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
    mem->mb_handle = mem_alloc(mailbox_fd, size, PAGE_SIZE, MEM_FLAG_L1_NONALLOCATING);
    mem->bus_addr = mem_lock(mailbox_fd, mem->mb_handle);
    mem->virtual_addr = mapmem(BUS_TO_PHYS(mem->bus_addr), size);
    mem->size = size;

    assert(mem->bus_addr != 0);

    fprintf(stderr, "MBox alloc: %d bytes, bus: %08X, virt: %08X\n", mem->size, mem->bus_addr, (uint32_t)mem->virtual_addr);

    return mem;
}

void dma_free(DMAMemHandle *mem)
{
    if (mem->virtual_addr == NULL)
        return;

    unmapmem(mem->virtual_addr, mem->size);
    mem_unlock(mailbox_fd, mem->mb_handle);
    mem_free(mailbox_fd, mem->mb_handle);
    mem->virtual_addr = NULL;
}

void *map_peripheral(uint32_t addr, uint32_t size)
{
    int mem_fd;
    // Check mem(4) about /dev/mem
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
    {
        perror("Failed to open /dev/mem: ");
        exit(-1);
    }

    uint32_t *result = (uint32_t *)mmap(
        NULL,
        size,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        bcm_host_get_peripheral_address() + addr);

    close(mem_fd);

    if (result == MAP_FAILED)
    {
        perror("mmap error: ");
        exit(-1);
    }
    return result;
}

void dma_alloc_buffers()
{
    dma_cbs = dma_malloc(CB_CNT * sizeof(DMAControlBlock));
    dma_ticks = dma_malloc(TICK_CNT * sizeof(uint32_t));
}

static inline DMAControlBlock *ith_cb_virt_addr(int i) { return (DMAControlBlock *)dma_cbs->virtual_addr + i; }

static inline uint32_t ith_cb_bus_addr(int i) { return dma_cbs->bus_addr + i * sizeof(DMAControlBlock); }

static inline uint32_t *ith_tick_virt_addr(int i) { return (uint32_t *)dma_ticks->virtual_addr + i; }

static inline uint32_t ith_tick_bus_addr(int i) { return dma_ticks->bus_addr + i * sizeof(uint32_t); }

void dma_init_cbs()
{
    DMAControlBlock *cb;
    for (int i = 0; i < TICK_CNT; i++)
    {
        // Tick block
        cb = ith_cb_virt_addr(2 * i);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
        cb->src = PERI_BUS_BASE + SYST_BASE + SYST_CLO;
        cb->dest = ith_tick_bus_addr(i);
        cb->tx_len = 4;
        cb->next_cb = ith_cb_bus_addr(2 * i + 1);

        // Delay block
        cb = ith_cb_virt_addr(2 * i + 1);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(5);
        cb->src = ith_cb_bus_addr(0); // Dummy data
        cb->dest = PERI_BUS_BASE + PWM_BASE + PWM_FIFO;
        cb->tx_len = 4;
        cb->next_cb = ith_cb_bus_addr((2 * i + 2) % CB_CNT);
    }
    fprintf(stderr, "Init: %d cbs, %d ticks\n", CB_CNT, TICK_CNT);
}

void init_hw_clk()
{
    // See Chanpter 6.3, BCM2835 ARM peripherals for controlling the hardware clock
    // Also check https://elinux.org/BCM2835_registers#CM for the register mapping

    // kill the clock if busy
    if (clk_reg->ctrl & CLK_CTL_BUSY)
    {
        do
        {
            clk_reg->ctrl = BCM_PASSWD | CLK_CTL_KILL;
        } while (clk_reg->ctrl & CLK_CTL_BUSY);
    }

    // Set clock source to plld
    clk_reg->ctrl = BCM_PASSWD | CLK_CTL_SRC(CLK_CTL_SRC_PLLD);
    usleep(10);

    // The original clock speed is 500MHZ, we divide it by 5 to get a 100MHZ clock
    clk_reg->div = BCM_PASSWD | CLK_DIV_DIVI(CLK_DIVI);
    usleep(10);

    // Enable the clock
    clk_reg->ctrl |= (BCM_PASSWD | CLK_CTL_ENAB);
}

void init_pwm()
{
    // reset PWM
    pwm_reg->ctrl = 0;
    usleep(10);
    pwm_reg->status = -1;
    usleep(10);

    /*
     * set number of bits to transmit
     * e.g, if CLK_MICROS is 5, since we have set the frequency of the
     * hardware clock to 100 MHZ, then the time taken for `100 * CLK_MICROS` bits
     * is (500 / 100) = 5 us, this is how we control the DMA sampling rate
     */
    pwm_reg->range1 = 100 * CLK_MICROS;

    // enable PWM DMA, raise panic and dreq thresholds to 15
    pwm_reg->dma_cfg = PWM_DMAC_ENAB | PWM_DMAC_PANIC(15) | PWM_DMAC_DREQ(15);
    usleep(10);

    // clear PWM fifo
    pwm_reg->ctrl = PWM_CTL_CLRF1;
    usleep(10);

    // enable PWM channel 1 and use fifo
    pwm_reg->ctrl = PWM_CTL_USEF1 | PWM_CTL_MODE1 | PWM_CTL_PWEN1;
}

void dma_start()
{
    // Reset the DMA channel
    dma_reg->cs = DMA_CHANNEL_ABORT;
    dma_reg->cs = 0;
    dma_reg->cs = DMA_CHANNEL_RESET;
    dma_reg->cb_addr = 0;

    dma_reg->cs = DMA_INTERRUPT_STATUS | DMA_END_FLAG;

    // Make cb_addr point to the first DMA control block and enable DMA transfer
    dma_reg->cb_addr = ith_cb_bus_addr(0);
    dma_reg->cs = DMA_PRIORITY(8) | DMA_PANIC_PRIORITY(8) | DMA_DISDEBUG;
    dma_reg->cs |= DMA_WAIT_ON_WRITES | DMA_ACTIVE;
}

void dma_end()
{
    // Shutdown DMA channel, otherwise it won't stop after program exits
    dma_reg->cs |= DMA_CHANNEL_ABORT;
    usleep(100);
    dma_reg->cs &= ~DMA_ACTIVE;
    dma_reg->cs |= DMA_CHANNEL_RESET;
    usleep(100);

    // Release the memory used by DMA, otherwise the memory will be leaked after program exits
    dma_free(dma_ticks);
    dma_free(dma_cbs);

    free(dma_ticks);
    free(dma_cbs);
}

int main ()
{
    uint8_t *dma_base_ptr = map_peripheral(DMA_BASE, PAGE_SIZE);
    dma_reg = (DMACtrlReg *)(dma_base_ptr + DMA_CHANNEL * 0x100);

    pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);

    uint8_t *cm_base_ptr = map_peripheral(CM_BASE, CM_LEN);
    clk_reg = (CLKCtrlReg *)(cm_base_ptr + CM_PWM);

    dma_alloc_buffers();
    usleep(100);

    dma_init_cbs();
    usleep(100);

    init_hw_clk();
    usleep(100);

    init_pwm();
    usleep(100);

    dma_start();
    usleep(100);

    uint32_t ticks[TICK_CNT];
    memcpy(ticks, ith_tick_virt_addr(0), TICK_CNT * sizeof(uint32_t));

    for (size_t i = 0; i < TICK_CNT; i++)
    {
        printf("DMA %d: %u\n", i, ticks[i]);
    }

    dma_end();
    return 0;
}
