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
#define CB_CNT TICK_CNT

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

int mailbox_fd = -1;
DMAMemHandle *dma_cbs;
DMAMemHandle *dma_ticks;
volatile DMACtrlReg *dma_reg;

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
        PERI_PHYS_BASE + addr);

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
        cb = ith_cb_virt_addr(i);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
        cb->src = PERI_BUS_BASE + SYST_BASE + SYST_CLO;
        cb->dest = ith_tick_bus_addr(i);
        cb->tx_len = 4;
        cb->next_cb = ith_cb_bus_addr((i + 1) % CB_CNT);
    }
    fprintf(stderr, "Init: %d cbs, %d ticks\n", CB_CNT, TICK_CNT);
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
}

int main()
{
    uint8_t *dma_base_ptr = map_peripheral(DMA_BASE, PAGE_SIZE);
    dma_reg = (DMACtrlReg *)(dma_base_ptr + DMA_CHANNEL * 0x100);

    dma_alloc_buffers();
    usleep(100);

    dma_init_cbs();
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
