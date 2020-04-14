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

#define GPIO_BASE 0x00200000
#define GPLEV0 13
#define GPIO_LEN 0xF4

#define CLK_BASE 0x00101000
#define CLK_LEN 0xA8
#define CLK_PWMCTL 40
#define CLK_PWMDIV 41
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
#define PWM_FIFO 6
#define PWM_TIMER (((PWM_BASE + PWM_FIFO * 4) & 0x00ffffff) | PERI_BUS_BASE)

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
#define SYST_CLO 1

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

#define TICKS_PER_PAGE 20
#define LEVELS_PER_PAGE 1000
#define PADDINGS_PER_PAGE 4
#define CBS_PER_PAGE (PAGE_SIZE / sizeof(DMAControlBlock))
#define LEVELS_PER_TICK (LEVELS_PER_PAGE / TICKS_PER_PAGE)

#define BUFFER_MS 100
#define LEVEL_CNT (BUFFER_MS * (1000 / CLK_MICROS)) // Number of `level` entries in buffer
#define RESULT_PAGE_CNT (LEVEL_CNT / LEVELS_PER_PAGE)
#define TICK_CNT (RESULT_PAGE_CNT * TICKS_PER_PAGE)
#define DELAY_CNT LEVEL_CNT
#define CB_CNT (LEVEL_CNT + TICK_CNT + DELAY_CNT)
#define CB_PAGE_CNT ((CB_CNT + CBS_PER_PAGE - 1) / CBS_PER_PAGE)

#define CLK_MICROS 5
#define SLEEP_TIME_MILLIS 5

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

typedef struct DMACbPage
{
    DMAControlBlock cbs[CBS_PER_PAGE];
} DMACbPage;

typedef struct DMAResultPage
{
    uint32_t ticks[TICKS_PER_PAGE];
    uint32_t levels[LEVELS_PER_PAGE];
    uint32_t padding[PADDINGS_PER_PAGE];
} DMAResultPage;

typedef struct DMAMemHandle
{
    void *virtual_addr; // Virutal base address of the page
    uint32_t bus_addr;  // Bus adress of the page, this is not a pointer because it does not point to valid virtual address
    uint32_t mb_handle; // Used by mailbox property interface
} DMAMemHandle;

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
DMAMemHandle dma_cb_pages;
DMAMemHandle dma_result_pages;
volatile DMACtrlReg *dma_reg;
volatile PWMCtrlReg *pwm_reg;
volatile uint32_t *clk_reg;

DMAMemHandle dma_malloc(unsigned int size)
{
    if (mailbox_fd < 0)
    {
        if ((mailbox_fd = mbox_open()) < 0)
        {
            fprintf(stderr, "Failed to open /dev/vcio\n");
        }
    }

    // Make `size` a multiple of PAGE_SIZE
    size = ((size + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;

    DMAMemHandle page;
    // Documentation: https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
    page.mb_handle = mem_alloc(mailbox_fd, size, PAGE_SIZE, MEM_FLAG_L1_NONALLOCATING);
    page.bus_addr = mem_lock(mailbox_fd, page.mb_handle);
    page.virtual_addr = mapmem(BUS_TO_PHYS(page.bus_addr), size);

    // fprintf(stderr, "Alloc: %6u bytes;  %p (bus=0x%08x, phys=0x%08x)\n",
    // size, page.virtual_addr, page.bus_addr, BUS_TO_PHYS(page.bus_addr));
    return page;
}

void dma_free(DMAMemHandle *page)
{
    if (page->virtual_addr == NULL)
        return;

    unmapmem(page->virtual_addr, PAGE_SIZE);
    mem_unlock(mailbox_fd, page->mb_handle);
    mem_free(mailbox_fd, page->mb_handle);
    page->virtual_addr = NULL;
}

void *map_peripheral(uint32_t addr, uint32_t size)
{
    int mem_fd;
    // Check mem(4) about /dev/mem
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
    {
        perror("Failed to open /dev/mem: ");
        return NULL;
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
        return NULL;
    }
    return result;
}

void dma_alloc_pages()
{
    dma_cb_pages = dma_malloc(CB_PAGE_CNT * sizeof(DMACbPage));
    dma_result_pages = dma_malloc(RESULT_PAGE_CNT * sizeof(DMAResultPage));
}

static inline DMAControlBlock *ith_cb_virt_addr(int i)
{
    int page = i / CBS_PER_PAGE, index = i % CBS_PER_PAGE;
    return &((DMACbPage *)dma_cb_pages.virtual_addr)[page].cbs[index];
}

static inline uint32_t ith_cb_bus_addr(int i)
{
    int page = i / CBS_PER_PAGE, index = i % CBS_PER_PAGE;
    return (uint32_t) & ((DMACbPage *)(uintptr_t)dma_cb_pages.bus_addr)[page].cbs[index];
}

static inline uint32_t *ith_tick_virt_addr(int i)
{
    int page = i / TICKS_PER_PAGE, index = i % TICKS_PER_PAGE;
    return &((DMAResultPage *)dma_result_pages.virtual_addr)[page].ticks[index];
}

static inline uint32_t ith_tick_bus_addr(int i)
{
    int page = i / TICKS_PER_PAGE, index = i % TICKS_PER_PAGE;
    return (uint32_t) & ((DMAResultPage *)(uintptr_t)dma_result_pages.bus_addr)[page].ticks[index];
}

static inline uint32_t *ith_level_virt_addr(int i)
{
    int page = i / LEVELS_PER_PAGE, index = i % LEVELS_PER_PAGE;
    return &((DMAResultPage *)dma_result_pages.virtual_addr)[page].levels[index];
}

static inline uint32_t ith_level_bus_addr(int i)
{
    int page = i / LEVELS_PER_PAGE, index = i % LEVELS_PER_PAGE;
    return (uint32_t) & ((DMAResultPage *)(uintptr_t)dma_result_pages.bus_addr)[page].levels[index];
}

void dma_init_cbs()
{
    int tick_idx = 0, level_idx = 0, cb_idx = 0;
    DMAControlBlock *cb;
    for (tick_idx = 0; tick_idx < TICK_CNT; tick_idx++)
    {
        // As time goes on, the cumulative error of PWM-paced delays may become large,
        // so we insert one access to system timer every `LEVELS_PER_TICK` accesses
        // to GPIO in order to correct this error

        // Tick block
        cb = ith_cb_virt_addr(cb_idx);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
        cb->src = PERI_BUS_BASE + SYST_BASE + SYST_CLO * 4;
        cb->dest = ith_tick_bus_addr(tick_idx);
        cb->tx_len = 4;
        cb_idx = (cb_idx + 1) % CB_CNT;
        cb->next_cb = ith_cb_bus_addr(cb_idx);

        for (int i = 0; i < LEVELS_PER_TICK; i++)
        {
            // Level block
            cb = ith_cb_virt_addr(cb_idx);
            cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
            cb->src = PERI_BUS_BASE + GPIO_BASE + GPLEV0 * 4;
            cb->dest = ith_level_bus_addr(level_idx++);
            cb->tx_len = 4;
            cb_idx = (cb_idx + 1) % CB_CNT;
            cb->next_cb = ith_cb_bus_addr(cb_idx);

            // Delay block
            cb = ith_cb_virt_addr(cb_idx);
            cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(5);
            cb->src = ith_cb_bus_addr(0);
            cb->dest = PWM_TIMER;
            cb->tx_len = 4;
            cb_idx = (cb_idx + 1) % CB_CNT;
            cb->next_cb = ith_cb_bus_addr(cb_idx);
        }
    }
    fprintf(stderr, "Init: %d cbs, %d levels, %d ticks\n", CB_CNT, LEVEL_CNT, TICK_CNT);
}

void init_hw_clk()
{
    // See Chanpter 6.3, BCM2835 ARM peripherals for controlling the hardware clock

    // kill the clock if busy
    size_t clkCtl = CLK_PWMCTL, clkDiv = CLK_PWMDIV, divI = 5;
    if (clk_reg[clkCtl] & CLK_CTL_BUSY)
    {
        do
        {
            clk_reg[clkCtl] = BCM_PASSWD | CLK_CTL_KILL;
        } while (clk_reg[clkCtl] & CLK_CTL_BUSY);
    }

    // The original clock speed is 500MHZ, we divide it by 5 to get a 100MHZ clock
    clk_reg[clkDiv] = BCM_PASSWD | CLK_DIV_DIVI(divI);
    usleep(10);

    // Set clock source to plld
    clk_reg[clkCtl] = BCM_PASSWD | CLK_CTL_SRC(CLK_CTL_SRC_PLLD);
    usleep(10);

    // Enable the clock
    clk_reg[clkCtl] |= (BCM_PASSWD | CLK_CTL_ENAB);
}

void dma_init_clock()
{
    init_hw_clk();

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
    // Shutdown DMA channel.
    dma_reg->cs |= DMA_CHANNEL_ABORT;
    usleep(100);
    dma_reg->cs &= ~DMA_ACTIVE;
    dma_reg->cs |= DMA_CHANNEL_RESET;
    usleep(100);

    // Release the memory used by DMA
    dma_free(&dma_result_pages);
    dma_free(&dma_cb_pages);
}

static inline uint32_t get_cur_level_idx()
{
    // Which DMA control block are we at?
    uint32_t cb = (dma_reg->cb_addr - dma_cb_pages.bus_addr) / sizeof(DMAControlBlock);

    // Which `level` entry are we sampling?
    uint32_t block = cb / (1 + 2 * LEVELS_PER_TICK);
    uint32_t index = cb % (1 + 2 * LEVELS_PER_TICK);
    return block * LEVELS_PER_TICK + (index > 1 ? index - 1 : index) / 2;
}

void monitor_gpios()
{
    printf("Enter thread\n");
    uint32_t cur_level = 0, cur_idx, old_idx = 0, cur_time = 0;
    while (1)
    {
        cur_idx = get_cur_level_idx();

        // `old_idx` is the index of `level` entry we have processed
        // `new_idx` is the index of `level` entry the DMA engine have sampled
        while (old_idx != cur_idx)
        {
            if (old_idx % LEVELS_PER_TICK == 0)
            {
                // As time goes on, the cumulative error of PWM-paced delays may become large,
                // so we insert one access to system timer every `LEVELS_PER_TICK` accesses
                // to GPIO in order to correct this error
                cur_time = *ith_tick_virt_addr(old_idx / LEVELS_PER_TICK);
            }
            uint32_t level = *ith_level_virt_addr(old_idx) & ~0xF0000000;
            if (level != cur_level)
            {
                fprintf(stderr, "Level change @%u: %08X\n", cur_time, level);
                cur_level = level;
            }
            cur_time += CLK_MICROS; // It will wrap around itself
            old_idx = (old_idx + 1) % LEVEL_CNT;
        }
        usleep(SLEEP_TIME_MILLIS * 1000);
    }
}

void sigint_handler(int signo)
{
    if (signo == SIGINT)
    {
        fprintf(stderr, "Ending GPIO monitoring!\n");
        dma_end();
        exit(0);
    }
}

int main()
{
    signal(SIGINT, sigint_handler);

    uint8_t *dma_base_ptr = map_peripheral(DMA_BASE, PAGE_SIZE);
    dma_reg = (DMACtrlReg *)(dma_base_ptr + DMA_CHANNEL * 0x100);

    pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
    clk_reg = map_peripheral(CLK_BASE, CLK_LEN);

    dma_alloc_pages();
    usleep(100);

    dma_init_cbs();
    usleep(100);

    dma_init_clock();
    usleep(100);

    dma_start();
    usleep(100);

    // Start monitoring GPIOs
    monitor_gpios();

    // Should not reach here
    dma_end();
    return 0;
}
