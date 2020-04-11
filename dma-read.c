#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "mailbox.h"

// https://www.raspberrypi.org/forums/viewtopic.php?t=92365
// https://elinux.org/BCM2835_registers#CM
#define PAGE_SIZE 4096

#define PERI_BUS_BASE 0x7E000000
#define PERI_PHYS_BASE 0x3F000000
#define BUS_TO_PHYS(x) ((x) & ~0xC0000000)

#define GPIO_BASE 0x00200000
#define GPLEV0 13
#define GPIO_LEN 0xF4

#define PWM_BASE 0x0020C000
#define PWM_LEN 0x28
#define CLK_PWMCTL 40
#define CLK_PWMDIV 41

/* PWM control bits */
#define PWM_CTL 0
#define PWM_STA 1
#define PWM_DMAC 2
#define PWM_RNG1 4
#define PWM_DAT1 5
#define PWM_FIFO 6
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

#define SYSTIMER_BASE 0x3000
#define SYST_LEN 0x1C
#define SYST_CLO 1

#define DMA_BASE 0x00007000
#define DMA_CHANNEL 5
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
#define DMA_DISDEBUG (1<<28)

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

#define NORMAL_DMA (DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP)

#define TIMED_DMA(x) (DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(x))

#define CB_PAGES_COUNT            1
#define RESULT_PAGES_COUNT        1
#define TICKS_PER_PAGE            20
#define LEVELS_PER_PAGE           1000
#define PADDINGS_PER_PAGE         1000
#define CBS_PER_PAGE              PAGE_SIZE / sizeof(DMAControlBlock)
// ---- Memory allocating defines
// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
#define MEM_FLAG_DIRECT           (1 << 2)
#define MEM_FLAG_COHERENT         (2 << 2)
#define MEM_FLAG_L1_NONALLOCATING (MEM_FLAG_DIRECT | MEM_FLAG_COHERENT)

typedef struct DMAChannelHeader
{
    uint32_t cs;        // DMA Channel Control and Status register
    uint32_t conblk_ad; // DMA Channel Control Block Address
} DMAChannelHeader;

typedef struct DMAControlBlock
{
    uint32_t ti;         // Transfer information
    uint32_t source_ad;  // Source (bus) address
    uint32_t dest_ad;    // Destination (bus) address
    uint32_t txfr_len;   // Transfer length (in bytes)
    uint32_t stride;     // 2D stride
    uint32_t next_conbk; // Next DMAControlBlock (bus) address
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

typedef struct DMAMemPageHandle
{
    void *virtual_addr;  // Virutal base address of the page
    uint32_t bus_addr;   // Bus adress of the page, this is not a pointer because it does not point to valid virtual address
    uint32_t mem_handle; // Used by mailbox property interface
} DMAMemPageHandle;

static int mailbox_fd = -1;
static DMAMemPageHandle dma_cb_pages[CB_PAGES_COUNT];
static DMAMemPageHandle dma_result_pages[RESULT_PAGES_COUNT];
static DMAChannelHeader *dma_channel_hdr;

static DMAMemPageHandle mem_page_alloc() {
    if (mailbox_fd < 0) {
        if ((mailbox_fd = mbox_open()) < 0) {
            fprintf(stderr, "Failed to open /dev/vcio\n");
        }
    }

    DMAMemPageHandle page;
    page.mem_handle = mem_alloc(mailbox_fd, PAGE_SIZE, PAGE_SIZE, MEM_FLAG_L1_NONALLOCATING);
    page.bus_addr = mem_lock(mailbox_fd, page.mem_handle);
    page.virtual_addr = mapmem(BUS_TO_PHYS(page.bus_addr), PAGE_SIZE);

    fprintf(stderr, "Alloc: %6d bytes;  %p (bus=0x%08x, phys=0x%08x)\n",
            (int)PAGE_SIZE, page.virtual_addr, page.bus_addr, BUS_TO_PHYS(page.bus_addr));
    assert(page.bus_addr); // otherwise: couldn't allocate contiguous block.
    memset(page.virtual_addr, 0x00, PAGE_SIZE);
    return page;
}

static void mem_page_free(DMAMemPageHandle *page) {
  if (page->virtual_addr == NULL) return;
  assert(mailbox_fd >= 0);  // someone should've initialized that on allocate.
  unmapmem(page->virtual_addr, PAGE_SIZE);
  mem_unlock(mailbox_fd, page->mem_handle);
  mem_free(mailbox_fd, page->mem_handle);
  page->virtual_addr = NULL;
}

static void *map_peripheral(uint32_t addr, uint32_t size)
{
    int mem_fd;
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
    {
        perror("can't open /dev/mem: ");
        fprintf(stderr, "You need to run this as root!\n");
        return NULL;
    }

    uint32_t *result = (uint32_t *)mmap(
        NULL, // Any adddress in our space will do
        size,
        PROT_READ | PROT_WRITE, // Enable r/w on GPIO registers.
        MAP_SHARED,
        mem_fd,                // File to map
        PERI_PHYS_BASE + addr
    );
    close(mem_fd);

    if (result == MAP_FAILED)
    {
        perror("mmap error");
        return NULL;
    }
    return result;
}

static void dma_alloc_pages() {
    for (size_t i = 0; i < CB_PAGES_COUNT; i++)
    {
        dma_cb_pages[i] = mem_page_alloc();
    }
    for (size_t i = 0; i < RESULT_PAGES_COUNT; i++)
    {
        dma_result_pages[i] = mem_page_alloc();
    }
}

static inline DMAControlBlock *ith_cb_virt_addr(int i) {
    int page = i / CBS_PER_PAGE, index = i % CBS_PER_PAGE;
    return &((DMACbPage *)dma_cb_pages[page].virtual_addr)->cbs[index];
}

static inline uint32_t ith_cb_bus_addr(int i) {
    int page = i / CBS_PER_PAGE, index = i % CBS_PER_PAGE;
    return (uint32_t)&((DMACbPage *)(uintptr_t)dma_cb_pages[page].bus_addr)->cbs[index];
}

static inline uint32_t *ith_tick_virt_addr(int i) {
    int page = i / TICKS_PER_PAGE, index = i % TICKS_PER_PAGE;
    return &((DMAResultPage *)dma_result_pages[page].virtual_addr)->ticks[index];
}

static inline uint32_t ith_tick_bus_addr(int i) {
    int page = i / TICKS_PER_PAGE, index = i % TICKS_PER_PAGE;
    return (uint32_t)&((DMAResultPage *)(uintptr_t)dma_result_pages[page].bus_addr)->ticks[index];
}

static inline uint32_t *ith_level_virt_addr(int i) {
    int page = i / LEVELS_PER_PAGE, index = i % LEVELS_PER_PAGE;
    return &((DMAResultPage *)dma_result_pages[page].virtual_addr)->levels[index];
}

static inline uint32_t ith_level_bus_addr(int i) {
    int page = i / LEVELS_PER_PAGE, index = i % LEVELS_PER_PAGE;
    return (uint32_t)&((DMAResultPage *)(uintptr_t)dma_result_pages[page].bus_addr)->levels[index];
}

static void dma_init_cbs() {
    // DMAControlBlock *cb = &((DMACbPage *)dma_cb_pages[0].virtual_addr)->cbs[0];
    // uint32_t cb_bus_addr = &((DMACbPage *)(uintptr_t)dma_cb_pages[0].bus_addr)->cbs[0];
    DMAControlBlock *cb = ith_cb_virt_addr(0);
    cb->ti = NORMAL_DMA;
    cb->source_ad = PERI_BUS_BASE + SYSTIMER_BASE + SYST_CLO * 4;
    cb->dest_ad = ith_tick_bus_addr(0);
    cb->txfr_len = 4;
    cb->next_conbk = ith_cb_bus_addr(0);
    // printf("Init cb@%8X: Src: %8X, Dest: %8X, nextbk: %8X\n", 
        // cb_bus_addr, cb->source_ad, cb->dest_ad, cb->next_conbk);
}

static void dma_start() {
    dma_channel_hdr->cs = DMA_CHANNEL_ABORT;
    dma_channel_hdr->cs = 0;
    dma_channel_hdr->cs = DMA_CHANNEL_RESET;
    dma_channel_hdr->conblk_ad = 0;

    dma_channel_hdr->cs = DMA_INTERRUPT_STATUS | DMA_END_FLAG;

    dma_channel_hdr->conblk_ad = ith_cb_bus_addr(0);
    dma_channel_hdr->cs = DMA_PRIORITY(8) | DMA_PANIC_PRIORITY(8) | DMA_DISDEBUG;
    dma_channel_hdr->cs |= DMA_WAIT_ON_WRITES | DMA_ACTIVE;
}

static void dma_end() {
    // Shutdown DMA channel.
    dma_channel_hdr->cs |= DMA_CHANNEL_ABORT;
    usleep(100);
    dma_channel_hdr->cs &= ~DMA_ACTIVE;
    dma_channel_hdr->cs |= DMA_CHANNEL_RESET;
}

int main()
{
    printf("Hello, world!\n");
    volatile uint32_t a, b;
    uint32_t *systimer_reg = map_peripheral(SYSTIMER_BASE, SYST_LEN);
    a = systimer_reg[SYST_CLO];
    printf("%u\n", a);
    dma_channel_hdr = map_peripheral(DMA_BASE, PAGE_SIZE);
    dma_alloc_pages();
    dma_init_cbs();
    usleep(100);
    dma_start();
    usleep(100);
    printf("DMA: %u\n", *ith_tick_virt_addr(0));
    printf("DMA: %u\n", *ith_tick_virt_addr(0));
    printf("DMA: %u\n", *ith_tick_virt_addr(0));
    printf("DMA: %u\n", *ith_tick_virt_addr(0));
    printf("DMA: %u\n", *ith_tick_virt_addr(0));
    dma_end();
    return 0;
}
