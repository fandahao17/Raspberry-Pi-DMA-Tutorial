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

#define TICK_CNT 20
#define CB_CNT TICK_CNT

#define CLK_MICROS 5

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
    DMAControlBlock cbs[CB_CNT];
} DMACbPage;

typedef struct DMAResultPage
{
    uint32_t ticks[TICK_CNT];
} DMAResultPage;

typedef struct DMAMemHandle
{
    void *virtual_addr; // Virutal base address of the page
    uint32_t bus_addr;  // Bus adress of the page, this is not a pointer because it does not point to valid virtual address
    uint32_t mb_handle; // Used by mailbox property interface
} DMAMemHandle;

int mailbox_fd = -1;
DMAMemHandle dma_cb_pages;
DMAMemHandle dma_result_pages;
volatile DMACtrlReg *dma_reg;

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
    dma_cb_pages = dma_malloc(sizeof(DMACbPage));
    dma_result_pages = dma_malloc(sizeof(DMAResultPage));
}

static inline DMAControlBlock *ith_cb_virt_addr(int i)
{
    return &((DMACbPage *)dma_cb_pages.virtual_addr)->cbs[i];
}

static inline uint32_t ith_cb_bus_addr(int i)
{
    return (uint32_t) & ((DMACbPage *)(uintptr_t)dma_cb_pages.bus_addr)->cbs[i];
}

static inline uint32_t *ith_tick_virt_addr(int i)
{
    return &((DMAResultPage *)dma_result_pages.virtual_addr)->ticks[i];
}

static inline uint32_t ith_tick_bus_addr(int i)
{
    return (uint32_t) & ((DMAResultPage *)(uintptr_t)dma_result_pages.bus_addr)->ticks[i];
}

void dma_init_cbs()
{
    DMAControlBlock *cb;
    for (int i = 0; i < TICK_CNT; i++)
    {
        cb = ith_cb_virt_addr(i);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
        cb->src = PERI_BUS_BASE + SYST_BASE + SYST_CLO * 4;
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

int main()
{
    uint8_t *dma_base_ptr = map_peripheral(DMA_BASE, PAGE_SIZE);
    dma_reg = (DMACtrlReg *)(dma_base_ptr + DMA_CHANNEL * 0x100);

    dma_alloc_pages();
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

    // Should not reach here
    dma_end();
    return 0;
}
