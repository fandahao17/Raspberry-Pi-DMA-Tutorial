#include <stdio.h>
#include <stdint.h>

#include "mailbox.h"

#define PAGE_SIZE       4096

#define PERI_BASE       0x7E000000

#define GPIO_BASE       (PERI_BASE + 0x00200000)
#define GPLEV0          13
#define GPIO_LEN        0xF4

#define PWM_BASE        (PERI_BASE + 0x0020C000)
#define PWM_LEN         0x28
#define CLK_PWMCTL      40
#define CLK_PWMDIV      41

/* PWM control bits */
#define PWM_CTL      0
#define PWM_STA      1
#define PWM_DMAC     2
#define PWM_RNG1     4
#define PWM_DAT1     5
#define PWM_FIFO     6
#define PWM_RNG2     8
#define PWM_DAT2     9

#define PWM_CTL_MSEN2 (1<<15)
#define PWM_CTL_PWEN2 (1<<8)
#define PWM_CTL_MSEN1 (1<<7)
#define PWM_CTL_CLRF1 (1<<6)
#define PWM_CTL_USEF1 (1<<5)
#define PWM_CTL_MODE1 (1<<1)
#define PWM_CTL_PWEN1 (1<<0)

#define PWM_DMAC_ENAB      (1 <<31)
#define PWM_DMAC_PANIC(x) ((x)<< 8)
#define PWM_DMAC_DREQ(x)   (x)

#define SYSTIMER_BASE   (PERI_BASE + 0x3000)
#define SYST_LEN        0x1C
#define SYST_CLO        1

#define DMA_BASE        (PERI_BASE + 0x00007000)
#define DMA_CHANNEL     5
#define DMA_OFFSET      0x100
#define DMA_ADDR        (DMA_BASE + DMA_OFFSET * DMA_CHANNEL)

/* DMA CS Control and Status bits */
#define DMA_ENABLE (0xFF0/4)
#define DMA_CHANNEL_RESET       (1<<31)
#define DMA_CHANNEL_ABORT       (1<<30)
#define DMA_WAIT_ON_WRITES      (1<<28)
#define DMA_PANIC_PRIORITY(x) ((x)<<20)
#define DMA_PRIORITY(x)       ((x)<<16)
#define DMA_INTERRUPT_STATUS    (1<< 2)
#define DMA_END_FLAG            (1<< 1)
#define DMA_ACTIVE              (1<< 0)

/* DMA control block "info" field bits */
#define DMA_NO_WIDE_BURSTS          (1<<26)
#define DMA_PERIPHERAL_MAPPING(x) ((x)<<16)
#define DMA_BURST_LENGTH(x)       ((x)<<12)
#define DMA_SRC_IGNORE              (1<<11)
#define DMA_SRC_DREQ                (1<<10)
#define DMA_SRC_WIDTH               (1<< 9)
#define DMA_SRC_INC                 (1<< 8)
#define DMA_DEST_IGNORE             (1<< 7)
#define DMA_DEST_DREQ               (1<< 6)
#define DMA_DEST_WIDTH              (1<< 5)
#define DMA_DEST_INC                (1<< 4)
#define DMA_WAIT_RESP               (1<< 3)

#define NORMAL_DMA (DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP)

#define TIMED_DMA(x) (DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(x))

#define TICKS_PER_PAGE  20
#define LEVELS_PER_PAGE 1000
#define PADDINGS_PER_PAGE 1000

typedef struct DMAChannelHeader
{
    uint32_t cs;            // DMA Channel Control and Status register
    uint32_t conblk_ad;     // DMA Channel Control Block Address
} DMAChannelHeader;

typedef struct DMAControlBlock
{
    uint32_t ti;            // Transfer information
    uint32_t source_ad;     // Source (bus) address
    uint32_t dest_ad;       // Destination (bus) address
    uint32_t txfr_len;      // Transfer length (in bytes)
    uint32_t stride;        // 2D stride
    uint32_t next_conbk;    // Next DMAControlBlock (bus) address
    uint32_t padding[2];    // 2-word padding
} DMAControlBlock;

typedef struct DMACbPage
{
    DMAControlBlock cbs[PAGE_SIZE / sizeof(DMAControlBlock)];
} DMACbPage;

typedef struct DMAResultPage
{
    uint32_t ticks[TICKS_PER_PAGE];
    uint32_t levels[LEVELS_PER_PAGE];
    uint32_t padding[PADDINGS_PER_PAGE];
} DMAResultPage;

typedef struct DMAMemPage
{
    void *virtual_addr;     // Virutal base address of the page     
    uint32_t bus_addr;      // Bus adress of the page, this is not a pointer because it does not point to valid virtual address 
    uint32_t mem_handle;    // Used by mailbox property interface
};


int main() {
    printf("Hello, world!\n");
    return 0;
}