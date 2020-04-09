#include <stdio.h>
#include <stdint.h>

#include "mailbox.h"

#define PAGE_SIZE 4096

#define PERI_BASE 0x7E000000


#define DMA_BASE  0x00007000
#define DMA_CHANNEL 5
#define DMA_OFFSET  0x100

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