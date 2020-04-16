# A simple tutorial on using Raspberry Pi DMA channels

Suppose you need to monitor the level changes of your Pi's GPIOs and report them as soon as possible. This could be as simple as a single *interrupt* if you are programming on *bare metal*. However, things are much more complicated if you are working on Pi's Linux system with tens of other processes running. To avoid your program from occupying all the CPU cycles, you need to use **DMA**.

I found that there actually exists very few tutorial-like resources on how to use Raspberry Pi's DMA channel properly. So I wrote this article for those who need to control DMA channels themselves, or (like me) are just curious about how great tools like [pigpio](http://abyz.me.uk/rpi/pigpio/index.html) achieves high-speed sampling while consuming surprisingly low CPU resources. I'm not an expert in Raspberry Pi, so I **welcome any comments or suggestions** so I can improve this tutorial.

**In this tutorial**, we will first control the DMA to continuously fetch the BCM2835 system timer, then we will pace DMA accesses with the PWM peripheral so we can designate a fixed DMA sample rate. I choose the system timer here because it increments every microsecond, so we can easily check whether our code is working correctly. Finally, I'm going to present a demo that uses DMA to monitor GPIO level changes at the accuracy of 1 us.

## Background knowledge

As described in [BCM2835-ARM-Peripherals](https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf) (the "*datasheet*"), There are three types of addresses in Raspberry Pi:

* *ARM Virtual Address*: The address used in the virtual address space of a Linux process.
* *ARM Physical Address*: The address used when accessing RAM. Since peripherals on BCM2835 are [memory-mapped](https://en.wikipedia.org/wiki/Memory-mapped_I/O), this address is used to access peripherals directly.
* *Bus Address*: This is the address used by the DMA engine.

Note that the *physical addresses* of the peripherals range from *0x3F000000* to *0x3FFFFFFF* and are mapped onto *bus address* range *0x7F000000* to *0x7FFFFFFF*, like this which we'll use later:

``` c
#define BUS_TO_PHYS(x) ((x) & ~0xC0000000
```

The BCM2835 has 16 DMA channels. The *DMA channel controller registers* start at *bus address* 0x7E007000, with adjacent channels offset by 0x100. Detailed information about the DMA controller can be found in the *datasheet*.

The *DMA control blocks* on BCM2835 are organized as a *linked list*, with the pivot being *DMA channel controller registers*. Note that all "pointer"s in the following image are **bus address**es.

<img src="./img/dma_demo.png">

## Mapping peripherals into virtual memory

As metioned above, peripherals can be accessed by user programs with their *physical address*. In order to configure the DMA channel, we need to bring the DMA controller registers into *virtual memory*.

The way we access these memory-mapped peripherals is to `mmap` them from the `/dev/mem` interface. We can wrap this procedure inside a function, note that I omitted error handling code here for clarity.

``` c
void *map_peripheral(uint32_t peri_offset, uint32_t size)
{
    // Check mem(4) man page for "/dev/mem"
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);

    uint32_t *result = (uint32_t *)mmap(
        NULL,
        size,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        PERI_PHYS_BASE + peri_offset); // PERI_PHYS_BASE defined as 0x3F00000

    close(mem_fd);

    return result;
}
```

With this defined, we can define. Also, remember to use [`volatile`](https://en.wikipedia.org/wiki/Volatile_(computer_programming)) when accessing peripherals!

``` c
volatile uint8_t *dma_base_ptr = map_peripheral(DMA_BASE, PAGE_SIZE);
// We use DMA channel 5 here, a little pointer arithmetic is required
volatile dma_channel_hdr = (DMAChannelHeader *)(dma_base_ptr + DMA_CHANNEL * 0x100);
```

## Allocating DMA Control Blocks and Result Buffer

The next step is to allocate our control blocks and a buffer to store DMA results. What's special about this is that since DMA accesses uses *bus address*, we at least need to know the *physical address* of our allocated memory region.

A straightforward way to get physical addresses out of virtual addresses would be using Linux's [`pagemap`](https://www.kernel.org/doc/Documentation/vm/pagemap.txt) interface. However, this seems to be reliable because we can't guarantee the memory we have is [cache coherent](https://en.wikipedia.org/wiki/Direct_memory_access#Cache_coherency), which is **crucial to proper DMA**. Also, the physical address backing a certain virtual address may be **subject to change**. Thus, **I do not recommend using** `pagemap`.

What comes to the rescue here is the [mailbox property interface](https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface). As it's developed for communication between the ARM and the GPU, which I guess also suffers from cache coherency issues, it provides ways to [allocate](https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#allocate-memory) contiguous memory on the **coherent** portion of L2 cache & RAM and [lock](https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface#lock-memory) it on a fixed bus address. It looks perfectly suitable for DMA, so it's becoming a standard way for projects adopting DMA. We are gonna use it, too.

We use an abstraction to represent a memory block.

``` c
typedef struct DMAMemHandle
{
    void *virtual_addr; // Virutal base address of the memory block
    uint32_t bus_addr;  // Bus address of the memory block
    uint32_t mb_handle; // Used internally by mailbox property interface
} DMAMemHandle;
```

I've included a mailbox implementation in this repository, with which we can implement `malloc` and `free`-like memory management. You may need to check its **permissive** license if you are to use it.

``` c
// `int mailbox_fd = mbox_open()` somewhere else
DMAMemHandle dma_malloc(unsigned int size
{
    // Error handling code omitted, see dma-unpaced.c for full code
    DMAMemHandle page;
    page.mb_handle = mem_alloc(mailbox_fd, size, PAGE_SIZE, MEM_FLAG_L1_NONALLOCATING);
    page.bus_addr = mem_lock(mailbox_fd, page.mb_handle);
    page.virtual_addr = mapmem(BUS_TO_PHYS(page.bus_addr), size);

    return page;
}

void dma_free(DMAMemHandle *page)
{
    // Error handling code omitted, see dma-unpaced.c for full code
    unmapmem(page->virtual_addr, PAGE_SIZE);
    mem_unlock(mailbox_fd, page->mb_handle);
    mem_free(mailbox_fd, page->mb_handle);
    page->virtual_addr = NULL;
}
```

## Starting DMA transfer 

As the mailbox `mem_alloc` function requires alignment by page, I specified the page structure for my control blocks and result buffer:

``` c
#define PAGE_SIZE 4096 // Normal page size of Pi
typedef struct DMACbPage
{
    DMAControlBlock cbs[PAGE_SIZE / sizeof(DMAControlBlock)];
} DMACbPage;

typedef struct DMAResultPage
{
    uint32_t ticks[PAGE_SIZE / sizeof(uint32_t)];
} DMAResultPage;
```

This definition allows us to get the *virtual* or *bus address* of the `ith` control block through *C struct indexing*.

``` c
static inline DMAControlBlock *ith_cb_virt_addr(int i)
{
    int page = i / CBS_PER_PAGE, index = i % CBS_PER_PAGE;
    return &((DMACbPage *)dma_cb_pages.virtual_addr)[page].cbs[index];
}

static inline uint32_t ith_cb_bus_addr(int i)
{
    int page = i / CBS_PER_PAGE, index = i % CBS_PER_PAGE;
    DMACbPage *page_base = (DMACbPage *)(uintptr_t)dma_cb_pages.bus_addr;
    // Note the trick here: Although bus address is not valid in virtual address space, 
    // we can still exploit struct indexing to calculate the address of one of its members
    return (uint32_t) & page_base[page].cbs[index];
}
```

It's quite straightforward to initialize the *DMA control block linked lis*t. Here is the code for setting up the *ith* control block:

``` c
DMAControlBlock *cb = ith_cb_virt_addr(i);
cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP; // Wait until receiving write sponse
cb->src = PERI_BUS_BASE + SYSTIMER_BASE + SYST_CLO * 4;  // Bus address of the lower word of system timer
cb->dest = ith_tick_bus_addr(i);
cb->tx_len = 4; // 32 bits = 4 bytes
cb->next_cb = ith_cb_bus_addr((i + 1) % NUM_CBS);  // A circular list here to do DMA forever
```

After that, let the *DMA controller register* (the "pivot") to point to the first *DMA control block* and set the `active` bit in the `CS` field to start DMA transfer.

``` c
dma_reg->cb_addr = ith_cb_bus_addr(0); // Point to the first control block
dma_reg->cs = DMA_PRIORITY(8) | DMA_PANIC_PRIORITY(8) | DMA_DISDEBUG;  // Sets AXI priority level
dma_reg->cs |= DMA_WAIT_ON_WRITES | DMA_ACTIVE; // Start DMA transfer
```

You could find code until now in[ `dma-unpaced.c`](./dma-unpaced.c). Now that the DMA starts, we could see the results of 20 consecutive fetches to the system timer. On my Pi, the output happens to be: 

``` 
$ sudo ./dma-unpaced
Init: 20 cbs, 20 ticks
DMA 0: 1616166277
DMA 1: 1616166277
DMA 2: 1616166278
DMA 3: 1616166273
DMA 4: 1616166274
DMA 5: 1616166274
DMA 6: 1616166274
DMA 7: 1616166274
DMA 8: 1616166275
DMA 9: 1616166275
DMA 10: 1616166275
DMA 11: 1616166275
DMA 12: 1616166276
DMA 13: 1616166276
DMA 14: 1616166276
DMA 15: 1616166276
DMA 16: 1616166276
DMA 17: 1616166277
DMA 18: 1616166277
DMA 19: 1616166277
```

## Pacing DMA accesses with *DREQ*

It seems that the speed of DMA accesses to the system timer is about *4~5 MHZ*, which is decent. However, most of the times we hope to do DMA accesses **at timed intervals**, so we can control the DMA *sampling rate* (or *write speed* for output). Fortunately, we could pace DMA accesses with the **DREQ** mechanism (documented in Section 4.2.1.3 in the *datasheet*), although it's a little tricky.

We do this by inserting a "delay" block between two DMA "acesses" blocks. **The idea is**: the "delay" block controls the DMA engine to feed some dummy data to the **PWM** FIFO, then the PWM controller reads from its FIFO and "sends" this data out. The DMA engine keeps waiting until the PWM finishes sending and sets the *DREQ* signal to high. As we could control the number of *clock cycles* the PWM controller takes to send the data, we could delay the DMA engine for the duration we want.

You could see that exactly what data to be sent to the PWM FIFO doesn't really matter. *We just use PWM to generate an accurate delay*. If you are unclear of how the PWM controller works, check Section 9.4 of the *datasheet*. Also note that **PCM** can do basically the same thing, but we'll use PWM for this tutorial. Another thing to notice is that the *clock* metioned last paragraph is the clock generated by the **clock manager**. Check *Section 6.3* of the *datasheet* for more information. 

In my example, I choose the 500-MHz **PLLD** here as our PWM clock source, as it's [unlikely to change](https://raspberrypi.stackexchange.com/questions/1153/what-are-the-different-clock-sources-for-the-general-purpose-clocks). I divide it by *5* to get a 100-Mhz clock and set `range` field of PWM controller to *100* so it takes 100 cycles to send the data, which yields a total duration of 1us. 

<img src="./img/dma_delay.png" width="500">

Here is a simplified version of the code used to control the PWM, note that I have omitted the code to enable the clock and PWM controller.

``` c
clk_reg[CLK_PWMCTL] = BCM_PASSWD | CLK_CTL_SRC(CLK_CTL_SRC_PLLD); // Clock source 
clk_reg[CLK_PWMDIV] = BCM_PASSWD | CLK_DIV_DIVI(5); // Divide by 5, 100 Mhz now

pwm_reg->range1 = 100 * 1; // Send for 100 cycles
```

Configuring the "delay" control block is roughly the same as before, except for the following fields: 

``` c
cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(5); // Tell DMA to use DREQ
cb->src = ith_cb_bus_addr(0); // Dummy data
cb->dest = PERI_BUS_BASE + PWM_BASE + PWM_FIFO * 4; // Send to PWM FIFO
```

The complete code with paced DMA accesses can be found in [dma-paced.c](./dma-paced.c). You can `make paced` and run the program yields the following output on my Pi:

```
$ sudo ./dma-paced
Init: 40 cbs, 20 ticks
DMA 0: 2855359907
DMA 1: 2855359908
DMA 2: 2855359909
DMA 3: 2855359910
DMA 4: 2855359911
DMA 5: 2855359912
DMA 6: 2855359913
DMA 7: 2855359914
DMA 8: 2855359915
DMA 9: 2855359916
DMA 10: 2855359917
DMA 11: 2855359898
DMA 12: 2855359899
DMA 13: 2855359900
DMA 14: 2855359901
DMA 15: 2855359902
DMA 16: 2855359903
DMA 17: 2855359904
DMA 18: 2855359905
DMA 19: 2855359906
```

We could see that the DMA accesses are done every microsecond, cool!

## Final example: us-level GPIO monitoring

Now that we've paced our DMA accesses at a fixed sampling rate, we could do something more useful than checking the system timer. In this final example, I present a demo that uses literally the same DMA technique introduced in last section to watch the levels of GPIO 0-27 every microsecond and on detected level change, report the new GPIO levels as well as when (in us) the level change took place. The code can be found at [dma-demo.c](./dma-demo.c).

A main loop is waked up every 5 ms to process those GPIO levels that has been recently sampled into the ring buffer. In case the process would be switched out by the Linux scheduler for a long time, I allocated a huge buffer that could store samples in the last 100 ms. The ring buffer is accessed in a straightforward way:

``` c
// `old_idx` is the index of the latest `level` entry we have processed
// `cur_idx` is the index of the latest `level` entry the DMA engine have sampled
while (1)
{
     // Get current DMA CB index in dma_reg->cb_addr, then calculate the current level entry index
    cur_idx = get_cur_level_idx();
    while (old_idx != cur_idx)
    {
        uint32_t level = *ith_level_virt_addr(old_idx) & ~0xF0000000; // Only GPIO 0-27
        if (level != cur_level)
        {
            fprintf(stderr, "Level change @%u: %08X\n", cur_time, level);
            cur_level = level;
        }
        cur_time += CLK_MICROS; // It will wrap around itself
        old_idx = (old_idx + 1) % LEVEL_CNT; // LEVEL_CNT is total num of samples in the buffer
    }
    usleep(SLEEP_TIME_MILLIS * 1000); // SLEEP_TIME_MILLIS is 5, but could be smaller
}
```

I noticed that in one second, the cumulative error of PWM-generated delays could be **more than ten microseconds**. So I inserted one timestamp,i.e, one DMA access to the system timer, after every 50 accesses to the GPIO level register to correct this error, making my `DMAResultPage` like this:

``` c
#define TIMESTAMPS_PER_PAGE 20
#define LEVELS_PER_PAGE 1000
#define PADDINGS_PER_PAGE 4

typedef struct DMAResultPage
{
    uint32_t timestamps[TIMESTAMPS_PER_PAGE];
    uint32_t levels[LEVELS_PER_PAGE];
    uint32_t padding[PADDINGS_PER_PAGE];
} DMAResultPage
```

Now compile this code with `make dma-demo`  and run with `sudo ./dma-demo`, then if you have pigpio installed, open another terminal and try toggling one of the GPIOs:

``` python
>>> import pigpio
>>> pi = pigpio.pi()
>>> pi.write(4, 1)
>>> pi.write(4, 0)
```

You could see that the level changes are detected immediately as you type in the python commands. Also check `htop` and you will see that the CPU usage of our demo program is just as low as around **20%**. With a lower sample rate like 5 us, the CPU usage is almost negligible.

## Conlusions and References

As is mentioned at beginning of this article, there are lots of great code examples that use DMA. [pigpio](http://abyz.me.uk/rpi/pigpio/index.html) is the one that use most, and it provides DMA memory allocation implementaions using both `pagemap` and mailbox, as well as DMA delay implementations with both PCM and PWM. I haven't used [ServoBlaster](https://github.com/richardghirst/PiBits/tree/master/ServoBlaster) before, but it is probably the first to use PWM to generate accurate DMA delay, and [this stackoverflow post](https://stackoverflow.com/questions/50427275/raspberry-how-does-the-pwm-via-dma-work) may help you understand its code. [Wallacoloo's example](https://github.com/Wallacoloo/Raspberry-Pi-DMA-Example) (a bit outdated) and [hezller's demo](https://github.com/hzeller/rpi-gpio-dma-demo) are also great references.

