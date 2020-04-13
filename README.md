# A simple guide on using RasPi DMA channels

Suppose you need to monitor the level changes of your Pi's GPIOs and report them as soon as possible. This could be as simple as a single *interrupt* if you are programming on *bare metal*. However, things are much more complicated if you are working on Pi's Linux system with tens of other processes running. To avoid your program from occupying all the CPU cycles, you need to use **DMA**.

I found that there actually exists very few resources on how to use Raspberry Pi's DMA channel properly. [Pigpio](http://abyz.me.uk/rpi/pigpio/index.html) uses DMA accesses to achieve microsecond-level sampling, but as the project is fairly large now it's not very suitable for beginners. However, pigpio, along with [Wallacoloo's example](https://github.com/Wallacoloo/Raspberry-Pi-DMA-Example) (a little old) and [hezller's demo](https://github.com/hzeller/rpi-gpio-dma-demo) are great references when you write code to control the DMA channels.

## Background knowledge

As described in [BCM2835-ARM-Peripherals](https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf) (the "*datasheet*"), There are three types of addresses in Raspberry Pi:

* *ARM Virtual Address*: The address used in the virtual address space of a Linux process.
* *ARM Physical Address*: The address used when accessing RAM. Since peripherals on BCM2835 are [memory-mapped](https://en.wikipedia.org/wiki/Memory-mapped_I/O), this address is used to access peripherals directly.
* *Bus Address*: This is the address used by the DMA engine.

Note that the *physical addresses* of the peripherals range from *0x3F000000* to *0x3FFFFFFF* and are mapped onto *bus address* range *0x7F000000* to *0x7FFFFFFF*.

The BCM2835 has 16 DMA channels. The *DMA channel controller registers* start at *bus address* 0x7E007000, with adjacent channels offset by 0x100.

The *DMA control blocks* on BCM2835 are organized as a *linked list*, with the pivot being *DMA channel controller registers*. Note that all "pointer"s in the following image are **bus address**es.

![Screen Shot 2020-04-12 at 3.44.00 PM](img/dma_demo.png)

Detailed information about the DMA controller can be found in the datasheet.

## Mapping peripherals into virtual memory

As metioned above, peripherals can be accessed by user programs with their *physical address*. In order to configure the DMA channel, we need to bring the DMA controller registers into *virtual memory*.

The way we access these memory-mapped peripherals is to `mmap` the `/dev/mem` interface. We can wrap this process inside a function, note that I omitted error handling code here for clarity.

``` c
void *map_peripheral(uint32_t peri_offset, uint32_t size)
{
    // Check mem(4) for "/dev/mem"
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

Note that the `offset` parameter of `mmap` requires alignment by page, so you may nees to do some pointer arithmetic when mapping these peripherals.

``` c
uint8_t *dma_base_ptr = map_peripheral(DMA_BASE, PAGE_SIZE);
// We use DMA channel 5 here, so DMA_CHANNEL defined as 5
dma_channel_hdr = (DMAChannelHeader *)(dma_base_ptr + DMA_CHANNEL * 0x100);
```

## Allocating DMA Control Blocks and Result Buffer

The next step is to allocate our control blocks and a buffer to store DMA results. What's special about this is that since DMA accesses uses *bus address*, we at least need to know the *physical address* of our allocated memory region.

A natural way to get physical addresses out of virtual addresses would be using Linux's [`pagemap`](https://www.kernel.org/doc/Documentation/vm/pagemap.txt) interface. However, this is not reliable because   we can't guarantee the memory we have is [cache coherent](https://en.wikipedia.org/wiki/Direct_memory_access#Cache_coherency), which is **crucial to proper DMA**. Also, the physical address backing a certain virtual address may be **subject to change**. Thus, it's not recommended to use `pagemap`.

[This](https://github.com/Wallacoloo/Raspberry-Pi-DMA-Example/blob/f0d5043eebce06ef5d35526e6ca16f1638c4081c/dma-gpio.c#L44) post is 

The mailbox property interface documentation suggests that memory sections with 0x8 and 0xc alias seem to be coherent when using DMA.

## Starting DMA transfer 

It's quite straightforward to initialize the *DMA control block linked lis*t. Here is the code for setting up the $i^{th}$ control block:

``` c
DMAControlBlock *cb = ith_cb_virt_addr(i);
cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
cb->src = PERI_BUS_BASE + SYSTIMER_BASE + SYST_CLO * 4;  // Bus address of the lower word of system timer
cb->dest = ith_tick_bus_addr(i);
cb->tx_len = 4;
cb->next_cb = ith_cb_bus_addr((i + 1) % NUM_CBS);  // A circular list here to do DMA indefinitely
```

After that, let the *DMA controller register* (the "pivot") to point to the first *DMA control block* and set the `active` bit in the `CS` field.

You could code until now in[ `FILE`](). Now that the DMA starts, we could see the results of 20 consecutive fetches to the system timer:

``` 
result
```

## Pacing DMA accesses with *DREQ*



## Final example: Monitoring GPIO changes at the us-level accuracy

