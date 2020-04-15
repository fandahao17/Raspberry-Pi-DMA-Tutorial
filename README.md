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

Note that the `offset` parameter of `mmap` requires alignment by page, so you may nees to do some pointer arithmetic when mapping these peripherals. Also, remember to use [`volatile`](https://en.wikipedia.org/wiki/Volatile_(computer_programming)) when accessing peripherals!

``` c
volatile uint8_t *dma_base_ptr = map_peripheral(DMA_BASE, PAGE_SIZE);
// We use DMA channel 5 here, so DMA_CHANNEL defined as 5
dma_channel_hdr = (DMAChannelHeader *)(dma_base_ptr + DMA_CHANNEL * 0x100);
```

## Allocating DMA Control Blocks and Result Buffer

The next step is to allocate our control blocks and a buffer to store DMA results. What's special about this is that since DMA accesses uses *bus address*, we at least need to know the *physical address* of our allocated memory region.

A natural way to get physical addresses out of virtual addresses would be using Linux's [`pagemap`](https://www.kernel.org/doc/Documentation/vm/pagemap.txt) interface. However, this is not reliable because   we can't guarantee the memory we have is [cache coherent](https://en.wikipedia.org/wiki/Direct_memory_access#Cache_coherency), which is **crucial to proper DMA**. Also, the physical address backing a certain virtual address may be **subject to change**. Thus, it's not recommended to use `pagemap`.

[This](https://github.com/Wallacoloo/Raspberry-Pi-DMA-Example/blob/f0d5043eebce06ef5d35526e6ca16f1638c4081c/dma-gpio.c#L44) post is 

The mailbox property interface documentation suggests that memory sections with 0x8 and 0xc alias seem to be coherent when using DMA.

## Starting DMA transfer 

It's quite straightforward to initialize the *DMA control block linked lis*t. Here is the code for setting up the *ith* control block:

``` c
DMAControlBlock *cb = ith_cb_virt_addr(i);
cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP; // Wait until receiving write sponse
cb->src = PERI_BUS_BASE + SYSTIMER_BASE + SYST_CLO * 4;  // Bus address of the lower word of system timer
cb->dest = ith_tick_bus_addr(i);
cb->tx_len = 4; // 32 bits = 4 bytes
cb->next_cb = ith_cb_bus_addr((i + 1) % NUM_CBS);  // A circular list here to do DMA forever
```

After that, let the *DMA controller register* (the "pivot") to point to the first *DMA control block* and set the `active` bit in the `CS` field.

``` c
dma_reg->cb_addr = ith_cb_bus_addr(0); // Point to the first control block
dma_reg->cs = DMA_PRIORITY(8) | DMA_PANIC_PRIORITY(8) | DMA_DISDEBUG;  // Sets AXI priority level
dma_reg->cs |= DMA_WAIT_ON_WRITES | DMA_ACTIVE; // Start DMA transfer
```

You could find code until now in[ `dma-unpaced.c`](./dma-unpaced.c). Now that the DMA starts, we could see the results of 20 consecutive fetches to the system timer. On my RPi, the output happens to be: 

``` 
$ ./dma-unpaced
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

![dma-delay](img/dma_delay.png)

Here is a simplified version of the code used to control the PWM, note that I have omitted the code to enable the clock and PWM controller.

``` c
clk_reg[CLK_PWMCTL] = BCM_PASSWD | CLK_CTL_SRC(CLK_CTL_SRC_PLLD); // Clock source 
clk_reg[CLK_PWMDIV] = BCM_PASSWD | CLK_DIV_DIVI(5); // Divide by 5, 100 Mhz now

pwm_reg->range1 = 100 * 1; // Send for 100 cycles
```

Configuring the "delay" control block is roughly the same as before, except the following field: 

``` c
cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(5); // Configure for DREQ
cb->src = ith_cb_bus_addr(0); // Dummy data
cb->dest = PERI_BUS_BASE + PWM_BASE + PWM_FIFO * 4; // Send to PWM FIFO
```

The complete code with paced DMA accesses can be found in [dma-paced.c](./dma-paced.c). You can `make paced` and run the program yields this output on my Pi:

```
$ ./dma-paced
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

## Final example: Monitoring GPIO changes at the us-level accuracy

