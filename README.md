# A simple guide on using RasPi DMA channels

Suppose you need to monitor the level changes of your Pi's GPIOs and report them as soon as possible. This could be as simple as a single *interrupt* if you are programming on *bare metal*. However, things are much more complicated if you are working on Pi's Linux system with tens of other processes running. To avoid your program from occupying all the CPU cycles, you need to use **DMA**.

I found that there actually exists very few resources on how to use Raspberry Pi's DMA channel properly. [Pigpio](http://abyz.me.uk/rpi/pigpio/index.html) uses DMA accesses to achieve microsecond-level sampling, but as the project is fairly large now it's not very suitable for beginners. However, pigpio, along with [Wallacoloo's example](https://github.com/Wallacoloo/Raspberry-Pi-DMA-Example) (a little old) and [hezller's demo](https://github.com/hzeller/rpi-gpio-dma-demo) are great references when you write code to control the DMA channels.

## Background knowledge

As described in [BCM2835-ARM-Peripherals](https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf) (the "*datasheet*"), There are three types of addresses in Raspberry Pi:

* *ARM Virtual Address*: The address used in the virtual address space of a Linux process.
* *ARM Physical Address*: The address used when accessing RAM. Since peripherals on BCM2835 are [memory-mapped](https://en.wikipedia.org/wiki/Memory-mapped_I/O), this address is used to access peripherals directly.
* *Bus Address*: This is the address used by the DMA engine.

Note that the *physical addresses* of the peripherals range from *0x3F000000* to *0x3FFFFFFF* and are mapped onto *bus address* range *0x7F000000* to *0x7FFFFFFF*.

The *DMA control blocks* on BCM2835 are organized as a *linked list*, with the pivot being *DMA channel controller registers*. Note that all "pointer"s in the following image are **bus address**es.

![Screen Shot 2020-04-12 at 3.44.00 PM](img/dma_demo.png)

Detailed information about the DMA controller can be found in the datasheet.

## Mapping peripherals into virtual memory with *mmap()*



## Allocating DMA memory with *pagemap* or *mailbox*



## Starting DMA transfer 



## Pacing DMA accesses with *DREQ*



## Final example: Monitoring GPIO changes at the us-level accuracy

