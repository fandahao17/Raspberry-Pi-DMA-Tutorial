CFLAGS=-O2 -W -Wall -std=c99 -D_XOPEN_SOURCE=500

dma-demo: dma-demo.o mailbox.o

dma-unpaced: dma-unpaced.o mailbox.o

dma-paced: dma-paced.o mailbox.o

clean:
	rm -f ./*.o ./dma-unpaced ./dma-paced ./dma-demo
