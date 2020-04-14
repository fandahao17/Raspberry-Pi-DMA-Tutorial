CFLAGS=-O3 -W -Wall -std=c99 -D_XOPEN_SOURCE=500

all: dma-read
	
dma-read: dma-read.o mailbox.o

test: all
	./dma-read

clean:
	rm -f ./*.o ./dma-read
