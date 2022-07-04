CFLAGS=-O2 -W -Wall -std=c99 -D_XOPEN_SOURCE=500
LDFLAGS=-lbcm_host

all: dma-demo dma-unpaced dma-paced

dma-demo.o : dma-demo.c
	-$(CC) -c dma-demo.c -o $@ $(CFLAGS)

dma-unpaced.o : dma-unpaced.c
	-$(CC) -c dma-unpaced.c -o $@ $(CFLAGS)

dma-paced.o : dma-paced.c
	-$(CC) -c dma-paced.c -o $@ $(CFLAGS)

mailbox.o : mailbox.c
	-$(CC) -c mailbox.c -o $@ $(CFLAGS)



dma-demo: dma-demo.o mailbox.o
	-$(CC) dma-demo.o mailbox.o -o $@ $(LDFLAGS)
	-chmod 750 dma-demo
	-sudo chown root.gpio dma-demo
	-sudo chmod u+s dma-demo

dma-unpaced: dma-unpaced.o mailbox.o
	-$(CC) dma-unpaced.o mailbox.o -o $@ $(LDFLAGS)
	-chmod 750 dma-unpaced
	-sudo chown root.gpio dma-unpaced
	-sudo chmod u+s dma-unpaced


dma-paced: dma-paced.o mailbox.o
	-$(CC) dma-paced.o mailbox.o -o $@ $(LDFLAGS)
	-chmod 750 dma-paced
	-sudo chown root.gpio dma-paced
	-sudo chmod u+s dma-paced


clean:
	rm -f ./*.o ./*~ ./dma-unpaced ./dma-paced ./dma-demo
