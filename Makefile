OBJS=\
jtag_fsm.o\
jtag.o\
loader.o

CFLAGS = -g -Wall -std=c99 -I/usr/include/libftdi1 -D_DEFAULT_SOURCE
LDFLAGS  = -lpthread -lftdi1

alchitry_loader: alchitry_loader.c $(OBJS)
	$(CC) $< -o $@ $(OBJS) $(CFLAGS) $(LDFLAGS)

.PHONY: clean indent scan
clean:
	$(RM) alchitry_loader *.o 

indent:
	clang-format -style=LLVM -i *.c *.h

scan:
	scan-build $(MAKE) clean all

