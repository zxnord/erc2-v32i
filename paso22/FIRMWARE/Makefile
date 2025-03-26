CC=riscv-none-embed-gcc
CCFLAGS=-march=rv32im 								\
	-mabi=ilp32 -mno-relax 							\
	-I /home/zxnord/opt/xpack-riscv-none-embed-gcc/lib/gcc/riscv-none-embed/10.2.0/ \ 	\
	-O2 											\
	-fno-pic 										\
	-march=rv32im									\
	-mabi=ilp32 									\
	-fno-stack-protector			 				\
	-w -Wl,--no-relax

AS=riscv-none-embed-as
CFLAGS=-march=rv32im -mabi=ilp32 -I/home/zxnord/opt/xpack-riscv-none-embed-gcc/lib/gcc/riscv-none-embed/10.2.0

LD=riscv-none-embed-ld
LDFLAGS=-m elf32lriscv --no-relax --nostdlib

ULX3S_RAM=262144
ULX3S_ADDR=65535

DEPS = \
	start.S   \
	putchar.S	\

C_DEPS = \
	read_spiflash.c	      \
	print.c       \

OBJS = \
	read_spiflash.o			\
	print.o			\
	putchar.o		\

%.o: %.S
	$(AS) -c -o $@ $< $(CFLAGS)

paso22.elf: $(OBJS)
	$(AS) -c -o start.o start.S $(CFLAGS)
	$(LD) $^ -o $@ -T bram.ld $(LDFLAGS)

paso22.hex: paso22.elf
	riscv-none-embed-objcopy -O ihex $< $@

.PHONY: clean

clean:
	rm -f *.o *.hex *.elf

all: paso22.hex

