CC=riscv-none-embed-gcc
CCFLAGS=-march=rv32im 								\
	-mabi=ilp32 -mno-relax 							\
	-I /home/zxnord/opt/xpack-riscv-none-embed-gcc/lib/gcc/riscv-none-embed/10.2.0/ 	\
	-O2 											\
	-fno-pic 										\
	-march=rv32im									\
	-mabi=ilp32 									\
	-fno-stack-protector			 				\
	-w -Wl,--no-relax

AS=riscv-none-embed-as
CFLAGS=-march=rv32im -mabi=ilp32 -mno-relax

LD=riscv-none-embed-ld
LDFLAGS=-m elf32lriscv --no-relax --nostdlib

ULX3S_RAM=262144
ULX3S_ADDR=65535

DEPS = \
	wait.S		\
	start.S   \
	putchar.S	\

C_DEPS = \
	print.c       \ 

OBJS = \
			 wait.o			  \
			 print.o			\
			 putchar.o		\
			 sieve.o		\

%.o: %.S
	$(CC) -c -o sieve.o sieve.c $(CCFLAGS)
	$(AS) -c -o $@ $< $(CFLAGS)

paso21.elf: $(OBJS)
	$(AS) -c -o start.o start.S $(CFLAGS)
	$(LD) $^ -o $@ -T bram.ld $(LDFLAGS)

paso21.hex: paso21.elf
	riscv-none-embed-objcopy -O ihex $< $@

paso21_fix.hex: paso21.hex
	python3 firmware_words.py

clean:
	rm -f *.o *.hex *.elf

all: paso21_fix.hex

