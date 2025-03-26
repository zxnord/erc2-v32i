AS=riscv-none-embed-as
CFLAGS=-march=rv32i -mabi=ilp32 -mno-relax

LD=riscv-none-embed-ld
LDFLAGS=-m elf32lriscv -nostdlib -no-relax

ULX3S_RAM=262144
ULX3S_ADDR=65535

DEPS = \
	blinker.S	\
	wait.S		\
	start.S

OBJS = \
	blinker.o	\
	wait.o

%.o: %.S
	$(AS) -c -o $@ $< $(CFLAGS)

paso20.elf: $(OBJS)
	$(AS) -c -o start.o start.S $(CFLAGS)
	$(LD) $^ -o $@ -T bram.ld $(LDFLAGS)

paso20.hex: paso20.elf
#	firmware_words blinker.elf -ram $(ULX3S_RAM) -max_addr $(ULX3S_ADDR) -out blinker.bram.hex
	riscv-none-embed-objcopy -O ihex $< $@

.PHONY: clean

clean:
	rm -f *.o *.hex *.elf

all: paso20.hex

