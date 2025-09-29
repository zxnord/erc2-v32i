# Makefile para el firmware del procesador erc2-v

# --- Configuración del Compilador ---
# Cambia esto si tu compilador tiene un prefijo diferente
CROSS_COMPILE ?= riscv64-unknown-elf-

TARGET      ?= test_leds

# Herramientas del toolchain
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

# --- Opciones de Compilación ---
# RV32I: Arquitectura base de 32 bits para enteros
# -mabi=ilp32: El ABI (Application Binary Interface) para 32 bits
# -nostdlib: No enlazar con la librería estándar de C, no la necesitamos
# -Tlinker.ld: Script de enlazador para ubicar el código en la dirección 0x00000000
CFLAGS = -march=rv32i -mabi=ilp32 -nostdlib -Tlinker.ld

# --- Reglas del Makefile ---

# El objetivo por defecto es construir el fichero .hex
all: $(TARGET).hex

# Regla para enlazar el objeto y crear el fichero ELF
$(TARGET).elf: $(TARGET).o
	$(CC) $(CFLAGS) -o $@ $^

# Regla para compilar/ensamblar ficheros .S a .o
%.o: %.S
	$(CC) $(CFLAGS) -c -o $@ $<

# Regla para extraer el binario raw del fichero ELF
$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $^ $@

# Regla para convertir el binario a formato Verilog HEX
# El formato es texto plano con números hexadecimales de 32 bits
$(TARGET).hex: $(TARGET).bin
	./bin_to_hex.py $^ $@

# Objetivo para limpiar los ficheros generados
clean:
	rm -f *.o *.elf *.bin *.hex

# Renombrar el fichero final a firmware.hex para que Verilog lo encuentre
.PHONY: firmware
firmware: $(TARGET).hex
	@echo "Copiando $(TARGET).hex a firmware.hex"
	@cp $(TARGET).hex firmware.hex
