/*
 * linker.ld - Script de enlazador robusto para erc2-v
 */

OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
  /* RAM de 4KB, como en el hardware */
  ram (rwx) : ORIGIN = 0x00000000, LENGTH = 4K
}

SECTIONS
{
  /* Sección de código */
  .text : {
    *start.o(.text .text.*)  /* _start must be first */
    *(.text.init)
    *(.text .text.*)
  } > ram

  /* Sección de datos de solo lectura */
  .rodata : {
    . = ALIGN(4);
    *(.rodata .rodata.*)
  } > ram

  /* Sección de datos inicializados */
  .data : {
    . = ALIGN(4);
    *(.data .data.*)
  } > ram

  /* Sección de datos no inicializados (a limpiar a cero) */
  .bss : {
    . = ALIGN(4);
    __bss_start = .;
    *(.bss .bss.*)
    __bss_end = .;
  } > ram

  /* Símbolo para el final de la memoria utilizada por el programa */
  _end = .;
}