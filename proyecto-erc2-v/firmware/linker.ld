/*
 * linker.ld - Script de enlazador para erc2-v
 */

OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
  /* La memoria RAM empieza en 0x0 y tiene 16KB de tamaño */
  ram (rwx) : ORIGIN = 0x00000000, LENGTH = 16K
}

SECTIONS
{
  /* La sección de código (.text) se coloca al principio de la RAM */
  .text :
  {
    . = ORIGIN(ram);
    *(.text .text.*)
  } > ram
}
