/*
 * linker_sw_ptw.ld - Script de enlazador específico para el test 'test_sw_ptw'
 *
 * Este script asegura que las tablas de páginas (L1 y L0) se coloquen en las
 * direcciones físicas exactas que el test de hardware espera.
 */

OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
  ram (rwx) : ORIGIN = 0x00000000, LENGTH = 128K
}

SECTIONS
{
  /* Sección de código, al principio de la RAM */
  .text : {
    *(.text.init)
    *(.text .text.*)
  } > ram

  /*
   * Colocamos la tabla L1 explícitamente en 0x1000.
   * El test configura SATP para que apunte aquí (PPN=1).
   */
  . = 0x1000;
  .l1_page_table : {
    . = ALIGN(4096); /* Sv32 requiere que las tablas estén alineadas a 4KB */
    *(.l1_page_table)
  } > ram

  . = 0x2000;
  .l0_code_page_table : {
    . = ALIGN(4096);
    *(.l0_code_page_table)
  } > ram

  /*
   * Colocamos la tabla L0 para los datos explícitamente en 0x3000.
   * La entrada en la tabla L1 apunta aquí (PPN=3).
   */
  . = 0x3000;
  .l0_data_page_table : {
    . = ALIGN(4096);
    *(.l0_data_page_table)
  } > ram

  /* El resto de las secciones van después del código */
  .rodata : {
    . = ALIGN(4);
    *(.rodata .rodata.*)
  } > ram

  .data : {
    . = ALIGN(4);
    *(.data .data.*)
  } > ram

  .bss : {
    . = ALIGN(4);
    __bss_start = .;
    *(.bss .bss.*)
    __bss_end = .;
  } > ram

  _end = .;
}
