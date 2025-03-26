MEMORY
{
   BRAM (RWX) : ORIGIN = 0x0000, LENGTH = 0x7200  /* 24kB RAM */
}
SECTIONS
{
    everything :
    {
        . = ALIGN(4);
        start.o (.text)
        *(.*)
    } >BRAM
}