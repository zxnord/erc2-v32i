.section .text
.global _start

_start:
    # Inicializar el stack pointer
    li  sp, 0x1000    # Stack al final de la RAM de 4KB
    
    # Llamar a main()
    call main
    
    # Si main retorna (no debería), detener
    j halt

# Bucle infinito para detener el procesador
halt:
    j halt
