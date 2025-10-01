# start.S: Fichero de arranque robusto para C

.section .text.init
.global _start

_start:
    # 1. Inicializar el puntero de la pila (sp) al final de la RAM (4KB).
    li sp, 4096

    # 2. Limpiar la sección .bss a ceros.
    #    El C estándar requiere que las variables estáticas/globales
    #    no inicializadas empiecen valiendo cero.
    la a0, __bss_start
    la a1, __bss_end
    bge a0, a1, bss_done_loop

bss_loop:
    sw zero, 0(a0)
    addi a0, a0, 4
    blt a0, a1, bss_loop

bss_done_loop:

    # 3. Llamar a la función main de nuestro programa en C.
    jal main

    # 4. Bucle infinito en caso de que main retorne.
loop:
    jal loop