# test_leds.S
# Programa de prueba para el procesador erc2-v
#
# Funcionalidad: Incrementa un contador y lo muestra en los LEDs.
# El registro x5 (t0) se usará como contador.

.section .text
.global _start

_start:
    # Inicializamos el contador a 0.
    # addi x5, x0, 0
    # Como nuestro procesador aún no soporta ADDI con registro x0, 
    # lo hacemos en dos pasos con LUI y ADDI.
    # Esta será una buena prueba.

    lui  x5, 0x0        # x5 = 0x00000000

loop:
    # Cargamos la dirección del registro de LEDs en x6 (t1).
    # La dirección es 0x00001000 según nuestro erc2-v.v
    lui  x6, 0x1        # x6 = 0x00001000

    # Escribimos el valor del contador (x5) en la dirección de los LEDs.
    # Nuestro procesador aún no tiene instrucciones STORE, así que este paso
    # es conceptual. El hardware en erc2-v.v reacciona a la dirección y
    # al dato que se está escribiendo, pero la CPU no tiene una instrucción
    # para ello. Vamos a modificar el procesador para que soporte `sw`.
    # Por ahora, dejaremos el código que *queremos* que funcione.
    sw   x5, 0(x6)      # Store Word: *(0x00001000) = x5

    # Incrementamos el contador.
    addi x5, x5, 1      # x5 = x5 + 1

    # Saltamos de nuevo al inicio del bucle.
    jal  x0, loop       # Jump and Link a 'loop', no guardamos la dirección de retorno.

