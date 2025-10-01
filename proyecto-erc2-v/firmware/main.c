/**
 * main.c: Programa de prueba en C para el procesador erc2-v
 *
 * Funcionalidad: Escribe un contador incremental en los LEDs
 * utilizando I/O mapeado en memoria.
 */

// Definimos un puntero a la dirección de los LEDs (0x1000)
// `volatile` evita que el compilador optimice el acceso.
#define LED_REG (*(volatile unsigned int *)0x80000000)

int main() {
    unsigned int counter = 0;

    while (1) {
        // Escribimos el valor del contador en el registro de los LEDs
        LED_REG = counter;

        // Añadimos un pequeño retardo para que el parpadeo no sea demasiado
        // rápido para el ojo humano. El valor exacto de este bucle
        // dependerá de la velocidad final del procesador.
        for (volatile int i = 0; i < 50000; ++i);

        counter++;
    }

    return 0; // No se debería llegar aquí
}
