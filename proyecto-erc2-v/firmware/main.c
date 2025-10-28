/**
 * Test de Regresión en C para el procesador ERC2-V.
 * 
 * Realiza las siguientes pruebas y muestra el resultado en los LEDs:
 * 1. Suma (10 + 15 = 25)
 * 2. Resta (25 - 5 = 20)
 * 3. Multiplicación (12 * 10 = 120)
 * 
 * Cada resultado se muestra durante unos segundos.
 */

// Puntero a los LEDs, mapeado en memoria
#define LEDS ((volatile unsigned int*)0x80000000)

// Prototipo de la función de delay
void delay(int cycles);

void main() {
    int a = 10;
    int b = 15;
    int c = 5;
    int res;

    // 1. Prueba de Suma
    res = a + b; // 10 + 15 = 25 (0x19)
    *LEDS = res;
    delay(5000000);

    // 2. Prueba de Resta
    res = res - c; // 25 - 5 = 20 (0x14)
    *LEDS = res;
    delay(5000000);

    // 3. Prueba de Multiplicación
    a = 12;
    b = 10;
    res = a * b; // 12 * 10 = 120 (0x78)
    *LEDS = res;
    delay(5000000);

    // 4. Prueba de División
    a = 120;
    b = 10;
    res = a / b; // 120 / 10 = 12 (0x0C)
    *LEDS = res;
    delay(5000000);

    // 5. Prueba de Resto
    a = 123;
    b = 10;
    res = a % b; // 123 % 10 = 3 (0x03)
    *LEDS = res;

    // Bucle infinito al final
    while(1);
}

void delay(int cycles) {
    for (int i = 0; i < cycles; i++) {
        // Esta instrucción 'nop' (no operation) se usa para que el bucle no sea optimizado
        // y eliminado por el compilador. Es una forma de 'perder tiempo'.
        asm volatile ("nop");
    }
}