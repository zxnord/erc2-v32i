void main() {
    volatile int *led_ptr = (int *)0x80000000;
    int a = 0x12345678;
    int b = 0x87654321;
    int result;

    while (1) {
        // ADD
        result = a + b;
        *led_ptr = result >> 24;
        for (volatile int i = 0; i < 1000000; i++);

        // SUB
        result = a - b;
        *led_ptr = result >> 24;
        for (volatile int i = 0; i < 1000000; i++);

        // AND
        result = a & b;
        *led_ptr = result >> 24;
        for (volatile int i = 0; i < 1000000; i++);

        // OR
        result = a | b;
        *led_ptr = result >> 24;
        for (volatile int i = 0; i < 1000000; i++);

        // XOR
        result = a ^ b;
        *led_ptr = result >> 24;
        for (volatile int i = 0; i < 1000000; i++);

        // SLL
        result = a << 3;
        *led_ptr = result >> 24;
        for (volatile int i = 0; i < 1000000; i++);

        // SRL
        result = (int)((unsigned int)a >> 3);
        *led_ptr = result >> 24;
        for (volatile int i = 0; i < 1000000; i++);

        // SRA
        __asm__("srai %0, %1, 3" : "=r"(result) : "r"(a));
        *led_ptr = result >> 24;
        for (volatile int i = 0; i < 1000000; i++);

        // SLT
        result = (a < b) ? 1 : 0;
        *led_ptr = result;
        for (volatile int i = 0; i < 1000000; i++);

        // SLTU
        result = ((unsigned int)a < (unsigned int)b) ? 1 : 0;
        *led_ptr = result;
        for (volatile int i = 0; i < 1000000; i++);
    }
}