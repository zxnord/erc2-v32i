#define LEDS ((volatile unsigned int*)0x80000000)

void delay(int cycles);

void main() {
    int test_no = 0;
    int res;
    int a, b;

    // Test 1: add
    test_no = 1;
    *LEDS = test_no;
    delay(1000000);
    a = 10;
    b = 15;
    res = a + b;
    if (res != 25) while(1);

    // Test 2: sub
    test_no = 2;
    *LEDS = test_no;
    delay(1000000);
    a = 25;
    b = 5;
    res = a - b;
    if (res != 20) while(1);

    // Test 3: mul
    test_no = 3;
    *LEDS = test_no;
    delay(1000000);
    a = 12;
    b = 10;
    res = a * b;
    if (res != 120) while(1);

    // Test 4: div
    test_no = 4;
    *LEDS = test_no;
    delay(1000000);
    a = 120;
    b = 10;
    res = a / b;
    if (res != 12) while(1);

    // Test 5: rem
    test_no = 5;
    *LEDS = test_no;
    delay(1000000);
    a = 123;
    b = 10;
    res = a % b;
    if (res != 3) while(1);

    // Test 6: amoswap.w
    test_no = 6;
    *LEDS = test_no;
    delay(1000000);
    int mem_var = 50;
    int new_val = 100;
    int old_val;
    asm volatile (
        "amoswap.w %0, %2, (%1)"
        : "=r"(old_val)
        : "r"(&mem_var), "r"(new_val)
        : "memory"
    );
    if (old_val != 50 || mem_var != 100) while(1);

    // Test 7: amoadd.w
    test_no = 7;
    *LEDS = test_no;
    delay(1000000);
    mem_var = 10;
    new_val = 5;
    asm volatile (
        "amoadd.w %0, %2, (%1)"
        : "=r"(old_val)
        : "r"(&mem_var), "r"(new_val)
        : "memory"
    );
    if (old_val != 10 || mem_var != 15) while(1);

    // Test 8: amomin.w (signed)
    test_no = 8;
    *LEDS = test_no;
    delay(1000000);
    mem_var = 10;
    new_val = -5;
    asm volatile (
        "amomin.w %0, %2, (%1)"
        : "=r"(old_val)
        : "r"(&mem_var), "r"(new_val)
        : "memory"
    );
    if (old_val != 10 || mem_var != -5) while(1);

    // Test 9: amomax.w (signed)
    test_no = 9;
    *LEDS = test_no;
    delay(1000000);
    mem_var = -5;
    new_val = -10;
    asm volatile (
        "amomax.w %0, %2, (%1)"
        : "=r"(old_val)
        : "r"(&mem_var), "r"(new_val)
        : "memory"
    );
    if (old_val != -5 || mem_var != -5) while(1);

    // All tests passed
    *LEDS = 0x42;
    delay(1000000);
    while(1);
}

void delay(int cycles) {
    for (int i = 0; i < cycles; i++) {
        asm volatile ("nop");
    }
}
