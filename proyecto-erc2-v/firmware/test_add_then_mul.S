.section .text
.global _start

_start:
    # LED address in a1
    lui a1, 0x80000
    
    # --- Addition ---
    li a0, 5
    li a2, 3
    add a0, a0, a2  # a0 = 8
    
    # Write addition result to LEDs
    sw a0, 0(a1)
    
    # --- Delay ---
    li a3, 50
outer_loop_1:
    li  a2, 100000
delay_loop_1:
    addi a2, a2, -1
    bnez a2, delay_loop_1
    addi a3, a3, -1
    bnez a3, outer_loop_1

    # --- Multiplication ---
    li a0, 12
    li a2, 10
    mul a0, a0, a2 # a0 = 120 = 0x78

    # Write multiplication result to LEDs
    sw a0, 0(a1)

    # Loop forever
loop:
    j loop
