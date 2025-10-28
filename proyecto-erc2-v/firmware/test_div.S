.section .text
.global _start

_start:
    # LED address in a1
    lui a1, 0x80000
    
    # --- Division (123 / 10 = 12) ---
    li a2, 123
    li a3, 10
    divu a4, a2, a3
    
    # Write quotient to LEDs
    sw a4, 0(a1)
    
    # --- Delay ---
    li a5, 100 // Un poco más largo
outer_loop_1:
    li  a6, 200000
delay_loop_1:
    addi a6, a6, -1
    bnez a6, delay_loop_1
    addi a5, a5, -1
    bnez a5, outer_loop_1

    # --- Remainder (123 % 10 = 3) ---
    remu a4, a2, a3

    # Write remainder to LEDs
    sw a4, 0(a1)

    # Loop forever
loop:
    j loop