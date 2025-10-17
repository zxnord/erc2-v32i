.section .text
.global _start

_start:
    # LED address in a1
    lui a1, 0x80000
    
    # Calculate value to write to LEDs
    li a0, 5
    addi a0, a0, 3  # a0 = 8
    
    # Write to LEDs
    sw a0, 0(a1)

    # Loop forever
loop:
    j loop
