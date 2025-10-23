.section .text
.global _start

_start:
    # LED address in a1
    lui a1, 0x80000
    
    # Operands
    li a0, 12
    li a2, 10
    
    # Multiplication
    mul a0, a0, a2
    
    # Write to LEDs
    sw a0, 0(a1)

    # Loop forever
loop:
    j loop
