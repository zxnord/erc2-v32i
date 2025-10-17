.section .text
.global _start

_start:
    # LED address in a1
    lui a1, 0x80000
    
    # Initialize counter in register a0
    li  a0, 0
    
loop:
    # Increment counter
    addi a0, a0, 1
    nop
    
    # Write counter to LEDs
    sw  a0, 0(a1)
    
    # Loop forever
    j loop
