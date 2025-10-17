.section .text
.global _start

_start:
    # Initialize counter in register a0
    li  a0, 0
    
    # LED address in a1
    lui a1, 0x80000
    
loop:
    # Write counter to LEDs
    sw  a0, 0(a1)
    
    # Simple delay loop
    li a3, 50
outer_loop:
    li  a2, 100000
delay_loop:
    addi a2, a2, -1
    bnez a2, delay_loop
    addi a3, a3, -1
    bnez a3, outer_loop
    
    # Increment counter
    addi a0, a0, 1
    
    # Loop forever
    j loop
