.section .text
.global _start

_start:
    # LED address in a1
    lui a1, 0x80000
    
    # Value to write to LEDs
    li a0, 1
    
    # Write to LEDs
    sw a0, 0(a1)

    # Initialize counter in register a0
    li  a0, 0
    
loop:
    # Write counter to LEDs
    sw  a0, 0(a1)
    
    # Simple delay loop (count to 100000)
    li  a2, 100000
delay_loop:
    addi a2, a2, -1
    bnez a2, delay_loop
    
    # Increment counter
    addi a0, a0, 1
    
    # Loop forever
    j loop
