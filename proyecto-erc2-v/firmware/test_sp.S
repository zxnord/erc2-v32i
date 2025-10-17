.section .text
.global _start

_start:
    # Initialize sp
    li sp, 0x1000

    # LED address in a1
    lui a1, 0x80000
    
    # Value to write to LEDs
    li a0, 1
    
    # Write to LEDs
    sw a0, 0(a1)

    # Loop forever
loop:
    j loop
