.section .text
.global _start

_start:
    # LED address in a1
    lui a1, 0x80000
    
    # Value to check
    li a0, 1
    
    # Branch if not zero
    bnez a0, turn_on_led
    
    # Loop forever if zero
loop1:
    j loop1

turn_on_led:
    # Value to write to LEDs
    li a0, 1
    
    # Write to LEDs
    sw a0, 0(a1)

    # Loop forever
loop2:
    j loop2
