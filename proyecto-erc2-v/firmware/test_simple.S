.section .text
.global _start

_start:
    # Simple test: just write 0x55 to LEDs and halt
    lui a0, 0x80000      # LED address = 0x80000000
    li  a1, 0x55         # Pattern 0x55 = 01010101
    sw  a1, 0(a0)        # Write to LEDs
    
halt:
    j halt               # Infinite loop

