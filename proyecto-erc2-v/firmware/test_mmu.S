.section .data
leds: .word 0x80000000

.section .text
.global _start

_start:
    la t3, leds
    lw t3, 0(t3)

    # Set exception handler
    la t0, exception_handler
    csrw stvec, t0

    # Enable MMU
    li t0, 0x80000000 # satp: MODE=Sv32, ASID=0, PPN=0
    csrw satp, t0
    nop

    csrr t1, satp
    srli t1, t1, 24 # Shift to get the upper 8 bits
    sw t1, 0(t3) # Display satp[31:24] on LEDs

loop:
    j loop

exception_handler:
    li t4, 0xDD
    sw t4, 0(t3)
    j exception_handler
