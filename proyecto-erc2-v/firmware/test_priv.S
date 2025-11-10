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

    # Attempt to execute an illegal instruction
    .word 0x00000000 # Illegal instruction

    # If we get here, it means the illegal instruction was not triggered
    li t4, 0xEE # Error code
    sw t4, 0(t3) # Display error on LEDs
    j _start

exception_handler:
    # Check cause
    csrr t0, scause
    li t1, 2 # Illegal instruction
    bne t0, t1, fail

    # All tests passed
    li t4, 0x42
    sw t4, 0(t3) # Display 0x42 on LEDs
    j end_loop

fail:
    li t4, 0xEE # Error code
    sw t4, 0(t3) # Display error on LEDs

end_loop:
    j end_loop
