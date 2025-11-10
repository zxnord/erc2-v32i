.section .data
leds: .word 0x80000000

.section .text
.global _start

_start:
    la t3, leds
    lw t3, 0(t3)

    li t4, 1 # Test number 1
    sw t4, 0(t3) # Display 1 on LEDs

    li t0, 0xDEADBEEF # Value to write to sstatus
    csrrw t1, sstatus, t0 # Write t0 to sstatus, read old sstatus into t1

    csrrs t2, sstatus, x0 # Read sstatus into t2

    bne t0, t2, fail # If t0 != t2, then the write/read failed

    # All tests passed
    li t4, 0x42
    sw t4, 0(t3) # Display 0x42 on LEDs
    j end_loop

fail:
    li t4, 0xEE # Error code
    sw t4, 0(t3) # Display error on LEDs

end_loop:
    j end_loop