# Test for the basic S-mode to M-mode trap mechanism

.section .text
.global _start

_start: # M-mode
    # Setup a simple trap handler
    la t0, trap_handler
    csrw mtvec, t0

    # Prepare to jump to S-mode
    la t0, s_mode_code
    csrw mepc, t0
    csrr t1, mstatus
    li t2, ~(3 << 11)
    and t1, t1, t2
    li t2, 1 << 11
    or t1, t1, t2
    csrw mstatus, t1

    # Jump to S-mode
    mret

s_mode_code: # S-mode
    # Trigger an exception with an environment call
    ecall

endless_loop:
    j endless_loop

trap_handler: # M-mode
    # Write a magic number to the LEDs to prove we reached the handler.
    # We use 0xBA as a unique 8-bit value.
    li t0, 0xBA
    li t1, 0x80000000 # Physical LED address
    sw t0, 0(t1)
trap_loop:
    j trap_loop
