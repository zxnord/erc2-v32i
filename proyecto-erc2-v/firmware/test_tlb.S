.section .text
.global _start

_start: # Executing in M-mode
    # --- 1. Setup M-mode trap handler ---
    la t0, m_trap_handler
    csrw mtvec, t0

    # --- 2. Program TLB from M-mode BEFORE switching ---
    # Entry 0: Identity map for code (VA 0x0 -> PA 0x0)
    li t0, 0
    li t1, ((0x0 << 10) | (0b101 << 1) | 1)
    li t2, 1 # Index 0 trigger
    csrw 0x7C0, t0
    csrw 0x7C1, t1
    csrw 0x7C2, t2
    csrw 0x7C2, x0  # IMPORTANT: Clear trigger

    # Entry 1: Map for LEDs (VA 0x10000000 -> PA 0x80000000)
    li t0, 0x10000
    li t1, ((0x80000 << 10) | (0b110 << 1) | 1)
    li t2, 5 # Index 1 trigger
    csrw 0x7C0, t0
    csrw 0x7C1, t1
    csrw 0x7C2, t2
    csrw 0x7C2, x0 # IMPORTANT: Clear trigger

    # --- 3. Prepare for transition to S-mode ---
    la t0, s_mode_code
    csrw mepc, t0
    csrr t1, mstatus
    li t2, ~(3 << 11)
    and t1, t1, t2
    li t2, 1 << 11
    or t1, t1, t2
    csrw mstatus, t1

    # --- 4. Transition to S-mode ---
    mret

# ===========================================================
s_mode_code: # Now executing in S-mode
    # The mapping is already set up in the TLB from M-mode.
    # Now, just enable the MMU.
    li t0, 0x80000000
    csrw satp, t0
    sfence.vma

    # Test the D-MMU mapping
    li t3, 0x10000000
    li t4, 0xAB
    sw t4, 0(t3)

s_mode_loop:
    j s_mode_loop

# ===========================================================
m_trap_handler:
    # If we end up here, something went wrong.
    # Write the cause code to the LEDs to debug.
    csrr t5, mcause
    li t6, 0x80000000 # Physical address of LEDs
    sw t5, 0(t6)
m_trap_loop:
    j m_trap_loop
