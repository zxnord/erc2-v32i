# Test for MMU Permission Faults

.section .text
.global _start

_start: # Executing in M-mode
    # --- 1. Setup M-mode trap handler ---
    la t0, m_trap_handler
    csrw mtvec, t0

    # --- 2. Prepare for transition to S-mode ---
    la t0, s_mode_code
    csrw mepc, t0
    csrr t1, mstatus
    li t2, ~(3 << 11)
    and t1, t1, t2
    li t2, 1 << 11
    or t1, t1, t2
    csrw mstatus, t1

    # --- 3. Transition to S-mode ---
    mret

# ===========================================================
s_mode_code: # Now executing in S-mode
    # --- 4. Point SATP to the root page table (L1_TABLE) ---
    # PPN of L1_TABLE (phys addr 0x1000) is 1. MODE is Sv32 (8).
    li t0, (8 << 22) | 1
    csrw satp, t0
    
    # --- 5. Flush TLB and enable MMU ---
    sfence.vma

    # --- 6. Test: Attempt to write to a Read-Only page ---
    # This should cause a Store Page Fault (cause 15 / 0x0F).
    # The trap handler will write this cause to the LEDs.
    li t3, 0x10000000   # Virtual address of our Read-Only page
    li t4, 0xDEADBEEF   # Value to write
    sw t4, 0(t3)        # This instruction should fault

    # --- If we get here, the test FAILED! ---
    # Write a failure code to the LEDs.
    li t3, 0x10001000   # Use the writable LED virtual address
    li t4, 0xBAD0BAD0
    sw t4, 0(t3)

s_mode_loop:
    j s_mode_loop

# ===========================================================
m_trap_handler:
    # If we get here, an exception occurred.
    # For this test, we EXPECT a store page fault (15).
    # Write the cause code to the LEDs to confirm.
    csrr t5, mcause
    li t6, 0x80000000 # Physical address of LEDs
    sw t5, 0(t6)
m_trap_loop:
    j m_trap_loop

# ===========================================================
# --- Data Section: Page Tables ---
.section .data
.align 12
L1_TABLE: # At physical address 0x1000
    # Entry 0: for VA 0x0... -> points to L0_TABLE_CODE (at 0x2000, PPN=2)
    .word (2 << 10) | 1 # PPN=2, V=1
    .zero 252 # Entries 1-63 are 0
    # Entry 64 (for VA 0x1000...): -> points to L0_TABLE_DATA (at 0x3000, PPN=3)
    .word (3 << 10) | 1 # PPN=3, V=1
    .zero 3836 # Fill the rest of the 4KB page

.align 12
L0_TABLE_CODE: # At physical address 0x2000
    # Entry 0: for VA 0x0... -> maps to PA 0x0... (PPN=0)
    # Perms: Valid, Read, Execute, Accessed, Dirty
    .word (0 << 10) | 0b11001011 # PPN=0, V,R,X,A,D
    .zero 4092

.align 12
L0_TABLE_DATA: # At physical address 0x3000
    # Entry 0: for VA 0x10000000 -> maps to PA 0x20000 (PPN=0x20)
    # Perms: Valid, Read, Accessed, Dirty (NO WRITE)
    .word (0x20 << 10) | 0b11001001 # PPN=0x20, V,R,A,D (W=0)
    # Entry 1: for VA 0x10001000 -> maps to LEDs at PA 0x80000000 (PPN=0x80000)
    # Perms: Valid, Read, Write, Accessed, Dirty
    .word (0x80000 << 10) | 0b11001111 # PPN=0x80000, V,R,W,A,D
    .zero 4088
