# Test for the Hardware Page Table Walker (PTW)

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
    # The PPN is the physical address >> 12.
    # L1_TABLE is at physical address 0x1000, so PPN is 1.
    li t0, 1
    # Set MODE to Sv32 (8)
    li t1, 8
    slli t1, t1, 22
    or t0, t0, t1
    csrw satp, t0
    
    # --- 5. Flush TLB and enable MMU ---
    sfence.vma

    # --- 6. Test the mapping ---
    # The next instruction fetch will cause a TLB miss.
    # The hardware PTW should walk the tables and find the identity map for code.
    # Then, the 'sw' will cause another TLB miss for the data address.
    # The PTW should walk the tables again and find the map for the LEDs.
    li t3, 0x10000000   # Virtual address for LEDs
    li t4, 0xAB         # Value to write
    sw t4, 0(t3)        # Write via translated address

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

# ===========================================================
# --- Data Section: Page Tables ---
# Must be page-aligned. We use .align 12 (4096 bytes).

.section .data
.align 12
L1_TABLE:
    # We need two entries in the L1 table.
    # Entry 0: for VA 0x0... -> points to L0_TABLE_CODE (at 0x2000, PPN=2)
    # Entry 64 (for VA 0x1000...): -> points to L0_TABLE_DATA (at 0x3000, PPN=3)
    # All other 1022 entries are 0 (invalid).
    .word (2 << 10) | 1 # PPN=2, V=1
    .zero 252 # Entries 1-63 are 0
    .word (3 << 10) | 1 # PPN=3, V=1
    .zero 3836 # Fill the rest of the 4KB page

.align 12
L0_TABLE_CODE:
    # Entry 0: for VA 0x00000... -> maps to PA 0x0... (PPN=0)
    # Perms: Valid, Read, Execute, Accessed, Dirty
    .word (0 << 10) | 0b11001011 # PPN=0, V,R,X,A,D
    .zero 4092 # Fill the rest of the 4KB page

.align 12
L0_TABLE_DATA:
    # Entry 0: for VA 0x10000... -> maps to PA 0x80000000 (PPN=0x80000)
    # Perms: Valid, Read, Write, Accessed, Dirty
    .word (0x80000 << 10) | 0b11001111 # PPN=0x80000, V,R,W,A,D
    .zero 4092 # Fill the rest of the 4KB page
