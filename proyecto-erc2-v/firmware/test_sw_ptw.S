# Test for the Software-based Page Table Walker with LED trace debugging

.macro WRITE_LED value
    li t6, 0x80000000 # Use t6 for address, as it's saved/restored
    li t5, \value     # Use t5 for value
    sw t5, 0(t6)
.endm

.section .text
.global _start

_start: # Executing in M-mode
    WRITE_LED 0x01

    # --- 1. Setup M-mode trap handler (our Software PTW) ---
    la t0, trap_handler
    csrw mtvec, t0

    # --- 2. Setup a stack pointer (sp) for the handler to use ---
    li sp, 0x4000

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
    WRITE_LED 0x02

    # --- 5. Point SATP to the root page table (L1_TABLE) ---
    li t0, (8 << 22) | 1
    csrw satp, t0
    
    # --- 6. Flush TLB and enable MMU ---
    sfence.vma

    # --- 7. Test the mapping ---
    # The next instruction fetch will cause a TLB miss.
    li t3, 0x10000000   # Virtual address for LEDs
    li t4, 0xAB         # Value to write
    sw t4, 0(t3)        # This should succeed after the PTW runs.

s_mode_loop:
    j s_mode_loop

# ===========================================================
# --- M-mode Trap Handler / Software Page Table Walker ---
# ===========================================================
trap_handler:
    WRITE_LED 0x10

    # Save context
    addi sp, sp, -28
    sw t0, 0(sp)
    sw t1, 4(sp)
    sw t2, 8(sp)
    sw t3, 12(sp)
    sw t4, 16(sp)
    sw t5, 20(sp)
    sw t6, 24(sp)

    # Check cause - we only handle page faults
    csrr t0, mcause
    li t1, 12 # Instruction page fault
    li t2, 13 # Load page fault
    li t3, 15 # Store page fault
    beq t0, t1, page_fault_handler
    beq t0, t2, page_fault_handler
    beq t0, t3, page_fault_handler
    j fault_error # If not a page fault, error out

page_fault_handler:
    WRITE_LED 0x20
    # t0 already has mcause
    # Get faulting address
    csrr t1, mtval # t1 = va

    # Get page table base from satp
    csrr t2, satp
    li t3, 0x3FFFFF # Mask for PPN
    and t2, t2, t3
    slli t2, t2, 12 # t2 = L1 table base physical address

    # --- L1 Walk ---
    srli t3, t1, 22 # t3 = vpn1
    slli t3, t3, 2 # vpn1 * 4
    add t2, t2, t3 # t2 = L1 PTE address
    lw t3, 0(t2) # t3 = L1 PTE
    andi t4, t3, 1 # V bit
    beq t4, x0, fault_error # PTE not valid
    andi t4, t3, 0xE # R,W,X bits
    bne t4, x0, fault_error # L1 PTE is a leaf, should not be
    WRITE_LED 0x30

    # --- L0 Walk ---
    srli t2, t3, 10
    slli t2, t2, 12 # t2 = L0 table base physical address
    srli t3, t1, 12
    li t4, 0x3FF # Mask for VPN0
    and t3, t3, t4 # t3 = vpn0
    slli t3, t3, 2 # vpn0 * 4
    add t2, t2, t3 # t2 = L0 PTE address
    lw t3, 0(t2) # t3 = L0 PTE
    andi t4, t3, 1 # V bit
    beq t4, x0, fault_error # PTE not valid
    andi t4, t3, 0xE # R,W,X bits
    beq t4, x0, fault_error # L0 PTE is not a leaf
    WRITE_LED 0x40

    # --- Install TLB Entry ---
    srli t4, t1, 12 # t4 = VPN
    li t5, 1
    li t6, 12 # Instruction page fault cause
    bne t0, t6, use_index_1
    j write_tlb
use_index_1:
    li t5, 5
write_tlb:
    WRITE_LED 0x50
    csrw 0x7C0, t4
    csrw 0x7C1, t3
    csrw 0x7C2, t5
    csrw 0x7C2, x0

    # --- Return from exception ---
    j restore_context

fault_error:
    WRITE_LED 0xEE
fault_loop:
    j fault_loop

restore_context:
    WRITE_LED 0x60
    lw t0, 0(sp)
    lw t1, 4(sp)
    lw t2, 8(sp)
    lw t3, 12(sp)
    lw t4, 16(sp)
    lw t5, 20(sp)
    lw t6, 24(sp)
    addi sp, sp, 28
    mret

# ===========================================================
# --- Data Section: Page Tables ---
.section .data
.align 12
L1_TABLE: # At physical address 0x1000 (PPN=1)
    .word (2 << 10) | 1
    .zero 252
    .word (3 << 10) | 1
    .zero 3836

.align 12
L0_TABLE_CODE: # At physical address 0x2000 (PPN=2)
    .word (0 << 10) | 0b11001011
    .zero 4092

.align 12
L0_TABLE_DATA: # At physical address 0x3000 (PPN=3)
    .word (0x80000 << 10) | 0b11001111
    .zero 4092