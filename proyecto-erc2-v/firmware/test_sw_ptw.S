# Test for the Hardware Page Table Walker

.macro WRITE_LED value
    li t6, 0x80000000 # Physical address for LEDs
    li t5, \value
    sw t5, 0(t6)
.endm

.section .text
.global _start

_start:
    # Announce we are starting
    WRITE_LED 0x01

    # --- 1. Point SATP to the root page table (L1_TABLE) ---
    # SATP format for Sv32: MODE (1 bit) | ASID (9 bits) | PPN (22 bits)
    # MODE = 1 (Sv32)
    # ASID = 0
    # PPN = 1 (Physical Page Number of L1_TABLE is 0x1000 >> 12 = 1)
    # So, satp = (1 << 31) | 1 = 0x80000001
    li t0, 0x80000001
    WRITE_LED 0x03
    csrw satp, t0
    # WRITE_LED 0x04 # This is unsafe after MMU is enabled
    
    # --- 2. Flush TLB to activate MMU ---
    sfence.vma
    # WRITE_LED 0x05 # This is unsafe after MMU is enabled

    # --- 3. Test the mapping ---
    # The following store should trigger a TLB miss, which the hardware PTW will handle.
    # The CPU will stall, the PTW will walk the tables, install the TLB entry,
    # and then the CPU will resume and this store will succeed.
    # The page tables map virtual 0x10000000 to physical 0x80000000 (LEDs).
    
    li t3, 0x10000000   # Virtual address for LEDs
    li t4, 0xAB         # Value to write
    sw t4, 0(t3)        # This should write 0xAB to the LEDs after the PTW runs.

endless_loop:
    j endless_loop

# ===========================================================
# --- Page Table Sections ---
# The linker script 'linker_sw_ptw.ld' will place these
# sections at the correct physical addresses.
# ===========================================================

.section .l1_page_table,"a",@progbits
.align 12
L1_TABLE: # At physical address 0x1000 (PPN=1)
    # Entry for 0x00000... (code section). Points to L0_TABLE_CODE (PPN=2).
    .word (2 << 10) | 1
    .zero 4 * 63 # some padding
    # Entry for 0x10000... (our test VA)
    # VPN1 is 0x40 (64)
    # PTE should point to L0_TABLE_DATA (PPN=3) and be valid, not a leaf.
    # PTE = (PPN << 10) | V-bit = (3 << 10) | 1
    .word (3 << 10) | 1
    .zero 4 * (1024 - 65)

.section .l0_code_page_table,"a",@progbits
.align 12
L0_TABLE_CODE: # At physical address 0x2000 (PPN=2)
    # Entry for 0x00000000 (code)
    # VPN0 is 0
    # PTE should point to physical 0x0 and be a valid, R/W/X leaf.
    # PPN = 0
    # Flags = V=1, R=1, W=1, X=1 -> 0b1111
    # PTE = (PPN << 10) | Flags = (0 << 10) | 15
    .word (0 << 10) | 15
    .zero 4 * 1023

.section .l0_data_page_table,"a",@progbits
.align 12
L0_TABLE_DATA: # At physical address 0x3000 (PPN=3)
    # Entry for 0x10000000
    # VPN0 is 0
    # PTE should point to physical 0x80000000 (LEDs) and be a valid, R/W/X leaf.
    # PPN = 0x80000
    # Flags = V=1, R=1, W=1, X=1, U=1 (User-accessible) -> 0b1111
    # PTE = (PPN << 10) | Flags = (0x80000 << 10) | 15
    .word (0x80000 << 10) | 15
    .zero 4 * 1023