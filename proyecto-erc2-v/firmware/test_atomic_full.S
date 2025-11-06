.section .data
test_val: .word 0
leds: .word 0x80000000

.section .text

.global _start



.macro delay_loop

    li t6, 3000000 # Adjust this value for desired delay

1:  addi t6, t6, -1

    bnez t6, 1b

.endm



_start:

    la t3, leds

    lw t3, 0(t3)



    # Test 1: amoadd.w

    li t4, 1

    sw t4, 0(t3) # Display 1 on LEDs

    delay_loop

    li t0, 5

    la t1, test_val

    sw zero, 0(t1) # test_val = 0

    amoadd.w zero, t0, (t1) # test_val should become 5

    lw t2, (t1)

    bne t2, t0, end_loop # if test_val != 5, fail



    # Test 2: amoswap.w

    li t4, 2

    sw t4, 0(t3) # Display 2 on LEDs

    delay_loop

    li t0, 10

    amoswap.w zero, t0, (t1) # test_val should become 10

    lw t2, (t1)

    bne t2, t0, end_loop # if test_val != 10, fail



    # Test 3: amoor.w

    li t4, 3

    sw t4, 0(t3) # Display 3 on LEDs

    delay_loop

    li t0, 3

    amoor.w zero, t0, (t1) # test_val (10) | 3 = 11

    lw t2, (t1)

    li t5, 11

    bne t2, t5, end_loop # if test_val != 11, fail



    # Test 4: amoxor.w

    li t4, 4

    sw t4, 0(t3) # Display 4 on LEDs

    delay_loop

    li t0, 5

    amoxor.w zero, t0, (t1) # test_val (11) ^ 5 = 14

    lw t2, (t1)

    li t5, 14

    bne t2, t5, end_loop # if test_val != 14, fail



    # Test 5: amoand.w

    li t4, 5

    sw t4, 0(t3) # Display 5 on LEDs

    delay_loop

    li t0, 12

    amoand.w zero, t0, (t1) # test_val (14) & 12 = 12

    lw t2, (t1)

    li t5, 12

    bne t2, t5, end_loop # if test_val != 12, fail



    # Test 6: amominu.w

    li t4, 6

    sw t4, 0(t3) # Display 6 on LEDs

    delay_loop

    li t0, 10

    amominu.w zero, t0, (t1) # min(12, 10) = 10

    lw t2, (t1)

    li t5, 10

    bne t2, t5, end_loop # if test_val != 10, fail



    # Test 7: amomaxu.w

    li t4, 7

    sw t4, 0(t3) # Display 7 on LEDs

    delay_loop

    li t0, 15

    amomaxu.w zero, t0, (t1) # max(10, 15) = 15

    lw t2, (t1)

    li t5, 15

    bne t2, t5, end_loop # if test_val != 15, fail



    # Test 8: amomin.w (signed)

    li t4, 8

    sw t4, 0(t3) # Display 8 on LEDs

    delay_loop

    li t0, -5

    amomin.w zero, t0, (t1) # min(15, -5) = -5

    lw t2, (t1)

    li t5, -5

    bne t2, t5, end_loop # if test_val != -5, fail



    # Test 9: amomax.w (signed)

    li t4, 9

    sw t4, 0(t3) # Display 9 on LEDs

    delay_loop

    li t0, -10

    amomax.w zero, t0, (t1) # max(-5, -10) = -5

    lw t2, (t1)

    li t5, -5

    bne t2, t5, end_loop # if test_val != -5, fail



    # Test 10: lr.w and sc.w (success)

    li t4, 10

    sw t4, 0(t3) # Display 10 on LEDs

    delay_loop

    li t0, 20

    lr.w t5, (t1)

    sc.w t6, t0, (t1)

    bnez t6, end_loop # if sc fails, fail

    lw t2, (t1)

    li t5, 20

    bne t2, t5, end_loop # if test_val != 20, fail


    # Test 11: lr.w and sc.w (failure)

    li t4, 11

    sw t4, 0(t3) # Display 11 on LEDs

    delay_loop

    li t0, 0 # Initialize test_val to 0

    sw t0, 0(t1)

    li t0, 30 # Value to attempt to store

    lr.w t5, (t1) # Load reserved, t5 gets 0

    li t2, 1 # Invalidate reservation by writing to the same address

    sw t2, 0(t1)

    sc.w t6, t0, (t1) # Should fail, t6 should be non-zero

    beqz t6, end_loop # If sc succeeds, it's a failure, so go to end_loop

    # If sc fails, it's a success, continue to next test



    # All tests passed

    li t4, 0x42

    sw t4, 0(t3) # Display 0x42 on LEDs

    delay_loop



    



    end_loop:



        j end_loop

    