.data

.text
.globl main

main:
# ######################################
# ### Load address of _answer to s0 
# ######################################
    mv  x28, s0
    mv  x29, ra

# ######################################
# ### sraiw test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

sraiw_2:
    li  gp, 2
    li  x13, 0xffffffff80000000
    sraiw x14, x13, 0
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

sraiw_3:
    li  gp, 3
    li  x13, 0xffffffff80000000
    sraiw x14, x13, 1
    li  x7, 0xffffffffc0000000
    bne x14, x7, failed

sraiw_4:
    li  gp, 4
    li  x13, 0xffffffff80000000
    sraiw x14, x13, 7
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

sraiw_5:
    li  gp, 5
    li  x13, 0xffffffff80000000
    sraiw x14, x13, 14
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

sraiw_6:
    li  gp, 6
    li  x13, 0xffffffff80000001
    sraiw x14, x13, 31
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sraiw_7:
    li  gp, 7
    li  x13, 2147483647
    sraiw x14, x13, 0
    li  x7, 2147483647
    bne x14, x7, failed

sraiw_8:
    li  gp, 8
    li  x13, 2147483647
    sraiw x14, x13, 1
    li  x7, 1073741823
    bne x14, x7, failed

sraiw_9:
    li  gp, 9
    li  x13, 2147483647
    sraiw x14, x13, 7
    li  x7, 16777215
    bne x14, x7, failed

sraiw_10:
    li  gp, 10
    li  x13, 2147483647
    sraiw x14, x13, 14
    li  x7, 131071
    bne x14, x7, failed

sraiw_11:
    li  gp, 11
    li  x13, 2147483647
    sraiw x14, x13, 31
    li  x7, 0
    bne x14, x7, failed

sraiw_12:
    li  gp, 12
    li  x13, 0xffffffff81818181
    sraiw x14, x13, 0
    li  x7, 0xffffffff81818181
    bne x14, x7, failed

sraiw_13:
    li  gp, 13
    li  x13, 0xffffffff81818181
    sraiw x14, x13, 1
    li  x7, 0xffffffffc0c0c0c0
    bne x14, x7, failed

sraiw_14:
    li  gp, 14
    li  x13, 0xffffffff81818181
    sraiw x14, x13, 7
    li  x7, 0xffffffffff030303
    bne x14, x7, failed

sraiw_15:
    li  gp, 15
    li  x13, 0xffffffff81818181
    sraiw x14, x13, 14
    li  x7, 0xfffffffffffe0606
    bne x14, x7, failed

sraiw_16:
    li  gp, 16
    li  x13, 0xffffffff81818181
    sraiw x14, x13, 31
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

# Verify that shifts ignore top 32 (using true 64-bit values)

sraiw_44:
    li  gp, 44
    li  x13, 0xffffffff12345678
    sraiw x14, x13, 0
    li  x7, 305419896
    bne x14, x7, failed

sraiw_45:
    li  gp, 45
    li  x13, 0xffffffff12345678
    sraiw x14, x13, 4
    li  x7, 19088743
    bne x14, x7, failed

sraiw_46:
    li  gp, 46
    li  x13, 2452903544
    sraiw x14, x13, 0
    li  x7, 0xffffffff92345678
    bne x14, x7, failed

sraiw_47:
    li  gp, 47
    li  x13, 2452903544
    sraiw x14, x13, 4
    li  x7, 0xfffffffff9234567
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

sraiw_17:
    li  gp, 17
    li  x11, 0xffffffff80000000
    sraiw x11, x11, 7
    li  x7, 0xffffffffff000000
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

sraiw_18:
    li  gp, 18
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    sraiw x14, x1, 7
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x6, x7, failed

sraiw_19:
    li  gp, 19
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    sraiw x14, x1, 14
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x6, x7, failed

sraiw_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 0xffffffff80000001
    sraiw x14, x1, 31
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x6, x7, failed

sraiw_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    sraiw x14, x1, 7
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

sraiw_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    nop
    sraiw x14, x1, 14
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

sraiw_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 0xffffffff80000001
    nop    
    nop
    sraiw x14, x1, 31
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sraiw_24:
    li  gp, 24
    sraiw x1, x0, 31
    li  x7, 0
    bne x1, x7, failed

sraiw_25:
    li  gp, 25
    li  x1, 31
    sraiw x0, x1, 28
    li  x7, 0
    bne x0, x7, failed

sraiw_26:
    li  gp, 26
    li  x13, 63050394783186944
    sraiw x14, x13, 28
    li  x7, 0
    bne x14, x7, failed

sraiw_27:
    li  gp, 27
    li  x13, 4026531840
    sraiw x14, x13, 4
    li  x7, 0xffffffffff000000
    bne x14, x7, failed


pass:
    li  gp, 0         # 0 means all tests passed

failed:
    sw  gp, 0(s0)     # Record the wrong test number

main_exit:
# ######################################
# ### Return to end the simulation
# ######################################
    mv  ra, x29
    mv  s0, x28
    ret
