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
# ### srliw test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

srliw_2:
    li  gp, 2
    li  x13, 0xffffffff80000000
    srliw x14, x13, 0
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

srliw_3:
    li  gp, 3
    li  x13, 0xffffffff80000000
    srliw x14, x13, 1
    li  x7, 1073741824
    bne x14, x7, failed

srliw_4:
    li  gp, 4
    li  x13, 0xffffffff80000000
    srliw x14, x13, 7
    li  x7, 16777216
    bne x14, x7, failed

srliw_5:
    li  gp, 5
    li  x13, 0xffffffff80000000
    srliw x14, x13, 14
    li  x7, 131072
    bne x14, x7, failed

srliw_6:
    li  gp, 6
    li  x13, 0xffffffff80000001
    srliw x14, x13, 31
    li  x7, 1
    bne x14, x7, failed

srliw_7:
    li  gp, 7
    li  x13, 0xffffffffffffffff
    srliw x14, x13, 0
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

srliw_8:
    li  gp, 8
    li  x13, 0xffffffffffffffff
    srliw x14, x13, 1
    li  x7, 2147483647
    bne x14, x7, failed

srliw_9:
    li  gp, 9
    li  x13, 0xffffffffffffffff
    srliw x14, x13, 7
    li  x7, 33554431
    bne x14, x7, failed

srliw_10:
    li  gp, 10
    li  x13, 0xffffffffffffffff
    srliw x14, x13, 14
    li  x7, 262143
    bne x14, x7, failed

srliw_11:
    li  gp, 11
    li  x13, 0xffffffffffffffff
    srliw x14, x13, 31
    li  x7, 1
    bne x14, x7, failed

srliw_12:
    li  gp, 12
    li  x13, 555819297
    srliw x14, x13, 0
    li  x7, 555819297
    bne x14, x7, failed

srliw_13:
    li  gp, 13
    li  x13, 555819297
    srliw x14, x13, 1
    li  x7, 277909648
    bne x14, x7, failed

srliw_14:
    li  gp, 14
    li  x13, 555819297
    srliw x14, x13, 7
    li  x7, 4342338
    bne x14, x7, failed

srliw_15:
    li  gp, 15
    li  x13, 555819297
    srliw x14, x13, 14
    li  x7, 33924
    bne x14, x7, failed

srliw_16:
    li  gp, 16
    li  x13, 555819297
    srliw x14, x13, 31
    li  x7, 0
    bne x14, x7, failed

# Verify that shifts ignore top 32 (using true 64-bit values)

srliw_44:
    li  gp, 44
    li  x13, 0xffffffff12345678
    srliw x14, x13, 0
    li  x7, 305419896
    bne x14, x7, failed

srliw_45:
    li  gp, 45
    li  x13, 0xffffffff12345678
    srliw x14, x13, 4
    li  x7, 19088743
    bne x14, x7, failed

srliw_46:
    li  gp, 46
    li  x13, 2452903544
    srliw x14, x13, 0
    li  x7, 0xffffffff92345678
    bne x14, x7, failed

srliw_47:
    li  gp, 47
    li  x13, 2452903544
    srliw x14, x13, 4
    li  x7, 153306471
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

srliw_17:
    li  gp, 17
    li  x11, 0xffffffff80000000
    srliw x11, x11, 7
    li  x7, 16777216
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

srliw_18:
    li  gp, 18
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    srliw x14, x1, 7
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16777216
    bne x6, x7, failed

srliw_19:
    li  gp, 19
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    srliw x14, x1, 14
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 131072
    bne x6, x7, failed

srliw_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 0xffffffff80000001
    srliw x14, x1, 31
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x6, x7, failed

srliw_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    srliw x14, x1, 7
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16777216
    bne x14, x7, failed

srliw_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    nop
    srliw x14, x1, 14
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 131072
    bne x14, x7, failed

srliw_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 0xffffffff80000001
    nop    
    nop
    srliw x14, x1, 31
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

srliw_24:
    li  gp, 24
    srliw x1, x0, 31
    li  x7, 0
    bne x1, x7, failed

srliw_25:
    li  gp, 25
    li  x1, 31
    srliw x0, x1, 28
    li  x7, 0
    bne x0, x7, failed

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
