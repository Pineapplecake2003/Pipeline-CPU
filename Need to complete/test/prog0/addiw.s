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
# ### addiw test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

addiw_2:
    li  gp, 2
    li  x13, 0
    addiw x14, x13, 0
    li  x7, 0
    bne x14, x7, failed

addiw_3:
    li  gp, 3
    li  x13, 1
    addiw x14, x13, 1
    li  x7, 2
    bne x14, x7, failed

addiw_4:
    li  gp, 4
    li  x13, 3
    addiw x14, x13, 7
    li  x7, 10
    bne x14, x7, failed

addiw_5:
    li  gp, 5
    li  x13, 0
    addiw x14, x13, -2048
    li  x7, 0xfffffffffffff800
    bne x14, x7, failed

addiw_6:
    li  gp, 6
    li  x13, 0xffffffff80000000
    addiw x14, x13, 0
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

addiw_7:
    li  gp, 7
    li  x13, 0xffffffff80000000
    addiw x14, x13, -2048
    li  x7, 2147481600
    bne x14, x7, failed

addiw_8:
    li  gp, 8
    li  x13, 0
    addiw x14, x13, 2047
    li  x7, 2047
    bne x14, x7, failed

addiw_9:
    li  gp, 9
    li  x13, 2147483647
    addiw x14, x13, 0
    li  x7, 2147483647
    bne x14, x7, failed

addiw_10:
    li  gp, 10
    li  x13, 2147483647
    addiw x14, x13, 2047
    li  x7, 0xffffffff800007fe
    bne x14, x7, failed

addiw_11:
    li  gp, 11
    li  x13, 0xffffffff80000000
    addiw x14, x13, 2047
    li  x7, 0xffffffff800007ff
    bne x14, x7, failed

addiw_12:
    li  gp, 12
    li  x13, 2147483647
    addiw x14, x13, -2048
    li  x7, 2147481599
    bne x14, x7, failed

addiw_13:
    li  gp, 13
    li  x13, 0
    addiw x14, x13, -1
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

addiw_14:
    li  gp, 14
    li  x13, 0xffffffffffffffff
    addiw x14, x13, 1
    li  x7, 0
    bne x14, x7, failed

addiw_15:
    li  gp, 15
    li  x13, 0xffffffffffffffff
    addiw x14, x13, -1
    li  x7, 0xfffffffffffffffe
    bne x14, x7, failed

addiw_16:
    li  gp, 16
    li  x13, 2147483647
    addiw x14, x13, 1
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

addiw_17:
    li  gp, 17
    li  x11, 13
    addiw x11, x11, 11
    li  x7, 24
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

addiw_18:
    li  gp, 18
    li  x4, 0
1:  li  x1, 13
    addiw x14, x1, 11
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 24
    bne x6, x7, failed

addiw_19:
    li  gp, 19
    li  x4, 0
1:  li  x1, 13
    addiw x14, x1, 10
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 23
    bne x6, x7, failed

addiw_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 13
    addiw x14, x1, 9
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 22
    bne x6, x7, failed

addiw_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 13
    addiw x14, x1, 11
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 24
    bne x14, x7, failed

addiw_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 13
    nop
    addiw x14, x1, 10
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 23
    bne x14, x7, failed

addiw_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 13
    nop    
    nop
    addiw x14, x1, 9
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 22
    bne x14, x7, failed

addiw_24:
    li  gp, 24
    addiw x1, x0, 32
    li  x7, 32
    bne x1, x7, failed

addiw_25:
    li  gp, 25
    li  x1, 33
    addiw x0, x1, 50
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
