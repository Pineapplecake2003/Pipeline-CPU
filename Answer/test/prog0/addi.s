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
# ### addi test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

addi_2:
    li  gp, 2
    li  x13, 0
    addi x14, x13, 0
    li  x7, 0
    bne x14, x7, failed

addi_3:
    li  gp, 3
    li  x13, 1
    addi x14, x13, 1
    li  x7, 2
    bne x14, x7, failed

addi_4:
    li  gp, 4
    li  x13, 3
    addi x14, x13, 7
    li  x7, 10
    bne x14, x7, failed

addi_5:
    li  gp, 5
    li  x13, 0
    addi x14, x13, -2048
    li  x7, 0xfffffffffffff800
    bne x14, x7, failed

addi_6:
    li  gp, 6
    li  x13, 0xffffffff80000000
    addi x14, x13, 0
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

addi_7:
    li  gp, 7
    li  x13, 0xffffffff80000000
    addi x14, x13, -2048
    li  x7, 0xffffffff7ffff800
    bne x14, x7, failed

addi_8:
    li  gp, 8
    li  x13, 0
    addi x14, x13, 2047
    li  x7, 2047
    bne x14, x7, failed

addi_9:
    li  gp, 9
    li  x13, 2147483647
    addi x14, x13, 0
    li  x7, 2147483647
    bne x14, x7, failed

addi_10:
    li  gp, 10
    li  x13, 2147483647
    addi x14, x13, 2047
    li  x7, 2147485694
    bne x14, x7, failed

addi_11:
    li  gp, 11
    li  x13, 0xffffffff80000000
    addi x14, x13, 2047
    li  x7, 0xffffffff800007ff
    bne x14, x7, failed

addi_12:
    li  gp, 12
    li  x13, 2147483647
    addi x14, x13, -2048
    li  x7, 2147481599
    bne x14, x7, failed

addi_13:
    li  gp, 13
    li  x13, 0
    addi x14, x13, -1
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

addi_14:
    li  gp, 14
    li  x13, 0xffffffffffffffff
    addi x14, x13, 1
    li  x7, 0
    bne x14, x7, failed

addi_15:
    li  gp, 15
    li  x13, 0xffffffffffffffff
    addi x14, x13, -1
    li  x7, 0xfffffffffffffffe
    bne x14, x7, failed

addi_16:
    li  gp, 16
    li  x13, 2147483647
    addi x14, x13, 1
    li  x7, 2147483648
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

addi_17:
    li  gp, 17
    li  x11, 13
    addi x11, x11, 11
    li  x7, 24
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

addi_18:
    li  gp, 18
    li  x4, 0
1:  li  x1, 13
    addi x14, x1, 11
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 24
    bne x6, x7, failed

addi_19:
    li  gp, 19
    li  x4, 0
1:  li  x1, 13
    addi x14, x1, 10
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 23
    bne x6, x7, failed

addi_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 13
    addi x14, x1, 9
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 22
    bne x6, x7, failed

addi_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 13
    addi x14, x1, 11
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 24
    bne x14, x7, failed

addi_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 13
    nop
    addi x14, x1, 10
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 23
    bne x14, x7, failed

addi_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 13
    nop    
    nop
    addi x14, x1, 9
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 22
    bne x14, x7, failed

addi_24:
    li  gp, 24
    addi x1, x0, 32
    li  x7, 32
    bne x1, x7, failed

addi_25:
    li  gp, 25
    li  x1, 33
    addi x0, x1, 50
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
