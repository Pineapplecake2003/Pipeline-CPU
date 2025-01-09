.data
tdat:
tdat1:  .byte 0xff
tdat2:  .byte 0x00
tdat3:  .byte 0xf0
tdat4:  .byte 0x0f

.text
.globl main

main:
# ######################################
# ### Load address of _answer to s0 
# ######################################
    mv  x28, s0
    mv  x29, ra

# ######################################
# ### lbu test cases
# ######################################

#-------------------------------------------------------------
# Basic tests
#-------------------------------------------------------------

lbu_2:
    li  gp, 2
    li  x15, 0x00000000000000ff
    la  x2, tdat
    lbu x14, 0(x2)
    li  x7, 255
    bne x14, x7, failed

lbu_3:
    li  gp, 3
    li  x15, 0x0000000000000000
    la  x2, tdat
    lbu x14, 1(x2)
    li  x7, 0
    bne x14, x7, failed

lbu_4:
    li  gp, 4
    li  x15, 0x00000000000000f0
    la  x2, tdat
    lbu x14, 2(x2)
    li  x7, 240
    bne x14, x7, failed

lbu_5:
    li  gp, 5
    li  x15, 0x000000000000000f
    la  x2, tdat
    lbu x14, 3(x2)
    li  x7, 15
    bne x14, x7, failed

# Test with negative offset

lbu_6:
    li  gp, 6
    li  x15, 0x00000000000000ff
    la  x2, tdat4
    lbu x14, -3(x2)
    li  x7, 255
    bne x14, x7, failed

lbu_7:
    li  gp, 7
    li  x15, 0x0000000000000000
    la  x2, tdat4
    lbu x14, -2(x2)
    li  x7, 0
    bne x14, x7, failed

lbu_8:
    li  gp, 8
    li  x15, 0x00000000000000f0
    la  x2, tdat4
    lbu x14, -1(x2)
    li  x7, 240
    bne x14, x7, failed

lbu_9:
    li  gp, 9
    li  x15, 0x000000000000000f
    la  x2, tdat4
    lbu x14, 0(x2)
    li  x7, 15
    bne x14, x7, failed

# Test with a negative base

lbu_10:
    li  gp, 10
    la  x1, tdat
    addi x1, x1, -32
    lbu x5, 32(x1)
    li  x7, 0x00000000000000ff
    bne x5, x7, failed

# Test with unaligned base

lbu_11:
    li  gp, 11
    la  x1, tdat
    addi x1, x1, -6
    lbu x5, 7(x1)
    li  x7, 0x0000000000000000
    bne x5, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

lbu_12:
    li  gp, 12
    li  x4, 0
1:  la  x13, tdat2
    lbu x14, 1(x13)
    addi  x6, x14, 0
    li  x7, 0x00000000000000f0
    bne x6, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lbu_13:
    li  gp, 13
    li  x4, 0
1:  la  x13, tdat3
    lbu x14, 1(x13)
    nop
    addi  x6, x14, 0
    li  x7, 0x000000000000000f
    bne x6, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lbu_14:
    li  gp, 14
    li  x4, 0
1:  la  x13, tdat1
    lbu x14, 1(x13)
    nop    
    nop
    addi  x6, x14, 0
    li  x7, 0x0000000000000000
    bne x6, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lbu_15:
    li  gp, 15
    li  x4, 0
1:  la  x13, tdat2
    lbu x14, 1(x13)
    li  x7, 0x00000000000000f0
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lbu_16:
    li  gp, 16
    li  x4, 0
1:  la  x13, tdat3
    nop
    lbu x14, 1(x13)
    li  x7, 0x000000000000000f
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lbu_17:
    li  gp, 17
    li  x4, 0
1:  la  x13, tdat1
    nop    
    nop
    lbu x14, 1(x13)
    li  x7, 0x0000000000000000
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

#-------------------------------------------------------------
# Test write-after-write hazard
#-------------------------------------------------------------

lbu_18:
    li  gp, 18
    la  x5, tdat
    lbu  x2, 0(x5)
    li  x2, 2
    li  x7, 2
    bne x2, x7, failed

lbu_19:
    li  gp, 19
    la  x5, tdat
    lbu  x2, 0(x5)
    nop
    li  x2, 2
    li  x7, 2
    bne x2, x7, failed

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
