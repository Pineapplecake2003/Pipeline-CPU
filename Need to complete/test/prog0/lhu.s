.data
tdat:
tdat1:  .half 0x00ff
tdat2:  .half 0xff00
tdat3:  .half 0x0ff0
tdat4:  .half 0xf00f

.text
.globl main

main:
# ######################################
# ### Load address of _answer to s0 
# ######################################
    mv  x28, s0
    mv  x29, ra

# ######################################
# ### lhu test cases
# ######################################

#-------------------------------------------------------------
# Basic tests
#-------------------------------------------------------------

lhu_2:
    li  gp, 2
    li  x15, 0x00000000000000ff
    la  x2, tdat
    lhu x14, 0(x2)
    li  x7, 255
    bne x14, x7, failed

lhu_3:
    li  gp, 3
    li  x15, 0x000000000000ff00
    la  x2, tdat
    lhu x14, 2(x2)
    li  x7, 65280
    bne x14, x7, failed

lhu_4:
    li  gp, 4
    li  x15, 0x0000000000000ff0
    la  x2, tdat
    lhu x14, 4(x2)
    li  x7, 4080
    bne x14, x7, failed

lhu_5:
    li  gp, 5
    li  x15, 0x000000000000f00f
    la  x2, tdat
    lhu x14, 6(x2)
    li  x7, 61455
    bne x14, x7, failed

# Test with negative offset

lhu_6:
    li  gp, 6
    li  x15, 0x00000000000000ff
    la  x2, tdat4
    lhu x14, -6(x2)
    li  x7, 255
    bne x14, x7, failed

lhu_7:
    li  gp, 7
    li  x15, 0x000000000000ff00
    la  x2, tdat4
    lhu x14, -4(x2)
    li  x7, 65280
    bne x14, x7, failed

lhu_8:
    li  gp, 8
    li  x15, 0x0000000000000ff0
    la  x2, tdat4
    lhu x14, -2(x2)
    li  x7, 4080
    bne x14, x7, failed

lhu_9:
    li  gp, 9
    li  x15, 0x000000000000f00f
    la  x2, tdat4
    lhu x14, 0(x2)
    li  x7, 61455
    bne x14, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

lhu_12:
    li  gp, 12
    li  x4, 0
1:  la  x13, tdat2
    lhu x14, 2(x13)
    addi  x6, x14, 0
    li  x7, 0x0000000000000ff0
    bne x6, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lhu_13:
    li  gp, 13
    li  x4, 0
1:  la  x13, tdat3
    lhu x14, 2(x13)
    nop
    addi  x6, x14, 0
    li  x7, 0x000000000000f00f
    bne x6, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lhu_14:
    li  gp, 14
    li  x4, 0
1:  la  x13, tdat1
    lhu x14, 2(x13)
    nop    
    nop
    addi  x6, x14, 0
    li  x7, 0x000000000000ff00
    bne x6, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lhu_15:
    li  gp, 15
    li  x4, 0
1:  la  x13, tdat2
    lhu x14, 2(x13)
    li  x7, 0x0000000000000ff0
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lhu_16:
    li  gp, 16
    li  x4, 0
1:  la  x13, tdat3
    nop
    lhu x14, 2(x13)
    li  x7, 0x000000000000f00f
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lhu_17:
    li  gp, 17
    li  x4, 0
1:  la  x13, tdat1
    nop    
    nop
    lhu x14, 2(x13)
    li  x7, 0x000000000000ff00
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

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
