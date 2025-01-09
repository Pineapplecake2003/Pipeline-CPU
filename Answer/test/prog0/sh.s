.data
tdat:
tdat1:  .half 0xbeef
tdat2:  .half 0xbeef
tdat3:  .half 0xbeef
tdat4:  .half 0xbeef
tdat5:  .half 0xbeef
tdat6:  .half 0xbeef
tdat7:  .half 0xbeef
tdat8:  .half 0xbeef
tdat9:  .half 0xbeef
tdat10: .half 0xbeef

.text
.globl main

main:
# ######################################
# ### Load address of _answer to s0 
# ######################################
    mv  x28, s0
    mv  x29, ra

# ######################################
# ### sh test cases
# ######################################

#-------------------------------------------------------------
# Basic tests
#-------------------------------------------------------------

sh_2:
    li  gp, 2
    la  x2, tdat
    li  x1, 0x00000000000000aa
    la  x15, 7f
    sh x1, 0(x2)
    lh x14, 0(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 170
    bne x14, x7, failed

sh_3:
    li  gp, 3
    la  x2, tdat
    li  x1, 0xffffffffffffaa00
    la  x15, 7f
    sh x1, 2(x2)
    lh x14, 2(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffffffaa00
    bne x14, x7, failed

sh_4:
    li  gp, 4
    la  x2, tdat
    li  x1, 0xffffffffbeef0aa0
    la  x15, 7f
    sh x1, 4(x2)
    lw x14, 4(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffbeef0aa0
    bne x14, x7, failed

sh_5:
    li  gp, 5
    la  x2, tdat
    li  x1, 0xffffffffffffa00a
    la  x15, 7f
    sh x1, 6(x2)
    lh x14, 6(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffffffa00a
    bne x14, x7, failed

# Test with negative offset

sh_6:
    li  gp, 6
    la  x2, tdat8
    li  x1, 0x00000000000000aa
    la  x15, 7f
    sh x1, -6(x2)
    lh x14, -6(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 170
    bne x14, x7, failed

sh_7:
    li  gp, 7
    la  x2, tdat8
    li  x1, 0xffffffffffffaa00
    la  x15, 7f
    sh x1, -4(x2)
    lh x14, -4(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffffffaa00
    bne x14, x7, failed

sh_8:
    li  gp, 8
    la  x2, tdat8
    li  x1, 0x0000000000000aa0
    la  x15, 7f
    sh x1, -2(x2)
    lh x14, -2(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 2720
    bne x14, x7, failed

sh_9:
    li  gp, 9
    la  x2, tdat8
    li  x1, 0xffffffffffffa00a
    la  x15, 7f
    sh x1, 0(x2)
    lh x14, 0(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffffffa00a
    bne x14, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

sh_12:
    li  gp, 12
    li  x4, 0
1:  li  x13, 0xffffffffffffccdd
    la  x12, tdat
    sh x13, 0(x12)
    lh x14, 0(x12)
    li  x7, 0xffffffffffffccdd
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sh_13:
    li  gp, 13
    li  x4, 0
1:  li  x13, 0xffffffffffffbccd
    la  x12, tdat
    nop
    sh x13, 2(x12)
    lh x14, 2(x12)
    li  x7, 0xffffffffffffbccd
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sh_14:
    li  gp, 14
    li  x4, 0
1:  li  x13, 0xffffffffffffbbcc
    la  x12, tdat
    nop    
    nop
    sh x13, 4(x12)
    lh x14, 4(x12)
    li  x7, 0xffffffffffffbbcc
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sh_15:
    li  gp, 15
    li  x4, 0
1:  li  x13, 0xffffffffffffabbc
    nop
    la  x12, tdat
    sh x13, 6(x12)
    lh x14, 6(x12)
    li  x7, 0xffffffffffffabbc
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sh_16:
    li  gp, 16
    li  x4, 0
1:  li  x13, 0xffffffffffffaabb
    nop
    la  x12, tdat
    nop
    sh x13, 8(x12)
    lh x14, 8(x12)
    li  x7, 0xffffffffffffaabb
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sh_17:
    li  gp, 17
    li  x4, 0
1:  li  x13, 0xffffffffffffdaab
    nop    
    nop
    la  x12, tdat
    sh x13, 10(x12)
    lh x14, 10(x12)
    li  x7, 0xffffffffffffdaab
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sh_18:
    li  gp, 18
    li  x4, 0
1:  la  x2, tdat
    li  x1, 0x2233
    sh x1, 0(x2)
    lh x14, 0(x2)
    li  x7, 0x2233
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sh_19:
    li  gp, 19
    li  x4, 0
1:  la  x2, tdat
    li  x1, 0x1223
    nop
    sh x1, 2(x2)
    lh x14, 2(x2)
    li  x7, 0x1223
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sh_20:
    li  gp, 20
    li  x4, 0
1:  la  x2, tdat
    li  x1, 0x1122
    nop    
    nop
    sh x1, 4(x2)
    lh x14, 4(x2)
    li  x7, 0x1122
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sh_21:
    li  gp, 21
    li  x4, 0
1:  la  x2, tdat
    nop
    li  x1, 0x0112
    sh x1, 6(x2)
    lh x14, 6(x2)
    li  x7, 0x0112
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sh_22:
    li  gp, 22
    li  x4, 0
1:  la  x2, tdat
    nop
    li  x1, 0x0011
    nop
    sh x1, 8(x2)
    lh x14, 8(x2)
    li  x7, 0x0011
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sh_23:
    li  gp, 23
    li  x4, 0
1:  la  x2, tdat
    nop    
    nop
    li  x1, 0x3001
    sh x1, 10(x2)
    lh x14, 10(x2)
    li  x7, 0x3001
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

    li a0, 0xbeef
    la a1, tdat
    sh a0, 6(a1)

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
