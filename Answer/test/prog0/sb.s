.data
tdat:
tdat1:  .byte 0xef
tdat2:  .byte 0xef
tdat3:  .byte 0xef
tdat4:  .byte 0xef
tdat5:  .byte 0xef
tdat6:  .byte 0xef
tdat7:  .byte 0xef
tdat8:  .byte 0xef
tdat9:  .byte 0xef
tdat10: .byte 0xef

.text
.globl main

main:
# ######################################
# ### Load address of _answer to s0 
# ######################################
    mv  x28, s0
    mv  x29, ra

# ######################################
# ### sb test cases
# ######################################

#-------------------------------------------------------------
# Basic tests
#-------------------------------------------------------------

sb_2:
    li  gp, 2
    la  x2, tdat
    li  x1, 0xffffffffffffffaa
    la  x15, 7f
    sb x1, 0(x2)
    lb x14, 0(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffffffffaa
    bne x14, x7, failed

sb_3:
    li  gp, 3
    la  x2, tdat
    li  x1, 0x0000000000000000
    la  x15, 7f
    sb x1, 1(x2)
    lb x14, 1(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0
    bne x14, x7, failed

sb_4:
    li  gp, 4
    la  x2, tdat
    li  x1, 0xffffffffffffefa0
    la  x15, 7f
    sb x1, 2(x2)
    lh x14, 2(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffffffefa0
    bne x14, x7, failed

sb_5:
    li  gp, 5
    la  x2, tdat
    li  x1, 0x000000000000000a
    la  x15, 7f
    sb x1, 3(x2)
    lb x14, 3(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 10
    bne x14, x7, failed

# Test with negative offset

sb_6:
    li  gp, 6
    la  x2, tdat8
    li  x1, 0xffffffffffffffaa
    la  x15, 7f
    sb x1, -3(x2)
    lb x14, -3(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffffffffaa
    bne x14, x7, failed

sb_7:
    li  gp, 7
    la  x2, tdat8
    li  x1, 0x0000000000000000
    la  x15, 7f
    sb x1, -2(x2)
    lb x14, -2(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0
    bne x14, x7, failed

sb_8:
    li  gp, 8
    la  x2, tdat8
    li  x1, 0xffffffffffffffa0
    la  x15, 7f
    sb x1, -1(x2)
    lb x14, -1(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffffffffa0
    bne x14, x7, failed

sb_9:
    li  gp, 9
    la  x2, tdat8
    li  x1, 0x000000000000000a
    la  x15, 7f
    sb x1, 0(x2)
    lb x14, 0(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 10
    bne x14, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

sb_12:
    li  gp, 12
    li  x4, 0
1:  li  x13, 0xffffffffffffffdd
    la  x12, tdat
    sb x13, 0(x12)
    lb x14, 0(x12)
    li  x7, 0xffffffffffffffdd
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sb_13:
    li  gp, 13
    li  x4, 0
1:  li  x13, 0xffffffffffffffcd
    la  x12, tdat
    nop
    sb x13, 1(x12)
    lb x14, 1(x12)
    li  x7, 0xffffffffffffffcd
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sb_14:
    li  gp, 14
    li  x4, 0
1:  li  x13, 0xffffffffffffffcc
    la  x12, tdat
    nop    
    nop
    sb x13, 2(x12)
    lb x14, 2(x12)
    li  x7, 0xffffffffffffffcc
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sb_15:
    li  gp, 15
    li  x4, 0
1:  li  x13, 0xffffffffffffffbc
    nop
    la  x12, tdat
    sb x13, 3(x12)
    lb x14, 3(x12)
    li  x7, 0xffffffffffffffbc
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sb_16:
    li  gp, 16
    li  x4, 0
1:  li  x13, 0xffffffffffffffbb
    nop
    la  x12, tdat
    nop
    sb x13, 4(x12)
    lb x14, 4(x12)
    li  x7, 0xffffffffffffffbb
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sb_17:
    li  gp, 17
    li  x4, 0
1:  li  x13, 0xffffffffffffffab
    nop    
    nop
    la  x12, tdat
    sb x13, 5(x12)
    lb x14, 5(x12)
    li  x7, 0xffffffffffffffab
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sb_18:
    li  gp, 18
    li  x4, 0
1:  la  x2, tdat
    li  x1, 0x33
    sb x1, 0(x2)
    lb x14, 0(x2)
    li  x7, 0x33
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sb_19:
    li  gp, 19
    li  x4, 0
1:  la  x2, tdat
    li  x1, 0x23
    nop
    sb x1, 1(x2)
    lb x14, 1(x2)
    li  x7, 0x23
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sb_20:
    li  gp, 20
    li  x4, 0
1:  la  x2, tdat
    li  x1, 0x22
    nop    
    nop
    sb x1, 2(x2)
    lb x14, 2(x2)
    li  x7, 0x22
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sb_21:
    li  gp, 21
    li  x4, 0
1:  la  x2, tdat
    nop
    li  x1, 0x12
    sb x1, 3(x2)
    lb x14, 3(x2)
    li  x7, 0x12
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sb_22:
    li  gp, 22
    li  x4, 0
1:  la  x2, tdat
    nop
    li  x1, 0x11
    nop
    sb x1, 4(x2)
    lb x14, 4(x2)
    li  x7, 0x11
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sb_23:
    li  gp, 23
    li  x4, 0
1:  la  x2, tdat
    nop    
    nop
    li  x1, 0x01
    sb x1, 5(x2)
    lb x14, 5(x2)
    li  x7, 0x01
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

    li a0, 0xef
    la a1, tdat
    sb a0, 3(a1)

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
