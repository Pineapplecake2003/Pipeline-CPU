.data
tdat:
tdat1:  .word 0xdeadbeef
tdat2:  .word 0xdeadbeef
tdat3:  .word 0xdeadbeef
tdat4:  .word 0xdeadbeef
tdat5:  .word 0xdeadbeef
tdat6:  .word 0xdeadbeef
tdat7:  .word 0xdeadbeef
tdat8:  .word 0xdeadbeef
tdat9:  .word 0xdeadbeef
tdat10: .word 0xdeadbeef

.text
.globl main

main:
# ######################################
# ### Load address of _answer to s0 
# ######################################
    mv  x28, s0
    mv  x29, ra

# ######################################
# ### sw test cases
# ######################################

#-------------------------------------------------------------
# Basic tests
#-------------------------------------------------------------

sw_2:
    li  gp, 2
    la  x2, tdat
    li  x1, 0x0000000000aa00aa
    la  x15, 7f
    sw x1, 0(x2)
    lw x14, 0(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 11141290
    bne x14, x7, failed

sw_3:
    li  gp, 3
    la  x2, tdat
    li  x1, 0xffffffffaa00aa00
    la  x15, 7f
    sw x1, 4(x2)
    lw x14, 4(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffaa00aa00
    bne x14, x7, failed

sw_4:
    li  gp, 4
    la  x2, tdat
    li  x1, 0x000000000aa00aa0
    la  x15, 7f
    sw x1, 8(x2)
    lw x14, 8(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 178260640
    bne x14, x7, failed

sw_5:
    li  gp, 5
    la  x2, tdat
    li  x1, 0xffffffffa00aa00a
    la  x15, 7f
    sw x1, 12(x2)
    lw x14, 12(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffa00aa00a
    bne x14, x7, failed

# Test with negative offset

sw_6:
    li  gp, 6
    la  x2, tdat8
    li  x1, 0x0000000000aa00aa
    la  x15, 7f
    sw x1, -12(x2)
    lw x14, -12(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 11141290
    bne x14, x7, failed

sw_7:
    li  gp, 7
    la  x2, tdat8
    li  x1, 0xffffffffaa00aa00
    la  x15, 7f
    sw x1, -8(x2)
    lw x14, -8(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffaa00aa00
    bne x14, x7, failed

sw_8:
    li  gp, 8
    la  x2, tdat8
    li  x1, 0x000000000aa00aa0
    la  x15, 7f
    sw x1, -4(x2)
    lw x14, -4(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 178260640
    bne x14, x7, failed

sw_9:
    li  gp, 9
    la  x2, tdat8
    li  x1, 0xffffffffa00aa00a
    la  x15, 7f
    sw x1, 0(x2)
    lw x14, 0(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xffffffffa00aa00a
    bne x14, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

sw_12:
    li  gp, 12
    li  x4, 0
1:  li  x13, 0xffffffffaabbccdd
    la  x12, tdat
    sw x13, 0(x12)
    lw x14, 0(x12)
    li  x7, 0xffffffffaabbccdd
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sw_13:
    li  gp, 13
    li  x4, 0
1:  li  x13, 0xffffffffdaabbccd
    la  x12, tdat
    nop
    sw x13, 4(x12)
    lw x14, 4(x12)
    li  x7, 0xffffffffdaabbccd
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sw_14:
    li  gp, 14
    li  x4, 0
1:  li  x13, 0xffffffffddaabbcc
    la  x12, tdat
    nop    
    nop
    sw x13, 8(x12)
    lw x14, 8(x12)
    li  x7, 0xffffffffddaabbcc
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sw_15:
    li  gp, 15
    li  x4, 0
1:  li  x13, 0xffffffffcddaabbc
    nop
    la  x12, tdat
    sw x13, 12(x12)
    lw x14, 12(x12)
    li  x7, 0xffffffffcddaabbc
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sw_16:
    li  gp, 16
    li  x4, 0
1:  li  x13, 0xffffffffccddaabb
    nop
    la  x12, tdat
    nop
    sw x13, 16(x12)
    lw x14, 16(x12)
    li  x7, 0xffffffffccddaabb
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sw_17:
    li  gp, 17
    li  x4, 0
1:  li  x13, 0xffffffffbccddaab
    nop    
    nop
    la  x12, tdat
    sw x13, 20(x12)
    lw x14, 20(x12)
    li  x7, 0xffffffffbccddaab
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sw_18:
    li  gp, 18
    li  x4, 0
1:  la  x2, tdat
    li  x1, 0x00112233
    sw x1, 0(x2)
    lw x14, 0(x2)
    li  x7, 0x00112233
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sw_19:
    li  gp, 19
    li  x4, 0
1:  la  x2, tdat
    li  x1, 0x30011223
    nop
    sw x1, 4(x2)
    lw x14, 4(x2)
    li  x7, 0x30011223
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sw_20:
    li  gp, 20
    li  x4, 0
1:  la  x2, tdat
    li  x1, 0x33001122
    nop    
    nop
    sw x1, 8(x2)
    lw x14, 8(x2)
    li  x7, 0x33001122
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sw_21:
    li  gp, 21
    li  x4, 0
1:  la  x2, tdat
    nop
    li  x1, 0x23300112
    sw x1, 12(x2)
    lw x14, 12(x2)
    li  x7, 0x23300112
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sw_22:
    li  gp, 22
    li  x4, 0
1:  la  x2, tdat
    nop
    li  x1, 0x22330011
    nop
    sw x1, 16(x2)
    lw x14, 16(x2)
    li  x7, 0x22330011
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sw_23:
    li  gp, 23
    li  x4, 0
1:  la  x2, tdat
    nop    
    nop
    li  x1, 0x12233001
    sw x1, 20(x2)
    lw x14, 20(x2)
    li  x7, 0x12233001
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
