.data
tdat:
tdat1:  .word 0x00ff00ff
tdat2:  .word 0xff00ff00
tdat3:  .word 0x0ff00ff0
tdat4:  .word 0xf00ff00f

.text
.globl main

main:
# ######################################
# ### Load address of _answer to s0 
# ######################################
    mv  x28, s0
    mv  x29, ra

# ######################################
# ### lwu test cases
# ######################################

#-------------------------------------------------------------
# Basic tests
#-------------------------------------------------------------

lwu_2:
    li  gp, 2
    li  x15, 0x0000000000ff00ff
    la  x2, tdat
    lwu x14, 0(x2)
    li  x7, 16711935
    bne x14, x7, failed

lwu_3:
    li  gp, 3
    li  x15, 0x00000000ff00ff00
    la  x2, tdat
    lwu x14, 4(x2)
    li  x7, 4278255360
    bne x14, x7, failed

lwu_4:
    li  gp, 4
    li  x15, 0x000000000ff00ff0
    la  x2, tdat
    lwu x14, 8(x2)
    li  x7, 267390960
    bne x14, x7, failed

lwu_5:
    li  gp, 5
    li  x15, 0x00000000f00ff00f
    la  x2, tdat
    lwu x14, 12(x2)
    li  x7, 4027576335
    bne x14, x7, failed

# Test with negative offset

lwu_6:
    li  gp, 6
    li  x15, 0x0000000000ff00ff
    la  x2, tdat4
    lwu x14, -12(x2)
    li  x7, 16711935
    bne x14, x7, failed

lwu_7:
    li  gp, 7
    li  x15, 0x00000000ff00ff00
    la  x2, tdat4
    lwu x14, -8(x2)
    li  x7, 4278255360
    bne x14, x7, failed

lwu_8:
    li  gp, 8
    li  x15, 0x000000000ff00ff0
    la  x2, tdat4
    lwu x14, -4(x2)
    li  x7, 267390960
    bne x14, x7, failed

lwu_9:
    li  gp, 9
    li  x15, 0x00000000f00ff00f
    la  x2, tdat4
    lwu x14, 0(x2)
    li  x7, 4027576335
    bne x14, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

lwu_12:
    li  gp, 12
    li  x4, 0
1:  la  x13, tdat2
    lwu x14, 4(x13)
    addi  x6, x14, 0
    li  x7, 0x000000000ff00ff0
    bne x6, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lwu_13:
    li  gp, 13
    li  x4, 0
1:  la  x13, tdat3
    lwu x14, 4(x13)
    nop
    addi  x6, x14, 0
    li  x7, 0x00000000f00ff00f
    bne x6, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lwu_14:
    li  gp, 14
    li  x4, 0
1:  la  x13, tdat1
    lwu x14, 4(x13)
    nop    
    nop
    addi  x6, x14, 0
    li  x7, 0x00000000ff00ff00
    bne x6, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lwu_15:
    li  gp, 15
    li  x4, 0
1:  la  x13, tdat2
    lwu x14, 4(x13)
    li  x7, 0x000000000ff00ff0
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lwu_16:
    li  gp, 16
    li  x4, 0
1:  la  x13, tdat3
    nop
    lwu x14, 4(x13)
    li  x7, 0x00000000f00ff00f
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

lwu_17:
    li  gp, 17
    li  x4, 0
1:  la  x13, tdat1
    nop    
    nop
    lwu x14, 4(x13)
    li  x7, 0x00000000ff00ff00
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
