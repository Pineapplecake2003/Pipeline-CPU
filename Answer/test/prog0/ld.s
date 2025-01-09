.data
tdat:
tdat1:  .dword 0x00ff00ff00ff00ff
tdat2:  .dword 0xff00ff00ff00ff00
tdat3:  .dword 0x0ff00ff00ff00ff0
tdat4:  .dword 0xf00ff00ff00ff00f

.text
.globl main

main:
# ######################################
# ### Load address of _answer to s0 
# ######################################
    mv  x28, s0
    mv  x29, ra

# ######################################
# ### ld test cases
# ######################################

#-------------------------------------------------------------
# Basic tests
#-------------------------------------------------------------

ld_2:
    li  gp, 2
    li  x15, 0x00ff00ff00ff00ff
    la  x2, tdat
    ld x14, 0(x2)
    li  x7, 71777214294589695
    bne x14, x7, failed

ld_3:
    li  gp, 3
    li  x15, 0xff00ff00ff00ff00
    la  x2, tdat
    ld x14, 8(x2)
    li  x7, 0xff00ff00ff00ff00
    bne x14, x7, failed

ld_4:
    li  gp, 4
    li  x15, 0x0ff00ff00ff00ff0
    la  x2, tdat
    ld x14, 16(x2)
    li  x7, 1148435428713435120
    bne x14, x7, failed

ld_5:
    li  gp, 5
    li  x15, 0xf00ff00ff00ff00f
    la  x2, tdat
    ld x14, 24(x2)
    li  x7, 0xf00ff00ff00ff00f
    bne x14, x7, failed

# Test with negative offset

ld_6:
    li  gp, 6
    li  x15, 0x00ff00ff00ff00ff
    la  x2, tdat4
    ld x14, -24(x2)
    li  x7, 71777214294589695
    bne x14, x7, failed

ld_7:
    li  gp, 7
    li  x15, 0xff00ff00ff00ff00
    la  x2, tdat4
    ld x14, -16(x2)
    li  x7, 0xff00ff00ff00ff00
    bne x14, x7, failed

ld_8:
    li  gp, 8
    li  x15, 0x0ff00ff00ff00ff0
    la  x2, tdat4
    ld x14, -8(x2)
    li  x7, 1148435428713435120
    bne x14, x7, failed

ld_9:
    li  gp, 9
    li  x15, 0xf00ff00ff00ff00f
    la  x2, tdat4
    ld x14, 0(x2)
    li  x7, 0xf00ff00ff00ff00f
    bne x14, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

ld_12:
    li  gp, 12
    li  x4, 0
1:  la  x13, tdat2
    ld x14, 8(x13)
    addi  x6, x14, 0
    li  x7, 0x0ff00ff00ff00ff0
    bne x6, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

ld_13:
    li  gp, 13
    li  x4, 0
1:  la  x13, tdat3
    ld x14, 8(x13)
    nop
    addi  x6, x14, 0
    li  x7, 0xf00ff00ff00ff00f
    bne x6, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

ld_14:
    li  gp, 14
    li  x4, 0
1:  la  x13, tdat1
    ld x14, 8(x13)
    nop    
    nop
    addi  x6, x14, 0
    li  x7, 0xff00ff00ff00ff00
    bne x6, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

ld_15:
    li  gp, 15
    li  x4, 0
1:  la  x13, tdat2
    ld x14, 8(x13)
    li  x7, 0x0ff00ff00ff00ff0
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

ld_16:
    li  gp, 16
    li  x4, 0
1:  la  x13, tdat3
    nop
    ld x14, 8(x13)
    li  x7, 0xf00ff00ff00ff00f
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

ld_17:
    li  gp, 17
    li  x4, 0
1:  la  x13, tdat1
    nop    
    nop
    ld x14, 8(x13)
    li  x7, 0xff00ff00ff00ff00
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
