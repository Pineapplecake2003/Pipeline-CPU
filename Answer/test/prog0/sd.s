.data
tdat:
tdat1:  .dword 0xdeadbeefdeadbeef
tdat2:  .dword 0xdeadbeefdeadbeef
tdat3:  .dword 0xdeadbeefdeadbeef
tdat4:  .dword 0xdeadbeefdeadbeef
tdat5:  .dword 0xdeadbeefdeadbeef
tdat6:  .dword 0xdeadbeefdeadbeef
tdat7:  .dword 0xdeadbeefdeadbeef
tdat8:  .dword 0xdeadbeefdeadbeef
tdat9:  .dword 0xdeadbeefdeadbeef
tdat10: .dword 0xdeadbeefdeadbeef

.text
.globl main

main:
# ######################################
# ### Load address of _answer to s0 
# ######################################
    mv  x28, s0
    mv  x29, ra

# ######################################
# ### sd test cases
# ######################################

#-------------------------------------------------------------
# Basic tests
#-------------------------------------------------------------

sd_2:
    li  gp, 2
    la  x2, tdat
    li  x1, 0x00aa00aa00aa00aa
    la  x15, 7f
    sd x1, 0(x2)
    ld x14, 0(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 47851476196393130
    bne x14, x7, failed

sd_3:
    li  gp, 3
    la  x2, tdat
    li  x1, 0xaa00aa00aa00aa00
    la  x15, 7f
    sd x1, 8(x2)
    ld x14, 8(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xaa00aa00aa00aa00
    bne x14, x7, failed

sd_4:
    li  gp, 4
    la  x2, tdat
    li  x1, 0x0aa00aa00aa00aa0
    la  x15, 7f
    sd x1, 16(x2)
    ld x14, 16(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 765623619142290080
    bne x14, x7, failed

sd_5:
    li  gp, 5
    la  x2, tdat
    li  x1, 0xa00aa00aa00aa00a
    la  x15, 7f
    sd x1, 24(x2)
    ld x14, 24(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xa00aa00aa00aa00a
    bne x14, x7, failed

# Test with negative offset

sd_6:
    li  gp, 6
    la  x2, tdat8
    li  x1, 0x00aa00aa00aa00aa
    la  x15, 7f
    sd x1, -24(x2)
    ld x14, -24(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 47851476196393130
    bne x14, x7, failed

sd_7:
    li  gp, 7
    la  x2, tdat8
    li  x1, 0xaa00aa00aa00aa00
    la  x15, 7f
    sd x1, -16(x2)
    ld x14, -16(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xaa00aa00aa00aa00
    bne x14, x7, failed

sd_8:
    li  gp, 8
    la  x2, tdat8
    li  x1, 0x0aa00aa00aa00aa0
    la  x15, 7f
    sd x1, -8(x2)
    ld x14, -8(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 765623619142290080
    bne x14, x7, failed

sd_9:
    li  gp, 9
    la  x2, tdat8
    li  x1, 0xa00aa00aa00aa00a
    la  x15, 7f
    sd x1, 0(x2)
    ld x14, 0(x2)
    j 8f
7:    
    mv x14, x1
8:    
    li  x7, 0xa00aa00aa00aa00a
    bne x14, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

sd_12:
    li  gp, 12
    li  x4, 0
1:  li  x13, 0xabbccdd
    la  x12, tdat
    sd x13, 0(x12)
    ld x14, 0(x12)
    li  x7, 0xabbccdd
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sd_13:
    li  gp, 13
    li  x4, 0
1:  li  x13, 0xaabbccd
    la  x12, tdat
    nop
    sd x13, 8(x12)
    ld x14, 8(x12)
    li  x7, 0xaabbccd
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sd_14:
    li  gp, 14
    li  x4, 0
1:  li  x13, 0xdaabbcc
    la  x12, tdat
    nop    
    nop
    sd x13, 16(x12)
    ld x14, 16(x12)
    li  x7, 0xdaabbcc
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sd_15:
    li  gp, 15
    li  x4, 0
1:  li  x13, 0xddaabbc
    nop
    la  x12, tdat
    sd x13, 24(x12)
    ld x14, 24(x12)
    li  x7, 0xddaabbc
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sd_16:
    li  gp, 16
    li  x4, 0
1:  li  x13, 0xcddaabb
    nop
    la  x12, tdat
    nop
    sd x13, 32(x12)
    ld x14, 32(x12)
    li  x7, 0xcddaabb
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sd_17:
    li  gp, 17
    li  x4, 0
1:  li  x13, 0xccddaab
    nop    
    nop
    la  x12, tdat
    sd x13, 40(x12)
    ld x14, 40(x12)
    li  x7, 0xccddaab
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sd_18:
    li  gp, 18
    li  x4, 0
1:  la  x2, tdat
    li  x1, 0x00112233
    sd x1, 0(x2)
    ld x14, 0(x2)
    li  x7, 0x00112233
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sd_19:
    li  gp, 19
    li  x4, 0
1:  la  x2, tdat
    li  x1, 0x30011223
    nop
    sd x1, 8(x2)
    ld x14, 8(x2)
    li  x7, 0x30011223
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sd_20:
    li  gp, 20
    li  x4, 0
1:  la  x2, tdat
    li  x1, 0x33001122
    nop    
    nop
    sd x1, 16(x2)
    ld x14, 16(x2)
    li  x7, 0x33001122
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sd_21:
    li  gp, 21
    li  x4, 0
1:  la  x2, tdat
    nop
    li  x1, 0x23300112
    sd x1, 24(x2)
    ld x14, 24(x2)
    li  x7, 0x23300112
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sd_22:
    li  gp, 22
    li  x4, 0
1:  la  x2, tdat
    nop
    li  x1, 0x22330011
    nop
    sd x1, 32(x2)
    ld x14, 32(x2)
    li  x7, 0x22330011
    bne x14, x7, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

sd_23:
    li  gp, 23
    li  x4, 0
1:  la  x2, tdat
    nop    
    nop
    li  x1, 0x12233001
    sd x1, 40(x2)
    ld x14, 40(x2)
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
