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
# ### srl test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

# Verify that shifts only use bottom six(rv64) or five(rv32) bits

srl_17:
    li  gp, 17
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffc0
    srl x14, x11, x12
    li  x7, 0x0000000021212121
    bne x14, x7, failed
    li  x7, 555819297
    bne x14, x7, failed

srl_18:
    li  gp, 18
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffc1
    srl x14, x11, x12
    li  x7, 0x0000000010909090
    bne x14, x7, failed
    li  x7, 277909648
    bne x14, x7, failed

srl_19:
    li  gp, 19
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffc7
    srl x14, x11, x12
    li  x7, 0x0000000000424242
    bne x14, x7, failed
    li  x7, 4342338
    bne x14, x7, failed

srl_20:
    li  gp, 20
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffce
    srl x14, x11, x12
    li  x7, 0x0000000000008484
    bne x14, x7, failed
    li  x7, 33924
    bne x14, x7, failed

srl_21:
    li  gp, 21
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffff
    srl x14, x11, x12
    li  x7, 0x0000000000000000
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

srl_22:
    li  gp, 22
    li  x11, 0x80000000
    li  x12, 7
    srl x11, x11, x12
    li  x7, 0x01000000
    bne x11, x7, failed
    li  x7, 16777216
    bne x11, x7, failed

srl_23:
    li  gp, 23
    li  x11, 0x80000000
    li  x12, 14
    srl x12, x11, x12
    li  x7, 0x00020000
    bne x12, x7, failed
    li  x7, 131072
    bne x12, x7, failed

srl_24:
    li  gp, 24
    li  x11, 7
    srl x11, x11, x11
    li  x7, 0
    bne x11, x7, failed
    li  x7, 0
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

srl_25:
    li  gp, 25
    li  x4, 0
1:  li  x1, 0x80000000
    li  x2, 7
    srl x14, x1, x2
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x01000000
    bne x6, x7, failed
    li  x7, 16777216
    bne x6, x7, failed

srl_26:
    li  gp, 26
    li  x4, 0
1:  li  x1, 0x80000000
    li  x2, 14
    srl x14, x1, x2
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x00020000
    bne x6, x7, failed
    li  x7, 131072
    bne x6, x7, failed

srl_27:
    li  gp, 27
    li  x4, 0
1:  li  x1, 0x80000000
    li  x2, 31
    srl x14, x1, x2
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x00000001
    bne x6, x7, failed
    li  x7, 1
    bne x6, x7, failed

srl_28:
    li  gp, 28
    li  x4, 0
1:  li  x1, 0x80000000
    li  x2, 7
    srl x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x01000000
    bne x14, x7, failed
    li  x7, 16777216
    bne x14, x7, failed

srl_29:
    li  gp, 29
    li  x4, 0
1:  li  x1, 0x80000000
    li  x2, 14
    nop
    srl x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x00020000
    bne x14, x7, failed
    li  x7, 131072
    bne x14, x7, failed

srl_30:
    li  gp, 30
    li  x4, 0
1:  li  x1, 0x80000000
    li  x2, 31
    nop    
    nop
    srl x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x00000001
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

srl_31:
    li  gp, 31
    li  x4, 0
1:  li  x1, 0x80000000
    nop
    li  x2, 7
    srl x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x01000000
    bne x14, x7, failed
    li  x7, 16777216
    bne x14, x7, failed

srl_32:
    li  gp, 32
    li  x4, 0
1:  li  x1, 0x80000000
    nop
    li  x2, 14
    nop
    srl x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x00020000
    bne x14, x7, failed
    li  x7, 131072
    bne x14, x7, failed

srl_33:
    li  gp, 33
    li  x4, 0
1:  li  x1, 0x80000000
    nop    
    nop
    li  x2, 31
    srl x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x00000001
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

srl_34:
    li  gp, 34
    li  x4, 0
1:  li  x2, 7
    li  x1, 2147483648
    srl x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16777216
    bne x14, x7, failed

srl_35:
    li  gp, 35
    li  x4, 0
1:  li  x2, 14
    li  x1, 2147483648
    nop
    srl x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 131072
    bne x14, x7, failed

srl_36:
    li  gp, 36
    li  x4, 0
1:  li  x2, 31
    li  x1, 2147483648
    nop    
    nop
    srl x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

srl_37:
    li  gp, 37
    li  x4, 0
1:  li  x2, 7
    nop
    li  x1, 2147483648
    srl x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16777216
    bne x14, x7, failed

srl_38:
    li  gp, 38
    li  x4, 0
1:  li  x2, 14
    nop
    li  x1, 2147483648
    nop
    srl x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 131072
    bne x14, x7, failed

srl_39:
    li  gp, 39
    li  x4, 0
1:  li  x2, 31
    nop    
    nop
    li  x1, 2147483648
    srl x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

srl_40:
    li  gp, 40
    li  x1, 15
    srl x2, x0, x1
    li  x7, 0
    bne x2, x7, failed

srl_41:
    li  gp, 41
    li  x1, 32
    srl x2, x1, x0
    li  x7, 32
    bne x2, x7, failed

srl_42:
    li  gp, 42
    srl x1, x0, x0
    li  x7, 0
    bne x1, x7, failed

srl_43:
    li  gp, 43
    li  x1, 1024
    li  x2, 2048
    srl x0, x1, x2
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
