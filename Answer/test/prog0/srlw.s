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
# ### srlw test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

srlw_2:
    li  gp, 2
    li  x11, 0xffffffff80000000
    li  x12, 0
    srlw x14, x11, x12
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

srlw_3:
    li  gp, 3
    li  x11, 0xffffffff80000000
    li  x12, 1
    srlw x14, x11, x12
    li  x7, 0x0000000040000000
    bne x14, x7, failed
    li  x7, 1073741824
    bne x14, x7, failed

srlw_4:
    li  gp, 4
    li  x11, 0xffffffff80000000
    li  x12, 7
    srlw x14, x11, x12
    li  x7, 0x0000000001000000
    bne x14, x7, failed
    li  x7, 16777216
    bne x14, x7, failed

srlw_5:
    li  gp, 5
    li  x11, 0xffffffff80000000
    li  x12, 14
    srlw x14, x11, x12
    li  x7, 0x0000000000020000
    bne x14, x7, failed
    li  x7, 131072
    bne x14, x7, failed

srlw_6:
    li  gp, 6
    li  x11, 0xffffffff80000001
    li  x12, 31
    srlw x14, x11, x12
    li  x7, 0x0000000000000001
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

srlw_7:
    li  gp, 7
    li  x11, 0xffffffffffffffff
    li  x12, 0
    srlw x14, x11, x12
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

srlw_8:
    li  gp, 8
    li  x11, 0xffffffffffffffff
    li  x12, 1
    srlw x14, x11, x12
    li  x7, 0x000000007fffffff
    bne x14, x7, failed
    li  x7, 2147483647
    bne x14, x7, failed

srlw_9:
    li  gp, 9
    li  x11, 0xffffffffffffffff
    li  x12, 7
    srlw x14, x11, x12
    li  x7, 0x0000000001ffffff
    bne x14, x7, failed
    li  x7, 33554431
    bne x14, x7, failed

srlw_10:
    li  gp, 10
    li  x11, 0xffffffffffffffff
    li  x12, 14
    srlw x14, x11, x12
    li  x7, 0x000000000003ffff
    bne x14, x7, failed
    li  x7, 262143
    bne x14, x7, failed

srlw_11:
    li  gp, 11
    li  x11, 0xffffffffffffffff
    li  x12, 31
    srlw x14, x11, x12
    li  x7, 0x0000000000000001
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

srlw_12:
    li  gp, 12
    li  x11, 0x0000000021212121
    li  x12, 0
    srlw x14, x11, x12
    li  x7, 0x0000000021212121
    bne x14, x7, failed
    li  x7, 555819297
    bne x14, x7, failed

srlw_13:
    li  gp, 13
    li  x11, 0x0000000021212121
    li  x12, 1
    srlw x14, x11, x12
    li  x7, 0x0000000010909090
    bne x14, x7, failed
    li  x7, 277909648
    bne x14, x7, failed

srlw_14:
    li  gp, 14
    li  x11, 0x0000000021212121
    li  x12, 7
    srlw x14, x11, x12
    li  x7, 0x0000000000424242
    bne x14, x7, failed
    li  x7, 4342338
    bne x14, x7, failed

srlw_15:
    li  gp, 15
    li  x11, 0x0000000021212121
    li  x12, 14
    srlw x14, x11, x12
    li  x7, 0x0000000000008484
    bne x14, x7, failed
    li  x7, 33924
    bne x14, x7, failed

srlw_16:
    li  gp, 16
    li  x11, 0x0000000021212121
    li  x12, 31
    srlw x14, x11, x12
    li  x7, 0x0000000000000000
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

# Verify that shifts only use bottom five bits

srlw_17:
    li  gp, 17
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffe0
    srlw x14, x11, x12
    li  x7, 0x0000000021212121
    bne x14, x7, failed
    li  x7, 555819297
    bne x14, x7, failed

srlw_18:
    li  gp, 18
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffe1
    srlw x14, x11, x12
    li  x7, 0x0000000010909090
    bne x14, x7, failed
    li  x7, 277909648
    bne x14, x7, failed

srlw_19:
    li  gp, 19
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffe7
    srlw x14, x11, x12
    li  x7, 0x0000000000424242
    bne x14, x7, failed
    li  x7, 4342338
    bne x14, x7, failed

srlw_20:
    li  gp, 20
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffee
    srlw x14, x11, x12
    li  x7, 0x0000000000008484
    bne x14, x7, failed
    li  x7, 33924
    bne x14, x7, failed

srlw_21:
    li  gp, 21
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffff
    srlw x14, x11, x12
    li  x7, 0x0000000000000000
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

# Verify that shifts ignore top 32 (using true 64-bit values)

srlw_44:
    li  gp, 44
    li  x11, 0xffffffff12345678
    li  x12, 0
    srlw x14, x11, x12
    li  x7, 0x0000000012345678
    bne x14, x7, failed
    li  x7, 305419896
    bne x14, x7, failed

srlw_45:
    li  gp, 45
    li  x11, 0xffffffff12345678
    li  x12, 4
    srlw x14, x11, x12
    li  x7, 0x0000000001234567
    bne x14, x7, failed
    li  x7, 19088743
    bne x14, x7, failed

srlw_46:
    li  gp, 46
    li  x11, 0x0000000092345678
    li  x12, 0
    srlw x14, x11, x12
    li  x7, 0xffffffff92345678
    bne x14, x7, failed
    li  x7, 0xffffffff92345678
    bne x14, x7, failed

srlw_47:
    li  gp, 47
    li  x11, 0x0000000092345678
    li  x12, 4
    srlw x14, x11, x12
    li  x7, 0x0000000009234567
    bne x14, x7, failed
    li  x7, 153306471
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

srlw_22:
    li  gp, 22
    li  x11, 0xffffffff80000000
    li  x12, 7
    srlw x11, x11, x12
    li  x7, 0x0000000001000000
    bne x11, x7, failed
    li  x7, 16777216
    bne x11, x7, failed

srlw_23:
    li  gp, 23
    li  x11, 0xffffffff80000000
    li  x12, 14
    srlw x12, x11, x12
    li  x7, 0x0000000000020000
    bne x12, x7, failed
    li  x7, 131072
    bne x12, x7, failed

srlw_24:
    li  gp, 24
    li  x11, 7
    srlw x11, x11, x11
    li  x7, 0
    bne x11, x7, failed
    li  x7, 0
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

srlw_25:
    li  gp, 25
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 7
    srlw x14, x1, x2
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000001000000
    bne x6, x7, failed
    li  x7, 16777216
    bne x6, x7, failed

srlw_26:
    li  gp, 26
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 14
    srlw x14, x1, x2
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000020000
    bne x6, x7, failed
    li  x7, 131072
    bne x6, x7, failed

srlw_27:
    li  gp, 27
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 31
    srlw x14, x1, x2
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000000001
    bne x6, x7, failed
    li  x7, 1
    bne x6, x7, failed

srlw_28:
    li  gp, 28
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 7
    srlw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000001000000
    bne x14, x7, failed
    li  x7, 16777216
    bne x14, x7, failed

srlw_29:
    li  gp, 29
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 14
    nop
    srlw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000020000
    bne x14, x7, failed
    li  x7, 131072
    bne x14, x7, failed

srlw_30:
    li  gp, 30
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 31
    nop    
    nop
    srlw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000000001
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

srlw_31:
    li  gp, 31
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    nop
    li  x2, 7
    srlw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000001000000
    bne x14, x7, failed
    li  x7, 16777216
    bne x14, x7, failed

srlw_32:
    li  gp, 32
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    nop
    li  x2, 14
    nop
    srlw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000020000
    bne x14, x7, failed
    li  x7, 131072
    bne x14, x7, failed

srlw_33:
    li  gp, 33
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    nop    
    nop
    li  x2, 31
    srlw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000000001
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

srlw_34:
    li  gp, 34
    li  x4, 0
1:  li  x2, 7
    li  x1, 0xffffffff80000000
    srlw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16777216
    bne x14, x7, failed

srlw_35:
    li  gp, 35
    li  x4, 0
1:  li  x2, 14
    li  x1, 0xffffffff80000000
    nop
    srlw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 131072
    bne x14, x7, failed

srlw_36:
    li  gp, 36
    li  x4, 0
1:  li  x2, 31
    li  x1, 0xffffffff80000000
    nop    
    nop
    srlw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

srlw_37:
    li  gp, 37
    li  x4, 0
1:  li  x2, 7
    nop
    li  x1, 0xffffffff80000000
    srlw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16777216
    bne x14, x7, failed

srlw_38:
    li  gp, 38
    li  x4, 0
1:  li  x2, 14
    nop
    li  x1, 0xffffffff80000000
    nop
    srlw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 131072
    bne x14, x7, failed

srlw_39:
    li  gp, 39
    li  x4, 0
1:  li  x2, 31
    nop    
    nop
    li  x1, 0xffffffff80000000
    srlw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

srlw_40:
    li  gp, 40
    li  x1, 15
    srlw x2, x0, x1
    li  x7, 0
    bne x2, x7, failed

srlw_41:
    li  gp, 41
    li  x1, 32
    srlw x2, x1, x0
    li  x7, 32
    bne x2, x7, failed

srlw_42:
    li  gp, 42
    srlw x1, x0, x0
    li  x7, 0
    bne x1, x7, failed

srlw_43:
    li  gp, 43
    li  x1, 1024
    li  x2, 2048
    srlw x0, x1, x2
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
