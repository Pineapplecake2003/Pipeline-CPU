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
# ### sllw test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

sllw_2:
    li  gp, 2
    li  x11, 0x0000000000000001
    li  x12, 0
    sllw x14, x11, x12
    li  x7, 0x0000000000000001
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

sllw_3:
    li  gp, 3
    li  x11, 0x0000000000000001
    li  x12, 1
    sllw x14, x11, x12
    li  x7, 0x0000000000000002
    bne x14, x7, failed
    li  x7, 2
    bne x14, x7, failed

sllw_4:
    li  gp, 4
    li  x11, 0x0000000000000001
    li  x12, 7
    sllw x14, x11, x12
    li  x7, 0x0000000000000080
    bne x14, x7, failed
    li  x7, 128
    bne x14, x7, failed

sllw_5:
    li  gp, 5
    li  x11, 0x0000000000000001
    li  x12, 14
    sllw x14, x11, x12
    li  x7, 0x0000000000004000
    bne x14, x7, failed
    li  x7, 16384
    bne x14, x7, failed

sllw_6:
    li  gp, 6
    li  x11, 0x0000000000000001
    li  x12, 31
    sllw x14, x11, x12
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

sllw_7:
    li  gp, 7
    li  x11, 0xffffffffffffffff
    li  x12, 0
    sllw x14, x11, x12
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sllw_8:
    li  gp, 8
    li  x11, 0xffffffffffffffff
    li  x12, 1
    sllw x14, x11, x12
    li  x7, 0xfffffffffffffffe
    bne x14, x7, failed
    li  x7, 0xfffffffffffffffe
    bne x14, x7, failed

sllw_9:
    li  gp, 9
    li  x11, 0xffffffffffffffff
    li  x12, 7
    sllw x14, x11, x12
    li  x7, 0xffffffffffffff80
    bne x14, x7, failed
    li  x7, 0xffffffffffffff80
    bne x14, x7, failed

sllw_10:
    li  gp, 10
    li  x11, 0xffffffffffffffff
    li  x12, 14
    sllw x14, x11, x12
    li  x7, 0xffffffffffffc000
    bne x14, x7, failed
    li  x7, 0xffffffffffffc000
    bne x14, x7, failed

sllw_11:
    li  gp, 11
    li  x11, 0xffffffffffffffff
    li  x12, 31
    sllw x14, x11, x12
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

sllw_12:
    li  gp, 12
    li  x11, 0x0000000021212121
    li  x12, 0
    sllw x14, x11, x12
    li  x7, 0x0000000021212121
    bne x14, x7, failed
    li  x7, 555819297
    bne x14, x7, failed

sllw_13:
    li  gp, 13
    li  x11, 0x0000000021212121
    li  x12, 1
    sllw x14, x11, x12
    li  x7, 0x0000000042424242
    bne x14, x7, failed
    li  x7, 1111638594
    bne x14, x7, failed

sllw_14:
    li  gp, 14
    li  x11, 0x0000000021212121
    li  x12, 7
    sllw x14, x11, x12
    li  x7, 0xffffffff90909080
    bne x14, x7, failed
    li  x7, 0xffffffff90909080
    bne x14, x7, failed

sllw_15:
    li  gp, 15
    li  x11, 0x0000000021212121
    li  x12, 14
    sllw x14, x11, x12
    li  x7, 0x0000000048484000
    bne x14, x7, failed
    li  x7, 1212694528
    bne x14, x7, failed

sllw_16:
    li  gp, 16
    li  x11, 0x0000000021212121
    li  x12, 31
    sllw x14, x11, x12
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

# Verify that shifts only use bottom five bits

sllw_17:
    li  gp, 17
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffe0
    sllw x14, x11, x12
    li  x7, 0x0000000021212121
    bne x14, x7, failed
    li  x7, 555819297
    bne x14, x7, failed

sllw_18:
    li  gp, 18
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffe1
    sllw x14, x11, x12
    li  x7, 0x0000000042424242
    bne x14, x7, failed
    li  x7, 1111638594
    bne x14, x7, failed

sllw_19:
    li  gp, 19
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffe7
    sllw x14, x11, x12
    li  x7, 0xffffffff90909080
    bne x14, x7, failed
    li  x7, 0xffffffff90909080
    bne x14, x7, failed

sllw_20:
    li  gp, 20
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffee
    sllw x14, x11, x12
    li  x7, 0x0000000048484000
    bne x14, x7, failed
    li  x7, 1212694528
    bne x14, x7, failed

sllw_21:
    li  gp, 21
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffff
    sllw x14, x11, x12
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

# Verify that shifts ignore top 32 (using true 64-bit values)

sllw_44:
    li  gp, 44
    li  x11, 0xffffffff12345678
    li  x12, 0
    sllw x14, x11, x12
    li  x7, 0x0000000012345678
    bne x14, x7, failed
    li  x7, 305419896
    bne x14, x7, failed

sllw_45:
    li  gp, 45
    li  x11, 0xffffffff12345678
    li  x12, 4
    sllw x14, x11, x12
    li  x7, 0x0000000023456780
    bne x14, x7, failed
    li  x7, 591751040
    bne x14, x7, failed

sllw_46:
    li  gp, 46
    li  x11, 0x0000000092345678
    li  x12, 0
    sllw x14, x11, x12
    li  x7, 0xffffffff92345678
    bne x14, x7, failed
    li  x7, 0xffffffff92345678
    bne x14, x7, failed

sllw_47:
    li  gp, 47
    li  x11, 0x0000000099345678
    li  x12, 4
    sllw x14, x11, x12
    li  x7, 0xffffffff93456780
    bne x14, x7, failed
    li  x7, 0xffffffff93456780
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

sllw_22:
    li  gp, 22
    li  x11, 0x00000001
    li  x12, 7
    sllw x11, x11, x12
    li  x7, 0x00000080
    bne x11, x7, failed
    li  x7, 128
    bne x11, x7, failed

sllw_23:
    li  gp, 23
    li  x11, 0x00000001
    li  x12, 14
    sllw x12, x11, x12
    li  x7, 0x00004000
    bne x12, x7, failed
    li  x7, 16384
    bne x12, x7, failed

sllw_24:
    li  gp, 24
    li  x11, 3
    sllw x11, x11, x11
    li  x7, 24
    bne x11, x7, failed
    li  x7, 24
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

sllw_25:
    li  gp, 25
    li  x4, 0
1:  li  x1, 0x0000000000000001
    li  x2, 7
    sllw x14, x1, x2
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000000080
    bne x6, x7, failed
    li  x7, 128
    bne x6, x7, failed

sllw_26:
    li  gp, 26
    li  x4, 0
1:  li  x1, 0x0000000000000001
    li  x2, 14
    sllw x14, x1, x2
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000004000
    bne x6, x7, failed
    li  x7, 16384
    bne x6, x7, failed

sllw_27:
    li  gp, 27
    li  x4, 0
1:  li  x1, 0x0000000000000001
    li  x2, 31
    sllw x14, x1, x2
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffff80000000
    bne x6, x7, failed
    li  x7, 0xffffffff80000000
    bne x6, x7, failed

sllw_28:
    li  gp, 28
    li  x4, 0
1:  li  x1, 0x0000000000000001
    li  x2, 7
    sllw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000000080
    bne x14, x7, failed
    li  x7, 128
    bne x14, x7, failed

sllw_29:
    li  gp, 29
    li  x4, 0
1:  li  x1, 0x0000000000000001
    li  x2, 14
    nop
    sllw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000004000
    bne x14, x7, failed
    li  x7, 16384
    bne x14, x7, failed

sllw_30:
    li  gp, 30
    li  x4, 0
1:  li  x1, 0x0000000000000001
    li  x2, 31
    nop    
    nop
    sllw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

sllw_31:
    li  gp, 31
    li  x4, 0
1:  li  x1, 0x0000000000000001
    nop
    li  x2, 7
    sllw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000000080
    bne x14, x7, failed
    li  x7, 128
    bne x14, x7, failed

sllw_32:
    li  gp, 32
    li  x4, 0
1:  li  x1, 0x0000000000000001
    nop
    li  x2, 14
    nop
    sllw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000004000
    bne x14, x7, failed
    li  x7, 16384
    bne x14, x7, failed

sllw_33:
    li  gp, 33
    li  x4, 0
1:  li  x1, 0x0000000000000001
    nop    
    nop
    li  x2, 31
    sllw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

sllw_34:
    li  gp, 34
    li  x4, 0
1:  li  x2, 7
    li  x1, 1
    sllw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 128
    bne x14, x7, failed

sllw_35:
    li  gp, 35
    li  x4, 0
1:  li  x2, 14
    li  x1, 1
    nop
    sllw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16384
    bne x14, x7, failed

sllw_36:
    li  gp, 36
    li  x4, 0
1:  li  x2, 31
    li  x1, 1
    nop    
    nop
    sllw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

sllw_37:
    li  gp, 37
    li  x4, 0
1:  li  x2, 7
    nop
    li  x1, 1
    sllw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 128
    bne x14, x7, failed

sllw_38:
    li  gp, 38
    li  x4, 0
1:  li  x2, 14
    nop
    li  x1, 1
    nop
    sllw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16384
    bne x14, x7, failed

sllw_39:
    li  gp, 39
    li  x4, 0
1:  li  x2, 31
    nop    
    nop
    li  x1, 1
    sllw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

sllw_40:
    li  gp, 40
    li  x1, 15
    sllw x2, x0, x1
    li  x7, 0
    bne x2, x7, failed

sllw_41:
    li  gp, 41
    li  x1, 32
    sllw x2, x1, x0
    li  x7, 32
    bne x2, x7, failed

sllw_42:
    li  gp, 42
    sllw x1, x0, x0
    li  x7, 0
    bne x1, x7, failed

sllw_43:
    li  gp, 43
    li  x1, 1024
    li  x2, 2048
    sllw x0, x1, x2
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
