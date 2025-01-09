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
# ### sraw test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

sraw_2:
    li  gp, 2
    li  x11, 0xffffffff80000000
    li  x12, 0
    sraw x14, x11, x12
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

sraw_3:
    li  gp, 3
    li  x11, 0xffffffff80000000
    li  x12, 1
    sraw x14, x11, x12
    li  x7, 0xffffffffc0000000
    bne x14, x7, failed
    li  x7, 0xffffffffc0000000
    bne x14, x7, failed

sraw_4:
    li  gp, 4
    li  x11, 0xffffffff80000000
    li  x12, 7
    sraw x14, x11, x12
    li  x7, 0xffffffffff000000
    bne x14, x7, failed
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

sraw_5:
    li  gp, 5
    li  x11, 0xffffffff80000000
    li  x12, 14
    sraw x14, x11, x12
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

sraw_6:
    li  gp, 6
    li  x11, 0xffffffff80000001
    li  x12, 31
    sraw x14, x11, x12
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sraw_7:
    li  gp, 7
    li  x11, 0x000000007fffffff
    li  x12, 0
    sraw x14, x11, x12
    li  x7, 0x000000007fffffff
    bne x14, x7, failed
    li  x7, 2147483647
    bne x14, x7, failed

sraw_8:
    li  gp, 8
    li  x11, 0x000000007fffffff
    li  x12, 1
    sraw x14, x11, x12
    li  x7, 0x000000003fffffff
    bne x14, x7, failed
    li  x7, 1073741823
    bne x14, x7, failed

sraw_9:
    li  gp, 9
    li  x11, 0x000000007fffffff
    li  x12, 7
    sraw x14, x11, x12
    li  x7, 0x0000000000ffffff
    bne x14, x7, failed
    li  x7, 16777215
    bne x14, x7, failed

sraw_10:
    li  gp, 10
    li  x11, 0x000000007fffffff
    li  x12, 14
    sraw x14, x11, x12
    li  x7, 0x000000000001ffff
    bne x14, x7, failed
    li  x7, 131071
    bne x14, x7, failed

sraw_11:
    li  gp, 11
    li  x11, 0x000000007fffffff
    li  x12, 31
    sraw x14, x11, x12
    li  x7, 0x0000000000000000
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sraw_12:
    li  gp, 12
    li  x11, 0xffffffff81818181
    li  x12, 0
    sraw x14, x11, x12
    li  x7, 0xffffffff81818181
    bne x14, x7, failed
    li  x7, 0xffffffff81818181
    bne x14, x7, failed

sraw_13:
    li  gp, 13
    li  x11, 0xffffffff81818181
    li  x12, 1
    sraw x14, x11, x12
    li  x7, 0xffffffffc0c0c0c0
    bne x14, x7, failed
    li  x7, 0xffffffffc0c0c0c0
    bne x14, x7, failed

sraw_14:
    li  gp, 14
    li  x11, 0xffffffff81818181
    li  x12, 7
    sraw x14, x11, x12
    li  x7, 0xffffffffff030303
    bne x14, x7, failed
    li  x7, 0xffffffffff030303
    bne x14, x7, failed

sraw_15:
    li  gp, 15
    li  x11, 0xffffffff81818181
    li  x12, 14
    sraw x14, x11, x12
    li  x7, 0xfffffffffffe0606
    bne x14, x7, failed
    li  x7, 0xfffffffffffe0606
    bne x14, x7, failed

sraw_16:
    li  gp, 16
    li  x11, 0xffffffff81818181
    li  x12, 31
    sraw x14, x11, x12
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

# Verify that shifts only use bottom five bits

sraw_17:
    li  gp, 17
    li  x11, 0xffffffff81818181
    li  x12, 0xffffffffffffffe0
    sraw x14, x11, x12
    li  x7, 0xffffffff81818181
    bne x14, x7, failed
    li  x7, 0xffffffff81818181
    bne x14, x7, failed

sraw_18:
    li  gp, 18
    li  x11, 0xffffffff81818181
    li  x12, 0xffffffffffffffe1
    sraw x14, x11, x12
    li  x7, 0xffffffffc0c0c0c0
    bne x14, x7, failed
    li  x7, 0xffffffffc0c0c0c0
    bne x14, x7, failed

sraw_19:
    li  gp, 19
    li  x11, 0xffffffff81818181
    li  x12, 0xffffffffffffffe7
    sraw x14, x11, x12
    li  x7, 0xffffffffff030303
    bne x14, x7, failed
    li  x7, 0xffffffffff030303
    bne x14, x7, failed

sraw_20:
    li  gp, 20
    li  x11, 0xffffffff81818181
    li  x12, 0xffffffffffffffee
    sraw x14, x11, x12
    li  x7, 0xfffffffffffe0606
    bne x14, x7, failed
    li  x7, 0xfffffffffffe0606
    bne x14, x7, failed

sraw_21:
    li  gp, 21
    li  x11, 0xffffffff81818181
    li  x12, 0xffffffffffffffff
    sraw x14, x11, x12
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

# Verify that shifts ignore top 32 (using true 64-bit values)

sraw_44:
    li  gp, 44
    li  x11, 0xffffffff12345678
    li  x12, 0
    sraw x14, x11, x12
    li  x7, 0x0000000012345678
    bne x14, x7, failed
    li  x7, 305419896
    bne x14, x7, failed

sraw_45:
    li  gp, 45
    li  x11, 0xffffffff12345678
    li  x12, 4
    sraw x14, x11, x12
    li  x7, 0x0000000001234567
    bne x14, x7, failed
    li  x7, 19088743
    bne x14, x7, failed

sraw_46:
    li  gp, 46
    li  x11, 0x0000000092345678
    li  x12, 0
    sraw x14, x11, x12
    li  x7, 0xffffffff92345678
    bne x14, x7, failed
    li  x7, 0xffffffff92345678
    bne x14, x7, failed

sraw_47:
    li  gp, 47
    li  x11, 0x0000000092345678
    li  x12, 4
    sraw x14, x11, x12
    li  x7, 0xfffffffff9234567
    bne x14, x7, failed
    li  x7, 0xfffffffff9234567
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

sraw_22:
    li  gp, 22
    li  x11, 0xffffffff80000000
    li  x12, 7
    sraw x11, x11, x12
    li  x7, 0xffffffffff000000
    bne x11, x7, failed
    li  x7, 0xffffffffff000000
    bne x11, x7, failed

sraw_23:
    li  gp, 23
    li  x11, 0xffffffff80000000
    li  x12, 14
    sraw x12, x11, x12
    li  x7, 0xfffffffffffe0000
    bne x12, x7, failed
    li  x7, 0xfffffffffffe0000
    bne x12, x7, failed

sraw_24:
    li  gp, 24
    li  x11, 7
    sraw x11, x11, x11
    li  x7, 0
    bne x11, x7, failed
    li  x7, 0
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

sraw_25:
    li  gp, 25
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 7
    sraw x14, x1, x2
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x6, x7, failed
    li  x7, 0xffffffffff000000
    bne x6, x7, failed

sraw_26:
    li  gp, 26
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 14
    sraw x14, x1, x2
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x6, x7, failed
    li  x7, 0xfffffffffffe0000
    bne x6, x7, failed

sraw_27:
    li  gp, 27
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 31
    sraw x14, x1, x2
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x6, x7, failed
    li  x7, 0xffffffffffffffff
    bne x6, x7, failed

sraw_28:
    li  gp, 28
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 7
    sraw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x14, x7, failed
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

sraw_29:
    li  gp, 29
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 14
    nop
    sraw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

sraw_30:
    li  gp, 30
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 31
    nop    
    nop
    sraw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sraw_31:
    li  gp, 31
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    nop
    li  x2, 7
    sraw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x14, x7, failed
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

sraw_32:
    li  gp, 32
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    nop
    li  x2, 14
    nop
    sraw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

sraw_33:
    li  gp, 33
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    nop    
    nop
    li  x2, 31
    sraw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sraw_34:
    li  gp, 34
    li  x4, 0
1:  li  x2, 7
    li  x1, 0xffffffff80000000
    sraw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

sraw_35:
    li  gp, 35
    li  x4, 0
1:  li  x2, 14
    li  x1, 0xffffffff80000000
    nop
    sraw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

sraw_36:
    li  gp, 36
    li  x4, 0
1:  li  x2, 31
    li  x1, 0xffffffff80000000
    nop    
    nop
    sraw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sraw_37:
    li  gp, 37
    li  x4, 0
1:  li  x2, 7
    nop
    li  x1, 0xffffffff80000000
    sraw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

sraw_38:
    li  gp, 38
    li  x4, 0
1:  li  x2, 14
    nop
    li  x1, 0xffffffff80000000
    nop
    sraw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

sraw_39:
    li  gp, 39
    li  x4, 0
1:  li  x2, 31
    nop    
    nop
    li  x1, 0xffffffff80000000
    sraw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sraw_40:
    li  gp, 40
    li  x1, 15
    sraw x2, x0, x1
    li  x7, 0
    bne x2, x7, failed

sraw_41:
    li  gp, 41
    li  x1, 32
    sraw x2, x1, x0
    li  x7, 32
    bne x2, x7, failed

sraw_42:
    li  gp, 42
    sraw x1, x0, x0
    li  x7, 0
    bne x1, x7, failed

sraw_43:
    li  gp, 43
    li  x1, 1024
    li  x2, 2048
    sraw x0, x1, x2
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
