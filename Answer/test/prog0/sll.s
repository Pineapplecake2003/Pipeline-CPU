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
# ### sll test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

sll_2:
    li  gp, 2
    li  x11, 0x0000000000000001
    li  x12, 0
    sll x14, x11, x12
    li  x7, 0x0000000000000001
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

sll_3:
    li  gp, 3
    li  x11, 0x0000000000000001
    li  x12, 1
    sll x14, x11, x12
    li  x7, 0x0000000000000002
    bne x14, x7, failed
    li  x7, 2
    bne x14, x7, failed

sll_4:
    li  gp, 4
    li  x11, 0x0000000000000001
    li  x12, 7
    sll x14, x11, x12
    li  x7, 0x0000000000000080
    bne x14, x7, failed
    li  x7, 128
    bne x14, x7, failed

sll_5:
    li  gp, 5
    li  x11, 0x0000000000000001
    li  x12, 14
    sll x14, x11, x12
    li  x7, 0x0000000000004000
    bne x14, x7, failed
    li  x7, 16384
    bne x14, x7, failed

sll_6:
    li  gp, 6
    li  x11, 0x0000000000000001
    li  x12, 31
    sll x14, x11, x12
    li  x7, 0x0000000080000000
    bne x14, x7, failed
    li  x7, 2147483648
    bne x14, x7, failed

sll_7:
    li  gp, 7
    li  x11, 0xffffffffffffffff
    li  x12, 0
    sll x14, x11, x12
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sll_8:
    li  gp, 8
    li  x11, 0xffffffffffffffff
    li  x12, 1
    sll x14, x11, x12
    li  x7, 0xfffffffffffffffe
    bne x14, x7, failed
    li  x7, 0xfffffffffffffffe
    bne x14, x7, failed

sll_9:
    li  gp, 9
    li  x11, 0xffffffffffffffff
    li  x12, 7
    sll x14, x11, x12
    li  x7, 0xffffffffffffff80
    bne x14, x7, failed
    li  x7, 0xffffffffffffff80
    bne x14, x7, failed

sll_10:
    li  gp, 10
    li  x11, 0xffffffffffffffff
    li  x12, 14
    sll x14, x11, x12
    li  x7, 0xffffffffffffc000
    bne x14, x7, failed
    li  x7, 0xffffffffffffc000
    bne x14, x7, failed

sll_11:
    li  gp, 11
    li  x11, 0xffffffffffffffff
    li  x12, 31
    sll x14, x11, x12
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

sll_12:
    li  gp, 12
    li  x11, 0x0000000021212121
    li  x12, 0
    sll x14, x11, x12
    li  x7, 0x0000000021212121
    bne x14, x7, failed
    li  x7, 555819297
    bne x14, x7, failed

sll_13:
    li  gp, 13
    li  x11, 0x0000000021212121
    li  x12, 1
    sll x14, x11, x12
    li  x7, 0x0000000042424242
    bne x14, x7, failed
    li  x7, 1111638594
    bne x14, x7, failed

sll_14:
    li  gp, 14
    li  x11, 0x0000000021212121
    li  x12, 7
    sll x14, x11, x12
    li  x7, 0x0000001090909080
    bne x14, x7, failed
    li  x7, 71144870016
    bne x14, x7, failed

sll_15:
    li  gp, 15
    li  x11, 0x0000000021212121
    li  x12, 14
    sll x14, x11, x12
    li  x7, 0x0000084848484000
    bne x14, x7, failed
    li  x7, 9106543362048
    bne x14, x7, failed

sll_16:
    li  gp, 16
    li  x11, 0x0000000021212121
    li  x12, 31
    sll x14, x11, x12
    li  x7, 0x1090909080000000
    bne x14, x7, failed
    li  x7, 1193612851550355456
    bne x14, x7, failed

# Verify that shifts only use bottom six(rv64) or five(rv32) bits

sll_17:
    li  gp, 17
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffc0
    sll x14, x11, x12
    li  x7, 0x0000000021212121
    bne x14, x7, failed
    li  x7, 555819297
    bne x14, x7, failed

sll_18:
    li  gp, 18
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffc1
    sll x14, x11, x12
    li  x7, 0x0000000042424242
    bne x14, x7, failed
    li  x7, 1111638594
    bne x14, x7, failed

sll_19:
    li  gp, 19
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffc7
    sll x14, x11, x12
    li  x7, 0x0000001090909080
    bne x14, x7, failed
    li  x7, 71144870016
    bne x14, x7, failed

sll_20:
    li  gp, 20
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffce
    sll x14, x11, x12
    li  x7, 0x0000084848484000
    bne x14, x7, failed
    li  x7, 9106543362048
    bne x14, x7, failed

sll_21:
    li  gp, 21
    li  x11, 0x0000000021212121
    li  x12, 0xffffffffffffffff
    sll x14, x11, x12
    li  x7, 0x8000000000000000
    bne x14, x7, failed
    li  x7, 0x8000000000000000
    bne x14, x7, failed

sll_50:
    li  gp, 50
    li  x11, 0x0000000000000001
    li  x12, 63
    sll x14, x11, x12
    li  x7, 0x8000000000000000
    bne x14, x7, failed
    li  x7, 0x8000000000000000
    bne x14, x7, failed

sll_51:
    li  gp, 51
    li  x11, 0xffffffffffffffff
    li  x12, 39
    sll x14, x11, x12
    li  x7, 0xffffff8000000000
    bne x14, x7, failed
    li  x7, 0xffffff8000000000
    bne x14, x7, failed

sll_52:
    li  gp, 52
    li  x11, 0x0000000021212121
    li  x12, 43
    sll x14, x11, x12
    li  x7, 0x0909080000000000
    bne x14, x7, failed
    li  x7, 651060417224769536
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

sll_22:
    li  gp, 22
    li  x11, 0x00000001
    li  x12, 7
    sll x11, x11, x12
    li  x7, 0x00000080
    bne x11, x7, failed
    li  x7, 128
    bne x11, x7, failed

sll_23:
    li  gp, 23
    li  x11, 0x00000001
    li  x12, 14
    sll x12, x11, x12
    li  x7, 0x00004000
    bne x12, x7, failed
    li  x7, 16384
    bne x12, x7, failed

sll_24:
    li  gp, 24
    li  x11, 3
    sll x11, x11, x11
    li  x7, 24
    bne x11, x7, failed
    li  x7, 24
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

sll_25:
    li  gp, 25
    li  x4, 0
1:  li  x1, 0x0000000000000001
    li  x2, 7
    sll x14, x1, x2
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000000080
    bne x6, x7, failed
    li  x7, 128
    bne x6, x7, failed

sll_26:
    li  gp, 26
    li  x4, 0
1:  li  x1, 0x0000000000000001
    li  x2, 14
    sll x14, x1, x2
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000004000
    bne x6, x7, failed
    li  x7, 16384
    bne x6, x7, failed

sll_27:
    li  gp, 27
    li  x4, 0
1:  li  x1, 0x0000000000000001
    li  x2, 31
    sll x14, x1, x2
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000080000000
    bne x6, x7, failed
    li  x7, 2147483648
    bne x6, x7, failed

sll_28:
    li  gp, 28
    li  x4, 0
1:  li  x1, 0x0000000000000001
    li  x2, 7
    sll x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000000080
    bne x14, x7, failed
    li  x7, 128
    bne x14, x7, failed

sll_29:
    li  gp, 29
    li  x4, 0
1:  li  x1, 0x0000000000000001
    li  x2, 14
    nop
    sll x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000004000
    bne x14, x7, failed
    li  x7, 16384
    bne x14, x7, failed

sll_30:
    li  gp, 30
    li  x4, 0
1:  li  x1, 0x0000000000000001
    li  x2, 31
    nop    
    nop
    sll x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000080000000
    bne x14, x7, failed
    li  x7, 2147483648
    bne x14, x7, failed

sll_31:
    li  gp, 31
    li  x4, 0
1:  li  x1, 0x0000000000000001
    nop
    li  x2, 7
    sll x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000000080
    bne x14, x7, failed
    li  x7, 128
    bne x14, x7, failed

sll_32:
    li  gp, 32
    li  x4, 0
1:  li  x1, 0x0000000000000001
    nop
    li  x2, 14
    nop
    sll x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000000004000
    bne x14, x7, failed
    li  x7, 16384
    bne x14, x7, failed

sll_33:
    li  gp, 33
    li  x4, 0
1:  li  x1, 0x0000000000000001
    nop    
    nop
    li  x2, 31
    sll x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0000000080000000
    bne x14, x7, failed
    li  x7, 2147483648
    bne x14, x7, failed

sll_34:
    li  gp, 34
    li  x4, 0
1:  li  x2, 7
    li  x1, 1
    sll x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 128
    bne x14, x7, failed

sll_35:
    li  gp, 35
    li  x4, 0
1:  li  x2, 14
    li  x1, 1
    nop
    sll x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16384
    bne x14, x7, failed

sll_36:
    li  gp, 36
    li  x4, 0
1:  li  x2, 31
    li  x1, 1
    nop    
    nop
    sll x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 2147483648
    bne x14, x7, failed

sll_37:
    li  gp, 37
    li  x4, 0
1:  li  x2, 7
    nop
    li  x1, 1
    sll x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 128
    bne x14, x7, failed

sll_38:
    li  gp, 38
    li  x4, 0
1:  li  x2, 14
    nop
    li  x1, 1
    nop
    sll x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16384
    bne x14, x7, failed

sll_39:
    li  gp, 39
    li  x4, 0
1:  li  x2, 31
    nop    
    nop
    li  x1, 1
    sll x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 2147483648
    bne x14, x7, failed

sll_40:
    li  gp, 40
    li  x1, 15
    sll x2, x0, x1
    li  x7, 0
    bne x2, x7, failed

sll_41:
    li  gp, 41
    li  x1, 32
    sll x2, x1, x0
    li  x7, 32
    bne x2, x7, failed

sll_42:
    li  gp, 42
    sll x1, x0, x0
    li  x7, 0
    bne x1, x7, failed

sll_43:
    li  gp, 43
    li  x1, 1024
    li  x2, 2048
    sll x0, x1, x2
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
