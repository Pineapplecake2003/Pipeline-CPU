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
# ### sra test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

sra_2:
    li  gp, 2
    li  x11, 0xffffffff80000000
    li  x12, 0
    sra x14, x11, x12
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

sra_3:
    li  gp, 3
    li  x11, 0xffffffff80000000
    li  x12, 1
    sra x14, x11, x12
    li  x7, 0xffffffffc0000000
    bne x14, x7, failed
    li  x7, 0xffffffffc0000000
    bne x14, x7, failed

sra_4:
    li  gp, 4
    li  x11, 0xffffffff80000000
    li  x12, 7
    sra x14, x11, x12
    li  x7, 0xffffffffff000000
    bne x14, x7, failed
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

sra_5:
    li  gp, 5
    li  x11, 0xffffffff80000000
    li  x12, 14
    sra x14, x11, x12
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

sra_6:
    li  gp, 6
    li  x11, 0xffffffff80000001
    li  x12, 31
    sra x14, x11, x12
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sra_7:
    li  gp, 7
    li  x11, 0x000000007fffffff
    li  x12, 0
    sra x14, x11, x12
    li  x7, 0x000000007fffffff
    bne x14, x7, failed
    li  x7, 2147483647
    bne x14, x7, failed

sra_8:
    li  gp, 8
    li  x11, 0x000000007fffffff
    li  x12, 1
    sra x14, x11, x12
    li  x7, 0x000000003fffffff
    bne x14, x7, failed
    li  x7, 1073741823
    bne x14, x7, failed

sra_9:
    li  gp, 9
    li  x11, 0x000000007fffffff
    li  x12, 7
    sra x14, x11, x12
    li  x7, 0x0000000000ffffff
    bne x14, x7, failed
    li  x7, 16777215
    bne x14, x7, failed

sra_10:
    li  gp, 10
    li  x11, 0x000000007fffffff
    li  x12, 14
    sra x14, x11, x12
    li  x7, 0x000000000001ffff
    bne x14, x7, failed
    li  x7, 131071
    bne x14, x7, failed

sra_11:
    li  gp, 11
    li  x11, 0x000000007fffffff
    li  x12, 31
    sra x14, x11, x12
    li  x7, 0x0000000000000000
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sra_12:
    li  gp, 12
    li  x11, 0xffffffff81818181
    li  x12, 0
    sra x14, x11, x12
    li  x7, 0xffffffff81818181
    bne x14, x7, failed
    li  x7, 0xffffffff81818181
    bne x14, x7, failed

sra_13:
    li  gp, 13
    li  x11, 0xffffffff81818181
    li  x12, 1
    sra x14, x11, x12
    li  x7, 0xffffffffc0c0c0c0
    bne x14, x7, failed
    li  x7, 0xffffffffc0c0c0c0
    bne x14, x7, failed

sra_14:
    li  gp, 14
    li  x11, 0xffffffff81818181
    li  x12, 7
    sra x14, x11, x12
    li  x7, 0xffffffffff030303
    bne x14, x7, failed
    li  x7, 0xffffffffff030303
    bne x14, x7, failed

sra_15:
    li  gp, 15
    li  x11, 0xffffffff81818181
    li  x12, 14
    sra x14, x11, x12
    li  x7, 0xfffffffffffe0606
    bne x14, x7, failed
    li  x7, 0xfffffffffffe0606
    bne x14, x7, failed

sra_16:
    li  gp, 16
    li  x11, 0xffffffff81818181
    li  x12, 31
    sra x14, x11, x12
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

# Verify that shifts only use bottom six(rv64) or five(rv32) bits

sra_17:
    li  gp, 17
    li  x11, 0xffffffff81818181
    li  x12, 0xffffffffffffffc0
    sra x14, x11, x12
    li  x7, 0xffffffff81818181
    bne x14, x7, failed
    li  x7, 0xffffffff81818181
    bne x14, x7, failed

sra_18:
    li  gp, 18
    li  x11, 0xffffffff81818181
    li  x12, 0xffffffffffffffc1
    sra x14, x11, x12
    li  x7, 0xffffffffc0c0c0c0
    bne x14, x7, failed
    li  x7, 0xffffffffc0c0c0c0
    bne x14, x7, failed

sra_19:
    li  gp, 19
    li  x11, 0xffffffff81818181
    li  x12, 0xffffffffffffffc7
    sra x14, x11, x12
    li  x7, 0xffffffffff030303
    bne x14, x7, failed
    li  x7, 0xffffffffff030303
    bne x14, x7, failed

sra_20:
    li  gp, 20
    li  x11, 0xffffffff81818181
    li  x12, 0xffffffffffffffce
    sra x14, x11, x12
    li  x7, 0xfffffffffffe0606
    bne x14, x7, failed
    li  x7, 0xfffffffffffe0606
    bne x14, x7, failed

sra_21:
    li  gp, 21
    li  x11, 0xffffffff81818181
    li  x12, 0xffffffffffffffff
    sra x14, x11, x12
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

sra_22:
    li  gp, 22
    li  x11, 0xffffffff80000000
    li  x12, 7
    sra x11, x11, x12
    li  x7, 0xffffffffff000000
    bne x11, x7, failed
    li  x7, 0xffffffffff000000
    bne x11, x7, failed

sra_23:
    li  gp, 23
    li  x11, 0xffffffff80000000
    li  x12, 14
    sra x12, x11, x12
    li  x7, 0xfffffffffffe0000
    bne x12, x7, failed
    li  x7, 0xfffffffffffe0000
    bne x12, x7, failed

sra_24:
    li  gp, 24
    li  x11, 7
    sra x11, x11, x11
    li  x7, 0
    bne x11, x7, failed
    li  x7, 0
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

sra_25:
    li  gp, 25
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 7
    sra x14, x1, x2
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x6, x7, failed
    li  x7, 0xffffffffff000000
    bne x6, x7, failed

sra_26:
    li  gp, 26
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 14
    sra x14, x1, x2
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x6, x7, failed
    li  x7, 0xfffffffffffe0000
    bne x6, x7, failed

sra_27:
    li  gp, 27
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 31
    sra x14, x1, x2
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

sra_28:
    li  gp, 28
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 7
    sra x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x14, x7, failed
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

sra_29:
    li  gp, 29
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 14
    nop
    sra x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

sra_30:
    li  gp, 30
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    li  x2, 31
    nop    
    nop
    sra x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sra_31:
    li  gp, 31
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    nop
    li  x2, 7
    sra x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x14, x7, failed
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

sra_32:
    li  gp, 32
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    nop
    li  x2, 14
    nop
    sra x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

sra_33:
    li  gp, 33
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    nop    
    nop
    li  x2, 31
    sra x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sra_34:
    li  gp, 34
    li  x4, 0
1:  li  x2, 7
    li  x1, 0xffffffff80000000
    sra x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

sra_35:
    li  gp, 35
    li  x4, 0
1:  li  x2, 14
    li  x1, 0xffffffff80000000
    nop
    sra x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

sra_36:
    li  gp, 36
    li  x4, 0
1:  li  x2, 31
    li  x1, 0xffffffff80000000
    nop    
    nop
    sra x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sra_37:
    li  gp, 37
    li  x4, 0
1:  li  x2, 7
    nop
    li  x1, 0xffffffff80000000
    sra x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

sra_38:
    li  gp, 38
    li  x4, 0
1:  li  x2, 14
    nop
    li  x1, 0xffffffff80000000
    nop
    sra x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

sra_39:
    li  gp, 39
    li  x4, 0
1:  li  x2, 31
    nop    
    nop
    li  x1, 0xffffffff80000000
    sra x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

sra_40:
    li  gp, 40
    li  x1, 15
    sra x2, x0, x1
    li  x7, 0
    bne x2, x7, failed

sra_41:
    li  gp, 41
    li  x1, 32
    sra x2, x1, x0
    li  x7, 32
    bne x2, x7, failed

sra_42:
    li  gp, 42
    sra x1, x0, x0
    li  x7, 0
    bne x1, x7, failed

sra_43:
    li  gp, 43
    li  x1, 1024
    li  x2, 2048
    sra x0, x1, x2
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
