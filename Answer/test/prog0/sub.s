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
# ### sub test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

sub_2:
    li  gp, 2
    li  x11, 0x0000000000000000
    li  x12, 0x0000000000000000
    sub x14, x11, x12
    li  x7, 0x0000000000000000
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sub_3:
    li  gp, 3
    li  x11, 0x0000000000000001
    li  x12, 0x0000000000000001
    sub x14, x11, x12
    li  x7, 0x0000000000000000
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sub_4:
    li  gp, 4
    li  x11, 0x0000000000000003
    li  x12, 0x0000000000000007
    sub x14, x11, x12
    li  x7, 0xfffffffffffffffc
    bne x14, x7, failed
    li  x7, 0xfffffffffffffffc
    bne x14, x7, failed

sub_5:
    li  gp, 5
    li  x11, 0x0000000000000000
    li  x12, 0xffffffffffff8000
    sub x14, x11, x12
    li  x7, 0x0000000000008000
    bne x14, x7, failed
    li  x7, 32768
    bne x14, x7, failed

sub_6:
    li  gp, 6
    li  x11, 0xffffffff80000000
    li  x12, 0x0000000000000000
    sub x14, x11, x12
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

sub_7:
    li  gp, 7
    li  x11, 0xffffffff80000000
    li  x12, 0xffffffffffff8000
    sub x14, x11, x12
    li  x7, 0xffffffff80008000
    bne x14, x7, failed
    li  x7, 0xffffffff80008000
    bne x14, x7, failed

sub_8:
    li  gp, 8
    li  x11, 0x0000000000000000
    li  x12, 0x0000000000007fff
    sub x14, x11, x12
    li  x7, 0xffffffffffff8001
    bne x14, x7, failed
    li  x7, 0xffffffffffff8001
    bne x14, x7, failed

sub_9:
    li  gp, 9
    li  x11, 0x000000007fffffff
    li  x12, 0x0000000000000000
    sub x14, x11, x12
    li  x7, 0x000000007fffffff
    bne x14, x7, failed
    li  x7, 2147483647
    bne x14, x7, failed

sub_10:
    li  gp, 10
    li  x11, 0x000000007fffffff
    li  x12, 0x0000000000007fff
    sub x14, x11, x12
    li  x7, 0x000000007fff8000
    bne x14, x7, failed
    li  x7, 2147450880
    bne x14, x7, failed

sub_11:
    li  gp, 11
    li  x11, 0xffffffff80000000
    li  x12, 0x0000000000007fff
    sub x14, x11, x12
    li  x7, 0xffffffff7fff8001
    bne x14, x7, failed
    li  x7, 0xffffffff7fff8001
    bne x14, x7, failed

sub_12:
    li  gp, 12
    li  x11, 0x000000007fffffff
    li  x12, 0xffffffffffff8000
    sub x14, x11, x12
    li  x7, 0x0000000080007fff
    bne x14, x7, failed
    li  x7, 2147516415
    bne x14, x7, failed

sub_13:
    li  gp, 13
    li  x11, 0x0000000000000000
    li  x12, 0xffffffffffffffff
    sub x14, x11, x12
    li  x7, 0x0000000000000001
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

sub_14:
    li  gp, 14
    li  x11, 0xffffffffffffffff
    li  x12, 0x0000000000000001
    sub x14, x11, x12
    li  x7, 0xfffffffffffffffe
    bne x14, x7, failed
    li  x7, 0xfffffffffffffffe
    bne x14, x7, failed

sub_15:
    li  gp, 15
    li  x11, 0xffffffffffffffff
    li  x12, 0xffffffffffffffff
    sub x14, x11, x12
    li  x7, 0x0000000000000000
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

sub_16:
    li  gp, 16
    li  x11, 13
    li  x12, 11
    sub x11, x11, x12
    li  x7, 2
    bne x11, x7, failed
    li  x7, 2
    bne x11, x7, failed

sub_17:
    li  gp, 17
    li  x11, 14
    li  x12, 11
    sub x12, x11, x12
    li  x7, 3
    bne x12, x7, failed
    li  x7, 3
    bne x12, x7, failed

sub_18:
    li  gp, 18
    li  x11, 13
    sub x11, x11, x11
    li  x7, 0
    bne x11, x7, failed
    li  x7, 0
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

sub_19:
    li  gp, 19
    li  x4, 0
1:  li  x1, 13
    li  x2, 11
    sub x14, x1, x2
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 2
    bne x6, x7, failed
    li  x7, 2
    bne x6, x7, failed

sub_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 14
    li  x2, 11
    sub x14, x1, x2
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 3
    bne x6, x7, failed
    li  x7, 3
    bne x6, x7, failed

sub_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 15
    li  x2, 11
    sub x14, x1, x2
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 4
    bne x6, x7, failed
    li  x7, 4
    bne x6, x7, failed

sub_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 13
    li  x2, 11
    sub x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 2
    bne x14, x7, failed
    li  x7, 2
    bne x14, x7, failed

sub_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 14
    li  x2, 11
    nop
    sub x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 3
    bne x14, x7, failed
    li  x7, 3
    bne x14, x7, failed

sub_24:
    li  gp, 24
    li  x4, 0
1:  li  x1, 15
    li  x2, 11
    nop    
    nop
    sub x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 4
    bne x14, x7, failed
    li  x7, 4
    bne x14, x7, failed

sub_25:
    li  gp, 25
    li  x4, 0
1:  li  x1, 13
    nop
    li  x2, 11
    sub x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 2
    bne x14, x7, failed
    li  x7, 2
    bne x14, x7, failed

sub_26:
    li  gp, 26
    li  x4, 0
1:  li  x1, 14
    nop
    li  x2, 11
    nop
    sub x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 3
    bne x14, x7, failed
    li  x7, 3
    bne x14, x7, failed

sub_27:
    li  gp, 27
    li  x4, 0
1:  li  x1, 15
    nop    
    nop
    li  x2, 11
    sub x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 4
    bne x14, x7, failed
    li  x7, 4
    bne x14, x7, failed

sub_28:
    li  gp, 28
    li  x4, 0
1:  li  x2, 11
    li  x1, 13
    sub x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 2
    bne x14, x7, failed

sub_29:
    li  gp, 29
    li  x4, 0
1:  li  x2, 11
    li  x1, 14
    nop
    sub x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 3
    bne x14, x7, failed

sub_30:
    li  gp, 30
    li  x4, 0
1:  li  x2, 11
    li  x1, 15
    nop    
    nop
    sub x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 4
    bne x14, x7, failed

sub_31:
    li  gp, 31
    li  x4, 0
1:  li  x2, 11
    nop
    li  x1, 13
    sub x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 2
    bne x14, x7, failed

sub_32:
    li  gp, 32
    li  x4, 0
1:  li  x2, 11
    nop
    li  x1, 14
    nop
    sub x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 3
    bne x14, x7, failed

sub_33:
    li  gp, 33
    li  x4, 0
1:  li  x2, 11
    nop    
    nop
    li  x1, 15
    sub x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 4
    bne x14, x7, failed

sub_34:
    li  gp, 34
    li  x1, 18446744073709551601
    sub x2, x0, x1
    li  x7, 15
    bne x2, x7, failed

sub_35:
    li  gp, 35
    li  x1, 32
    sub x2, x1, x0
    li  x7, 32
    bne x2, x7, failed

sub_36:
    li  gp, 36
    sub x1, x0, x0
    li  x7, 0
    bne x1, x7, failed

sub_37:
    li  gp, 37
    li  x1, 16
    li  x2, 30
    sub x0, x1, x2
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
