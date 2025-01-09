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
# ### slt test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

slt_2:
    li  gp, 2
    li  x11, 0x0000000000000000
    li  x12, 0x0000000000000000
    slt x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

slt_3:
    li  gp, 3
    li  x11, 0x0000000000000001
    li  x12, 0x0000000000000001
    slt x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

slt_4:
    li  gp, 4
    li  x11, 0x0000000000000003
    li  x12, 0x0000000000000007
    slt x14, x11, x12
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

slt_5:
    li  gp, 5
    li  x11, 0x0000000000000007
    li  x12, 0x0000000000000003
    slt x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

slt_6:
    li  gp, 6
    li  x11, 0x0000000000000000
    li  x12, 0xffffffffffff8000
    slt x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

slt_7:
    li  gp, 7
    li  x11, 0xffffffff80000000
    li  x12, 0x0000000000000000
    slt x14, x11, x12
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

slt_8:
    li  gp, 8
    li  x11, 0xffffffff80000000
    li  x12, 0xffffffffffff8000
    slt x14, x11, x12
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

slt_9:
    li  gp, 9
    li  x11, 0x0000000000000000
    li  x12, 0x0000000000007fff
    slt x14, x11, x12
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

slt_10:
    li  gp, 10
    li  x11, 0x000000007fffffff
    li  x12, 0x0000000000000000
    slt x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

slt_11:
    li  gp, 11
    li  x11, 0x000000007fffffff
    li  x12, 0x0000000000007fff
    slt x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

slt_12:
    li  gp, 12
    li  x11, 0xffffffff80000000
    li  x12, 0x0000000000007fff
    slt x14, x11, x12
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

slt_13:
    li  gp, 13
    li  x11, 0x000000007fffffff
    li  x12, 0xffffffffffff8000
    slt x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

slt_14:
    li  gp, 14
    li  x11, 0x0000000000000000
    li  x12, 0xffffffffffffffff
    slt x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

slt_15:
    li  gp, 15
    li  x11, 0xffffffffffffffff
    li  x12, 0x0000000000000001
    slt x14, x11, x12
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

slt_16:
    li  gp, 16
    li  x11, 0xffffffffffffffff
    li  x12, 0xffffffffffffffff
    slt x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

slt_17:
    li  gp, 17
    li  x11, 14
    li  x12, 13
    slt x11, x11, x12
    li  x7, 0
    bne x11, x7, failed
    li  x7, 0
    bne x11, x7, failed

slt_18:
    li  gp, 18
    li  x11, 11
    li  x12, 13
    slt x12, x11, x12
    li  x7, 1
    bne x12, x7, failed
    li  x7, 1
    bne x12, x7, failed

slt_19:
    li  gp, 19
    li  x11, 13
    slt x11, x11, x11
    li  x7, 0
    bne x11, x7, failed
    li  x7, 0
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

slt_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 11
    li  x2, 13
    slt x14, x1, x2
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x6, x7, failed
    li  x7, 1
    bne x6, x7, failed

slt_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 14
    li  x2, 13
    slt x14, x1, x2
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x6, x7, failed
    li  x7, 0
    bne x6, x7, failed

slt_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 12
    li  x2, 13
    slt x14, x1, x2
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x6, x7, failed
    li  x7, 1
    bne x6, x7, failed

slt_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 14
    li  x2, 13
    slt x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

slt_24:
    li  gp, 24
    li  x4, 0
1:  li  x1, 11
    li  x2, 13
    nop
    slt x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

slt_25:
    li  gp, 25
    li  x4, 0
1:  li  x1, 15
    li  x2, 13
    nop    
    nop
    slt x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

slt_26:
    li  gp, 26
    li  x4, 0
1:  li  x1, 10
    nop
    li  x2, 13
    slt x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

slt_27:
    li  gp, 27
    li  x4, 0
1:  li  x1, 16
    nop
    li  x2, 13
    nop
    slt x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

slt_28:
    li  gp, 28
    li  x4, 0
1:  li  x1, 9
    nop    
    nop
    li  x2, 13
    slt x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

slt_29:
    li  gp, 29
    li  x4, 0
1:  li  x2, 13
    li  x1, 17
    slt x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed

slt_30:
    li  gp, 30
    li  x4, 0
1:  li  x2, 13
    li  x1, 8
    nop
    slt x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

slt_31:
    li  gp, 31
    li  x4, 0
1:  li  x2, 13
    li  x1, 18
    nop    
    nop
    slt x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed

slt_32:
    li  gp, 32
    li  x4, 0
1:  li  x2, 13
    nop
    li  x1, 7
    slt x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

slt_33:
    li  gp, 33
    li  x4, 0
1:  li  x2, 13
    nop
    li  x1, 19
    nop
    slt x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed

slt_34:
    li  gp, 34
    li  x4, 0
1:  li  x2, 13
    nop    
    nop
    li  x1, 6
    slt x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

slt_35:
    li  gp, 35
    li  x1, 18446744073709551615
    slt x2, x0, x1
    li  x7, 0
    bne x2, x7, failed

slt_36:
    li  gp, 36
    li  x1, 18446744073709551615
    slt x2, x1, x0
    li  x7, 1
    bne x2, x7, failed

slt_37:
    li  gp, 37
    slt x1, x0, x0
    li  x7, 0
    bne x1, x7, failed

slt_38:
    li  gp, 38
    li  x1, 16
    li  x2, 30
    slt x0, x1, x2
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
