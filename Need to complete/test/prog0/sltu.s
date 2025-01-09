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
# ### sltu test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

sltu_2:
    li  gp, 2
    li  x11, 0x00000000
    li  x12, 0x00000000
    sltu x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sltu_3:
    li  gp, 3
    li  x11, 0x00000001
    li  x12, 0x00000001
    sltu x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sltu_4:
    li  gp, 4
    li  x11, 0x00000003
    li  x12, 0x00000007
    sltu x14, x11, x12
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

sltu_5:
    li  gp, 5
    li  x11, 0x00000007
    li  x12, 0x00000003
    sltu x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sltu_6:
    li  gp, 6
    li  x11, 0x00000000
    li  x12, 0xffff8000
    sltu x14, x11, x12
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

sltu_7:
    li  gp, 7
    li  x11, 0x80000000
    li  x12, 0x00000000
    sltu x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sltu_8:
    li  gp, 8
    li  x11, 0x80000000
    li  x12, 0xffff8000
    sltu x14, x11, x12
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

sltu_9:
    li  gp, 9
    li  x11, 0x00000000
    li  x12, 0x00007fff
    sltu x14, x11, x12
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

sltu_10:
    li  gp, 10
    li  x11, 0x7fffffff
    li  x12, 0x00000000
    sltu x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sltu_11:
    li  gp, 11
    li  x11, 0x7fffffff
    li  x12, 0x00007fff
    sltu x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sltu_12:
    li  gp, 12
    li  x11, 0x80000000
    li  x12, 0x00007fff
    sltu x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sltu_13:
    li  gp, 13
    li  x11, 0x7fffffff
    li  x12, 0xffff8000
    sltu x14, x11, x12
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

sltu_14:
    li  gp, 14
    li  x11, 0x00000000
    li  x12, 0xffffffff
    sltu x14, x11, x12
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

sltu_15:
    li  gp, 15
    li  x11, 0xffffffff
    li  x12, 0x00000001
    sltu x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sltu_16:
    li  gp, 16
    li  x11, 0xffffffff
    li  x12, 0xffffffff
    sltu x14, x11, x12
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

sltu_17:
    li  gp, 17
    li  x11, 14
    li  x12, 13
    sltu x11, x11, x12
    li  x7, 0
    bne x11, x7, failed
    li  x7, 0
    bne x11, x7, failed

sltu_18:
    li  gp, 18
    li  x11, 11
    li  x12, 13
    sltu x12, x11, x12
    li  x7, 1
    bne x12, x7, failed
    li  x7, 1
    bne x12, x7, failed

sltu_19:
    li  gp, 19
    li  x11, 13
    sltu x11, x11, x11
    li  x7, 0
    bne x11, x7, failed
    li  x7, 0
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

sltu_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 11
    li  x2, 13
    sltu x14, x1, x2
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x6, x7, failed
    li  x7, 1
    bne x6, x7, failed

sltu_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 14
    li  x2, 13
    sltu x14, x1, x2
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x6, x7, failed
    li  x7, 0
    bne x6, x7, failed

sltu_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 12
    li  x2, 13
    sltu x14, x1, x2
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

sltu_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 14
    li  x2, 13
    sltu x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sltu_24:
    li  gp, 24
    li  x4, 0
1:  li  x1, 11
    li  x2, 13
    nop
    sltu x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

sltu_25:
    li  gp, 25
    li  x4, 0
1:  li  x1, 15
    li  x2, 13
    nop    
    nop
    sltu x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sltu_26:
    li  gp, 26
    li  x4, 0
1:  li  x1, 10
    nop
    li  x2, 13
    sltu x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

sltu_27:
    li  gp, 27
    li  x4, 0
1:  li  x1, 16
    nop
    li  x2, 13
    nop
    sltu x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

sltu_28:
    li  gp, 28
    li  x4, 0
1:  li  x1, 9
    nop    
    nop
    li  x2, 13
    sltu x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed
    li  x7, 1
    bne x14, x7, failed

sltu_29:
    li  gp, 29
    li  x4, 0
1:  li  x2, 13
    li  x1, 17
    sltu x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed

sltu_30:
    li  gp, 30
    li  x4, 0
1:  li  x2, 13
    li  x1, 8
    nop
    sltu x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

sltu_31:
    li  gp, 31
    li  x4, 0
1:  li  x2, 13
    li  x1, 18
    nop    
    nop
    sltu x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed

sltu_32:
    li  gp, 32
    li  x4, 0
1:  li  x2, 13
    nop
    li  x1, 7
    sltu x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

sltu_33:
    li  gp, 33
    li  x4, 0
1:  li  x2, 13
    nop
    li  x1, 19
    nop
    sltu x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed

sltu_34:
    li  gp, 34
    li  x4, 0
1:  li  x2, 13
    nop    
    nop
    li  x1, 6
    sltu x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

sltu_35:
    li  gp, 35
    li  x1, 18446744073709551615
    sltu x2, x0, x1
    li  x7, 1
    bne x2, x7, failed

sltu_36:
    li  gp, 36
    li  x1, 18446744073709551615
    sltu x2, x1, x0
    li  x7, 0
    bne x2, x7, failed

sltu_37:
    li  gp, 37
    sltu x1, x0, x0
    li  x7, 0
    bne x1, x7, failed

sltu_38:
    li  gp, 38
    li  x1, 16
    li  x2, 30
    sltu x0, x1, x2
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
