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
# ### slti test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

slti_2:
    li  gp, 2
    li  x13, 0
    slti x14, x13, 0
    li  x7, 0
    bne x14, x7, failed

slti_3:
    li  gp, 3
    li  x13, 1
    slti x14, x13, 1
    li  x7, 0
    bne x14, x7, failed

slti_4:
    li  gp, 4
    li  x13, 3
    slti x14, x13, 7
    li  x7, 1
    bne x14, x7, failed

slti_5:
    li  gp, 5
    li  x13, 7
    slti x14, x13, 3
    li  x7, 0
    bne x14, x7, failed

slti_6:
    li  gp, 6
    li  x13, 0
    slti x14, x13, -2048
    li  x7, 0
    bne x14, x7, failed

slti_7:
    li  gp, 7
    li  x13, 0xffffffff80000000
    slti x14, x13, 0
    li  x7, 1
    bne x14, x7, failed

slti_8:
    li  gp, 8
    li  x13, 0xffffffff80000000
    slti x14, x13, -2048
    li  x7, 1
    bne x14, x7, failed

slti_9:
    li  gp, 9
    li  x13, 0
    slti x14, x13, 2047
    li  x7, 1
    bne x14, x7, failed

slti_10:
    li  gp, 10
    li  x13, 2147483647
    slti x14, x13, 0
    li  x7, 0
    bne x14, x7, failed

slti_11:
    li  gp, 11
    li  x13, 2147483647
    slti x14, x13, 2047
    li  x7, 0
    bne x14, x7, failed

slti_12:
    li  gp, 12
    li  x13, 0xffffffff80000000
    slti x14, x13, 2047
    li  x7, 1
    bne x14, x7, failed

slti_13:
    li  gp, 13
    li  x13, 2147483647
    slti x14, x13, -2048
    li  x7, 0
    bne x14, x7, failed

slti_14:
    li  gp, 14
    li  x13, 0
    slti x14, x13, -1
    li  x7, 0
    bne x14, x7, failed

slti_15:
    li  gp, 15
    li  x13, 0xffffffffffffffff
    slti x14, x13, 1
    li  x7, 1
    bne x14, x7, failed

slti_16:
    li  gp, 16
    li  x13, 0xffffffffffffffff
    slti x14, x13, -1
    li  x7, 0
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

slti_17:
    li  gp, 17
    li  x11, 11
    slti x11, x11, 13
    li  x7, 1
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

slti_18:
    li  gp, 18
    li  x4, 0
1:  li  x1, 15
    slti x14, x1, 10
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x6, x7, failed

slti_19:
    li  gp, 19
    li  x4, 0
1:  li  x1, 10
    slti x14, x1, 16
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x6, x7, failed

slti_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 16
    slti x14, x1, 9
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x6, x7, failed

slti_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 11
    slti x14, x1, 15
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

slti_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 17
    nop
    slti x14, x1, 8
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed

slti_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 12
    nop    
    nop
    slti x14, x1, 14
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

slti_24:
    li  gp, 24
    slti x1, x0, -1
    li  x7, 0
    bne x1, x7, failed

slti_25:
    li  gp, 25
    li  x1, 16711935
    slti x0, x1, -1
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
