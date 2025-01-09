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
# ### ori test cases
# ######################################

#-------------------------------------------------------------
# Logical tests
#-------------------------------------------------------------

ori_2:
    li  gp, 2
    li  x13, 0xffffffffff00ff00
    ori x14, x13, -241
    li  x7, 0xffffffffffffff0f
    bne x14, x7, failed

ori_3:
    li  gp, 3
    li  x13, 267390960
    ori x14, x13, 240
    li  x7, 267390960
    bne x14, x7, failed

ori_4:
    li  gp, 4
    li  x13, 16711935
    ori x14, x13, 1807
    li  x7, 16713727
    bne x14, x7, failed

ori_5:
    li  gp, 5
    li  x13, 0xfffffffff00ff00f
    ori x14, x13, 240
    li  x7, 0xfffffffff00ff0ff
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

ori_6:
    li  gp, 6
    li  x11, 4278255360
    ori x11, x11, 240
    li  x7, 4278255600
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

ori_7:
    li  gp, 7
    li  x4, 0
1:  li  x1, 267390960
    ori x14, x1, 240
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 267390960
    bne x6, x7, failed

ori_8:
    li  gp, 8
    li  x4, 0
1:  li  x1, 16711935
    ori x14, x1, 1807
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16713727
    bne x6, x7, failed

ori_9:
    li  gp, 9
    li  x4, 0
1:  li  x1, 0xfffffffff00ff00f
    ori x14, x1, 240
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffff00ff0ff
    bne x6, x7, failed

ori_10:
    li  gp, 10
    li  x4, 0
1:  li  x1, 267390960
    ori x14, x1, 240
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 267390960
    bne x14, x7, failed

ori_11:
    li  gp, 11
    li  x4, 0
1:  li  x1, 16711935
    nop
    ori x14, x1, -241
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

ori_12:
    li  gp, 12
    li  x4, 0
1:  li  x1, 0xfffffffff00ff00f
    nop    
    nop
    ori x14, x1, 240
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffff00ff0ff
    bne x14, x7, failed

ori_13:
    li  gp, 13
    ori x1, x0, 240
    li  x7, 240
    bne x1, x7, failed

ori_14:
    li  gp, 14
    li  x1, 16711935
    ori x0, x1, 1807
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
