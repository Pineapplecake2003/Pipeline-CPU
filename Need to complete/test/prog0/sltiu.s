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
# ### sltiu test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

sltiu_2:
    li  gp, 2
    li  x13, 0
    sltiu x14, x13, 0
    li  x7, 0
    bne x14, x7, failed

sltiu_3:
    li  gp, 3
    li  x13, 1
    sltiu x14, x13, 1
    li  x7, 0
    bne x14, x7, failed

sltiu_4:
    li  gp, 4
    li  x13, 3
    sltiu x14, x13, 7
    li  x7, 1
    bne x14, x7, failed

sltiu_5:
    li  gp, 5
    li  x13, 7
    sltiu x14, x13, 3
    li  x7, 0
    bne x14, x7, failed

sltiu_6:
    li  gp, 6
    li  x13, 0
    sltiu x14, x13, -2048
    li  x7, 1
    bne x14, x7, failed

sltiu_7:
    li  gp, 7
    li  x13, 0xffffffff80000000
    sltiu x14, x13, 0
    li  x7, 0
    bne x14, x7, failed

sltiu_8:
    li  gp, 8
    li  x13, 0xffffffff80000000
    sltiu x14, x13, -2048
    li  x7, 1
    bne x14, x7, failed

sltiu_9:
    li  gp, 9
    li  x13, 0
    sltiu x14, x13, 2047
    li  x7, 1
    bne x14, x7, failed

sltiu_10:
    li  gp, 10
    li  x13, 2147483647
    sltiu x14, x13, 0
    li  x7, 0
    bne x14, x7, failed

sltiu_11:
    li  gp, 11
    li  x13, 2147483647
    sltiu x14, x13, 2047
    li  x7, 0
    bne x14, x7, failed

sltiu_12:
    li  gp, 12
    li  x13, 0xffffffff80000000
    sltiu x14, x13, 2047
    li  x7, 0
    bne x14, x7, failed

sltiu_13:
    li  gp, 13
    li  x13, 2147483647
    sltiu x14, x13, -2048
    li  x7, 1
    bne x14, x7, failed

sltiu_14:
    li  gp, 14
    li  x13, 0
    sltiu x14, x13, -1
    li  x7, 1
    bne x14, x7, failed

sltiu_15:
    li  gp, 15
    li  x13, 0xffffffffffffffff
    sltiu x14, x13, 1
    li  x7, 0
    bne x14, x7, failed

sltiu_16:
    li  gp, 16
    li  x13, 0xffffffffffffffff
    sltiu x14, x13, -1
    li  x7, 0
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

sltiu_17:
    li  gp, 17
    li  x11, 11
    sltiu x11, x11, 13
    li  x7, 1
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

sltiu_18:
    li  gp, 18
    li  x4, 0
1:  li  x1, 15
    sltiu x14, x1, 10
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x6, x7, failed

sltiu_19:
    li  gp, 19
    li  x4, 0
1:  li  x1, 10
    sltiu x14, x1, 16
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x6, x7, failed

sltiu_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 16
    sltiu x14, x1, 9
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x6, x7, failed

sltiu_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 11
    sltiu x14, x1, 15
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

sltiu_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 17
    nop
    sltiu x14, x1, 8
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0
    bne x14, x7, failed

sltiu_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 12
    nop    
    nop
    sltiu x14, x1, 14
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

sltiu_24:
    li  gp, 24
    sltiu x1, x0, -1
    li  x7, 1
    bne x1, x7, failed

sltiu_25:
    li  gp, 25
    li  x1, 16711935
    sltiu x0, x1, -1
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
