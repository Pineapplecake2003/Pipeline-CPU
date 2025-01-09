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
# ### srai test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

srai_2:
    li  gp, 2
    li  x13, 0xffffff8000000000
    srai x14, x13, 0
    li  x7, 0xffffff8000000000
    bne x14, x7, failed

srai_3:
    li  gp, 3
    li  x13, 0xffffffff80000000
    srai x14, x13, 1
    li  x7, 0xffffffffc0000000
    bne x14, x7, failed

srai_4:
    li  gp, 4
    li  x13, 0xffffffff80000000
    srai x14, x13, 7
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

srai_5:
    li  gp, 5
    li  x13, 0xffffffff80000000
    srai x14, x13, 14
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

srai_6:
    li  gp, 6
    li  x13, 0xffffffff80000001
    srai x14, x13, 31
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

srai_7:
    li  gp, 7
    li  x13, 2147483647
    srai x14, x13, 0
    li  x7, 2147483647
    bne x14, x7, failed

srai_8:
    li  gp, 8
    li  x13, 2147483647
    srai x14, x13, 1
    li  x7, 1073741823
    bne x14, x7, failed

srai_9:
    li  gp, 9
    li  x13, 2147483647
    srai x14, x13, 7
    li  x7, 16777215
    bne x14, x7, failed

srai_10:
    li  gp, 10
    li  x13, 2147483647
    srai x14, x13, 14
    li  x7, 131071
    bne x14, x7, failed

srai_11:
    li  gp, 11
    li  x13, 2147483647
    srai x14, x13, 31
    li  x7, 0
    bne x14, x7, failed

srai_12:
    li  gp, 12
    li  x13, 0xffffffff81818181
    srai x14, x13, 0
    li  x7, 0xffffffff81818181
    bne x14, x7, failed

srai_13:
    li  gp, 13
    li  x13, 0xffffffff81818181
    srai x14, x13, 1
    li  x7, 0xffffffffc0c0c0c0
    bne x14, x7, failed

srai_14:
    li  gp, 14
    li  x13, 0xffffffff81818181
    srai x14, x13, 7
    li  x7, 0xffffffffff030303
    bne x14, x7, failed

srai_15:
    li  gp, 15
    li  x13, 0xffffffff81818181
    srai x14, x13, 14
    li  x7, 0xfffffffffffe0606
    bne x14, x7, failed

srai_16:
    li  gp, 16
    li  x13, 0xffffffff81818181
    srai x14, x13, 31
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

srai_17:
    li  gp, 17
    li  x11, 0xffffffff80000000
    srai x11, x11, 7
    li  x7, 0xffffffffff000000
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

srai_18:
    li  gp, 18
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    srai x14, x1, 7
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x6, x7, failed

srai_19:
    li  gp, 19
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    srai x14, x1, 14
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x6, x7, failed

srai_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 0xffffffff80000001
    srai x14, x1, 31
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x6, x7, failed

srai_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    srai x14, x1, 7
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffff000000
    bne x14, x7, failed

srai_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 0xffffffff80000000
    nop
    srai x14, x1, 14
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfffffffffffe0000
    bne x14, x7, failed

srai_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 0xffffffff80000001
    nop    
    nop
    srai x14, x1, 31
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

srai_24:
    li  gp, 24
    srai x1, x0, 4
    li  x7, 0
    bne x1, x7, failed

srai_25:
    li  gp, 25
    li  x1, 33
    srai x0, x1, 10
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
