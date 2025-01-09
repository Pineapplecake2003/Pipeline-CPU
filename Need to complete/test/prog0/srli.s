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
# ### srli test cases
# ######################################

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

srli_17:
    li  gp, 17
    li  x11, 2147483648
    srli x11, x11, 7
    li  x7, 16777216
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

srli_18:
    li  gp, 18
    li  x4, 0
1:  li  x1, 2147483648
    srli x14, x1, 7
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16777216
    bne x6, x7, failed

srli_19:
    li  gp, 19
    li  x4, 0
1:  li  x1, 2147483648
    srli x14, x1, 14
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 131072
    bne x6, x7, failed

srli_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 2147483649
    srli x14, x1, 31
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x6, x7, failed

srli_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 2147483648
    srli x14, x1, 7
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16777216
    bne x14, x7, failed

srli_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 2147483648
    nop
    srli x14, x1, 14
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 131072
    bne x14, x7, failed

srli_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 2147483649
    nop    
    nop
    srli x14, x1, 31
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 1
    bne x14, x7, failed

srli_24:
    li  gp, 24
    srli x1, x0, 4
    li  x7, 0
    bne x1, x7, failed

srli_25:
    li  gp, 25
    li  x1, 33
    srli x0, x1, 10
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
