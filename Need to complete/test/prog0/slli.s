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
# ### slli test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

slli_2:
    li  gp, 2
    li  x13, 1
    slli x14, x13, 0
    li  x7, 1
    bne x14, x7, failed

slli_3:
    li  gp, 3
    li  x13, 1
    slli x14, x13, 1
    li  x7, 2
    bne x14, x7, failed

slli_4:
    li  gp, 4
    li  x13, 1
    slli x14, x13, 7
    li  x7, 128
    bne x14, x7, failed

slli_5:
    li  gp, 5
    li  x13, 1
    slli x14, x13, 14
    li  x7, 16384
    bne x14, x7, failed

slli_6:
    li  gp, 6
    li  x13, 1
    slli x14, x13, 31
    li  x7, 2147483648
    bne x14, x7, failed

slli_7:
    li  gp, 7
    li  x13, 0xffffffffffffffff
    slli x14, x13, 0
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

slli_8:
    li  gp, 8
    li  x13, 0xffffffffffffffff
    slli x14, x13, 1
    li  x7, 0xfffffffffffffffe
    bne x14, x7, failed

slli_9:
    li  gp, 9
    li  x13, 0xffffffffffffffff
    slli x14, x13, 7
    li  x7, 0xffffffffffffff80
    bne x14, x7, failed

slli_10:
    li  gp, 10
    li  x13, 0xffffffffffffffff
    slli x14, x13, 14
    li  x7, 0xffffffffffffc000
    bne x14, x7, failed

slli_11:
    li  gp, 11
    li  x13, 0xffffffffffffffff
    slli x14, x13, 31
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

slli_12:
    li  gp, 12
    li  x13, 555819297
    slli x14, x13, 0
    li  x7, 555819297
    bne x14, x7, failed

slli_13:
    li  gp, 13
    li  x13, 555819297
    slli x14, x13, 1
    li  x7, 1111638594
    bne x14, x7, failed

slli_14:
    li  gp, 14
    li  x13, 555819297
    slli x14, x13, 7
    li  x7, 71144870016
    bne x14, x7, failed

slli_15:
    li  gp, 15
    li  x13, 555819297
    slli x14, x13, 14
    li  x7, 9106543362048
    bne x14, x7, failed

slli_16:
    li  gp, 16
    li  x13, 555819297
    slli x14, x13, 31
    li  x7, 1193612851550355456
    bne x14, x7, failed

slli_50:
    li  gp, 50
    li  x13, 1
    slli x14, x13, 63
    li  x7, 0x8000000000000000
    bne x14, x7, failed

slli_51:
    li  gp, 51
    li  x13, 0xffffffffffffffff
    slli x14, x13, 39
    li  x7, 0xffffff8000000000
    bne x14, x7, failed

slli_52:
    li  gp, 52
    li  x13, 555819297
    slli x14, x13, 43
    li  x7, 651060417224769536
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

slli_17:
    li  gp, 17
    li  x11, 1
    slli x11, x11, 7
    li  x7, 128
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

slli_18:
    li  gp, 18
    li  x4, 0
1:  li  x1, 1
    slli x14, x1, 7
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 128
    bne x6, x7, failed

slli_19:
    li  gp, 19
    li  x4, 0
1:  li  x1, 1
    slli x14, x1, 14
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16384
    bne x6, x7, failed

slli_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 1
    slli x14, x1, 31
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 2147483648
    bne x6, x7, failed

slli_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 1
    slli x14, x1, 7
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 128
    bne x14, x7, failed

slli_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 1
    nop
    slli x14, x1, 14
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 16384
    bne x14, x7, failed

slli_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 1
    nop    
    nop
    slli x14, x1, 31
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 2147483648
    bne x14, x7, failed

slli_24:
    li  gp, 24
    slli x1, x0, 31
    li  x7, 0
    bne x1, x7, failed

slli_25:
    li  gp, 25
    li  x1, 33
    slli x0, x1, 20
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
