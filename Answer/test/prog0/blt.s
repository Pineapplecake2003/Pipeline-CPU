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
# ### blt test cases
# ######################################

#-------------------------------------------------------------
# Branch tests
#-------------------------------------------------------------

# Each test checks both forward and backward branches

blt_2:
    li  gp, 2
    li  x1, 0
    li  x2, 1
    blt x1, x2, 2f
    bne x0, gp, failed
1:  bne x0, gp, 3f
2:  blt x1, x2, 1b
    bne x0, gp, failed
3:

blt_3:
    li  gp, 3
    li  x1, -1
    li  x2, 1
    blt x1, x2, 2f
    bne x0, gp, failed
1:  bne x0, gp, 3f
2:  blt x1, x2, 1b
    bne x0, gp, failed
3:

blt_4:
    li  gp, 4
    li  x1, -2
    li  x2, -1
    blt x1, x2, 2f
    bne x0, gp, failed
1:  bne x0, gp, 3f
2:  blt x1, x2, 1b
    bne x0, gp, failed
3:

blt_5:
    li  gp, 5
    li  x1, 1
    li  x2, 0
    blt x1, x2, 1f
    bne x0, gp, 2f
1:  bne x0, gp, failed
2:  blt x1, x2, 1b
3:

blt_6:
    li  gp, 6
    li  x1, 1
    li  x2, -1
    blt x1, x2, 1f
    bne x0, gp, 2f
1:  bne x0, gp, failed
2:  blt x1, x2, 1b
3:

blt_7:
    li  gp, 7
    li  x1, -1
    li  x2, -2
    blt x1, x2, 1f
    bne x0, gp, 2f
1:  bne x0, gp, failed
2:  blt x1, x2, 1b
3:

blt_8:
    li  gp, 8
    li  x1, 1
    li  x2, -2
    blt x1, x2, 1f
    bne x0, gp, 2f
1:  bne x0, gp, failed
2:  blt x1, x2, 1b
3:

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

blt_9:
    li  gp, 9
    li  x4, 0
1:  li  x1, 0
    li  x2, -1
    blt x1, x2, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

blt_10:
    li  gp, 10
    li  x4, 0
1:  li  x1, 0
    li  x2, -1
    nop
    blt x1, x2, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

blt_11:
    li  gp, 11
    li  x4, 0
1:  li  x1, 0
    li  x2, -1
    nop    
    nop
    blt x1, x2, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

blt_12:
    li  gp, 12
    li  x4, 0
1:  li  x1, 0
    nop
    li  x2, -1
    blt x1, x2, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

blt_13:
    li  gp, 13
    li  x4, 0
1:  li  x1, 0
    nop
    li  x2, -1
    nop
    blt x1, x2, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

blt_14:
    li  gp, 14
    li  x4, 0
1:  li  x1, 0
    nop    
    nop
    li  x2, -1
    blt x1, x2, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

blt_15:
    li  gp, 15
    li  x4, 0
1:  li  x1, 0
    li  x2, -1
    blt x1, x2, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

blt_16:
    li  gp, 16
    li  x4, 0
1:  li  x1, 0
    li  x2, -1
    nop
    blt x1, x2, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

blt_17:
    li  gp, 17
    li  x4, 0
1:  li  x1, 0
    li  x2, -1
    nop    
    nop
    blt x1, x2, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

blt_18:
    li  gp, 18
    li  x4, 0
1:  li  x1, 0
    nop
    li  x2, -1
    blt x1, x2, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

blt_19:
    li  gp, 19
    li  x4, 0
1:  li  x1, 0
    nop
    li  x2, -1
    nop
    blt x1, x2, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

blt_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 0
    nop    
    nop
    li  x2, -1
    blt x1, x2, failed
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

#-------------------------------------------------------------
# Test delay slot instructions not executed nor bypassed
#-------------------------------------------------------------

blt_21:
    li  gp, 21
    li  x1, 1
    blt x0, x1, 1f
    addi x1, x1, 1
    addi x1, x1, 1
    addi x1, x1, 1
    addi x1, x1, 1
1:  addi x1, x1, 1
    addi x1, x1, 1
    li  x7, 3
    bne x1, x7, failed

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
