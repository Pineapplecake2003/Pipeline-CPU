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
# ### jal test cases
# ######################################

#-------------------------------------------------------------
# Test 2: Basic test
#-------------------------------------------------------------

jal_2:
    li  gp, 2
    li  ra, 0

    jal x4, target_2
    
linkaddr_2:
    nop
    nop

    j failed

target_2:
    la  x2, linkaddr_2
    bne x2, x4, failed

#-------------------------------------------------------------
# Test delay slot instructions not executed nor bypassed
#-------------------------------------------------------------

jal_3:
    li  gp, 3
    li  ra, 1
    jal x0, 1f
    addi ra, ra, 1
    addi ra, ra, 1
    addi ra, ra, 1
    addi ra, ra, 1
1:  addi ra, ra, 1
    addi ra, ra, 1
    li  x7, 3
    bne ra, x7, failed

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
