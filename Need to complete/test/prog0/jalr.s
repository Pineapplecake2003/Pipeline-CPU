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
# ### jalr test cases
# ######################################

#-------------------------------------------------------------
# Test 2: Basic test
#-------------------------------------------------------------

jalr_2:
    li  gp, 2
    li  t0, 0
    la  t1, target_2

    jalr t0, t1, 0
linkaddr_2:
    j failed

target_2:
    la  t1, linkaddr_2
    bne t0, t1, failed

#-------------------------------------------------------------
# Test 3: Basic test2, rs = rd
#-------------------------------------------------------------

jalr_3:
    li  gp, 3
    la  t0, target_3

    jalr t0, t0, 0
linkaddr_3:
    j failed

target_3:
    la  t1, linkaddr_3
    bne t0, t1, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

jalr_4:
    li  gp, 4
    li  x4, 0
1:  la  x6, 2f
    jalr x13, x6, 0
    bne x0, gp, failed
2:  addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

jalr_5:
    li  gp, 5
    li  x4, 0
1:  la  x6, 2f
    nop
    jalr x13, x6, 0
    bne x0, gp, failed
2:  addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

jalr_6:
    li  gp, 6
    li  x4, 0
1:  la  x6, 2f
    nop    
    nop
    jalr x13, x6, 0
    bne x0, gp, failed
2:  addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b

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
