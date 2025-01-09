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
# ### lui test cases
# ######################################

#-------------------------------------------------------------
# Basic tests
#-------------------------------------------------------------

lui_2:
    li  gp, 2
    lui x1, 0x00000
    li  x7, 0x0000000000000000
    bne x1, x7, failed

lui_3:
    li  gp, 3
    lui x1, 0xfffff
    sra x1,x1,1
    li  x7, 0xfffffffffffff800
    bne x1, x7, failed

lui_4:
    li  gp, 4
    lui x1, 0x7ffff
    sra x1,x1,20
    li  x7, 0x00000000000007ff
    bne x1, x7, failed

lui_5:
    li  gp, 5
    lui x1, 0x80000
    sra x1,x1,20
    li  x7, 0xfffffffffffff800
    bne x1, x7, failed
    
lui_6:
    li  gp, 6
    lui x0, 0x80000
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
