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
# ### addw test cases
# ######################################

#-------------------------------------------------------------
# Arithmetic tests
#-------------------------------------------------------------

addw_2:
    li  gp, 2
    li  x11, 0x00000000
    li  x12, 0x00000000
    addw x14, x11, x12
    li  x7, 0x00000000
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

addw_3:
    li  gp, 3
    li  x11, 0x00000001
    li  x12, 0x00000001
    addw x14, x11, x12
    li  x7, 0x00000002
    bne x14, x7, failed
    li  x7, 2
    bne x14, x7, failed

addw_4:
    li  gp, 4
    li  x11, 0x00000003
    li  x12, 0x00000007
    addw x14, x11, x12
    li  x7, 0x0000000a
    bne x14, x7, failed
    li  x7, 10
    bne x14, x7, failed

addw_5:
    li  gp, 5
    li  x11, 0x0000000000000000
    li  x12, 0xffffffffffff8000
    addw x14, x11, x12
    li  x7, 0xffffffffffff8000
    bne x14, x7, failed
    li  x7, 0xffffffffffff8000
    bne x14, x7, failed

addw_6:
    li  gp, 6
    li  x11, 0xffffffff80000000
    li  x12, 0x00000000
    addw x14, x11, x12
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

addw_7:
    li  gp, 7
    li  x11, 0xffffffff80000000
    li  x12, 0xffffffffffff8000
    addw x14, x11, x12
    li  x7, 0x000000007fff8000
    bne x14, x7, failed
    li  x7, 2147450880
    bne x14, x7, failed

addw_8:
    li  gp, 8
    li  x11, 0x0000000000000000
    li  x12, 0x0000000000007fff
    addw x14, x11, x12
    li  x7, 0x0000000000007fff
    bne x14, x7, failed
    li  x7, 32767
    bne x14, x7, failed

addw_9:
    li  gp, 9
    li  x11, 0x000000007fffffff
    li  x12, 0x0000000000000000
    addw x14, x11, x12
    li  x7, 0x000000007fffffff
    bne x14, x7, failed
    li  x7, 2147483647
    bne x14, x7, failed

addw_10:
    li  gp, 10
    li  x11, 0x000000007fffffff
    li  x12, 0x0000000000007fff
    addw x14, x11, x12
    li  x7, 0xffffffff80007ffe
    bne x14, x7, failed
    li  x7, 0xffffffff80007ffe
    bne x14, x7, failed

addw_11:
    li  gp, 11
    li  x11, 0xffffffff80000000
    li  x12, 0x0000000000007fff
    addw x14, x11, x12
    li  x7, 0xffffffff80007fff
    bne x14, x7, failed
    li  x7, 0xffffffff80007fff
    bne x14, x7, failed

addw_12:
    li  gp, 12
    li  x11, 0x000000007fffffff
    li  x12, 0xffffffffffff8000
    addw x14, x11, x12
    li  x7, 0x000000007fff7fff
    bne x14, x7, failed
    li  x7, 2147450879
    bne x14, x7, failed

addw_13:
    li  gp, 13
    li  x11, 0x0000000000000000
    li  x12, 0xffffffffffffffff
    addw x14, x11, x12
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed
    li  x7, 0xffffffffffffffff
    bne x14, x7, failed

addw_14:
    li  gp, 14
    li  x11, 0xffffffffffffffff
    li  x12, 0x0000000000000001
    addw x14, x11, x12
    li  x7, 0x0000000000000000
    bne x14, x7, failed
    li  x7, 0
    bne x14, x7, failed

addw_15:
    li  gp, 15
    li  x11, 0xffffffffffffffff
    li  x12, 0xffffffffffffffff
    addw x14, x11, x12
    li  x7, 0xfffffffffffffffe
    bne x14, x7, failed
    li  x7, 0xfffffffffffffffe
    bne x14, x7, failed

addw_16:
    li  gp, 16
    li  x11, 0x0000000000000001
    li  x12, 0x000000007fffffff
    addw x14, x11, x12
    li  x7, 0xffffffff80000000
    bne x14, x7, failed
    li  x7, 0xffffffff80000000
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

addw_17:
    li  gp, 17
    li  x11, 13
    li  x12, 11
    addw x11, x11, x12
    li  x7, 24
    bne x11, x7, failed
    li  x7, 24
    bne x11, x7, failed

addw_18:
    li  gp, 18
    li  x11, 14
    li  x12, 11
    addw x12, x11, x12
    li  x7, 25
    bne x12, x7, failed
    li  x7, 25
    bne x12, x7, failed

addw_19:
    li  gp, 19
    li  x11, 13
    addw x11, x11, x11
    li  x7, 26
    bne x11, x7, failed
    li  x7, 26
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

addw_20:
    li  gp, 20
    li  x4, 0
1:  li  x1, 13
    li  x2, 11
    addw x14, x1, x2
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 24
    bne x6, x7, failed
    li  x7, 24
    bne x6, x7, failed

addw_21:
    li  gp, 21
    li  x4, 0
1:  li  x1, 14
    li  x2, 11
    addw x14, x1, x2
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 25
    bne x6, x7, failed
    li  x7, 25
    bne x6, x7, failed

addw_22:
    li  gp, 22
    li  x4, 0
1:  li  x1, 15
    li  x2, 11
    addw x14, x1, x2
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 26
    bne x6, x7, failed
    li  x7, 26
    bne x6, x7, failed

addw_23:
    li  gp, 23
    li  x4, 0
1:  li  x1, 13
    li  x2, 11
    addw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 24
    bne x14, x7, failed
    li  x7, 24
    bne x14, x7, failed

addw_24:
    li  gp, 24
    li  x4, 0
1:  li  x1, 14
    li  x2, 11
    nop
    addw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 25
    bne x14, x7, failed
    li  x7, 25
    bne x14, x7, failed

addw_25:
    li  gp, 25
    li  x4, 0
1:  li  x1, 15
    li  x2, 11
    nop    
    nop
    addw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 26
    bne x14, x7, failed
    li  x7, 26
    bne x14, x7, failed

addw_26:
    li  gp, 26
    li  x4, 0
1:  li  x1, 13
    nop
    li  x2, 11
    addw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 24
    bne x14, x7, failed
    li  x7, 24
    bne x14, x7, failed

addw_27:
    li  gp, 27
    li  x4, 0
1:  li  x1, 14
    nop
    li  x2, 11
    nop
    addw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 25
    bne x14, x7, failed
    li  x7, 25
    bne x14, x7, failed

addw_28:
    li  gp, 28
    li  x4, 0
1:  li  x1, 15
    nop    
    nop
    li  x2, 11
    addw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 26
    bne x14, x7, failed
    li  x7, 26
    bne x14, x7, failed

addw_29:
    li  gp, 29
    li  x4, 0
1:  li  x2, 11
    li  x1, 13
    addw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 24
    bne x14, x7, failed

addw_30:
    li  gp, 30
    li  x4, 0
1:  li  x2, 11
    li  x1, 14
    nop
    addw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 25
    bne x14, x7, failed

addw_31:
    li  gp, 31
    li  x4, 0
1:  li  x2, 11
    li  x1, 15
    nop    
    nop
    addw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 26
    bne x14, x7, failed

addw_32:
    li  gp, 32
    li  x4, 0
1:  li  x2, 11
    nop
    li  x1, 13
    addw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 24
    bne x14, x7, failed

addw_33:
    li  gp, 33
    li  x4, 0
1:  li  x2, 11
    nop
    li  x1, 14
    nop
    addw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 25
    bne x14, x7, failed

addw_34:
    li  gp, 34
    li  x4, 0
1:  li  x2, 11
    nop    
    nop
    li  x1, 15
    addw x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 26
    bne x14, x7, failed

addw_35:
    li  gp, 35
    li  x1, 15
    addw x2, x0, x1
    li  x7, 15
    bne x2, x7, failed

addw_36:
    li  gp, 36
    li  x1, 32
    addw x2, x1, x0
    li  x7, 32
    bne x2, x7, failed

addw_37:
    li  gp, 37
    addw x1, x0, x0
    li  x7, 0
    bne x1, x7, failed

addw_38:
    li  gp, 38
    li  x1, 16
    li  x2, 30
    addw x0, x1, x2
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
