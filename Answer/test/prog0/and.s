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
# ### and test cases
# ######################################

#-------------------------------------------------------------
# Logical tests
#-------------------------------------------------------------

and_2:
    li  gp, 2
    li  x11, 0xff00ff00
    li  x12, 0x0f0f0f0f
    and x14, x11, x12
    li  x7, 0x0f000f00
    bne x14, x7, failed
    li  x7, 251662080
    bne x14, x7, failed

and_3:
    li  gp, 3
    li  x11, 0x0ff00ff0
    li  x12, 0xf0f0f0f0
    and x14, x11, x12
    li  x7, 0x00f000f0
    bne x14, x7, failed
    li  x7, 15728880
    bne x14, x7, failed

and_4:
    li  gp, 4
    li  x11, 0x00ff00ff
    li  x12, 0x0f0f0f0f
    and x14, x11, x12
    li  x7, 0x000f000f
    bne x14, x7, failed
    li  x7, 983055
    bne x14, x7, failed

and_5:
    li  gp, 5
    li  x11, 0xf00ff00f
    li  x12, 0xf0f0f0f0
    and x14, x11, x12
    li  x7, 0xf000f000
    bne x14, x7, failed
    li  x7, 4026593280
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

and_6:
    li  gp, 6
    li  x11, 0xff00ff00
    li  x12, 0x0f0f0f0f
    and x11, x11, x12
    li  x7, 0x0f000f00
    bne x11, x7, failed
    li  x7, 251662080
    bne x11, x7, failed

and_7:
    li  gp, 7
    li  x11, 0x0ff00ff0
    li  x12, 0xf0f0f0f0
    and x12, x11, x12
    li  x7, 0x00f000f0
    bne x12, x7, failed
    li  x7, 15728880
    bne x12, x7, failed

and_8:
    li  gp, 8
    li  x11, 0xff00ff00
    and x11, x11, x11
    li  x7, 0xff00ff00
    bne x11, x7, failed
    li  x7, 4278255360
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

and_9:
    li  gp, 9
    li  x4, 0
1:  li  x1, 0xff00ff00
    li  x2, 0x0f0f0f0f
    and x14, x1, x2
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0f000f00
    bne x6, x7, failed
    li  x7, 251662080
    bne x6, x7, failed

and_10:
    li  gp, 10
    li  x4, 0
1:  li  x1, 0x0ff00ff0
    li  x2, 0xf0f0f0f0
    and x14, x1, x2
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x00f000f0
    bne x6, x7, failed
    li  x7, 15728880
    bne x6, x7, failed

and_11:
    li  gp, 11
    li  x4, 0
1:  li  x1, 0x00ff00ff
    li  x2, 0x0f0f0f0f
    and x14, x1, x2
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x000f000f
    bne x6, x7, failed
    li  x7, 983055
    bne x6, x7, failed

and_12:
    li  gp, 12
    li  x4, 0
1:  li  x1, 0xff00ff00
    li  x2, 0x0f0f0f0f
    and x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0f000f00
    bne x14, x7, failed
    li  x7, 251662080
    bne x14, x7, failed

and_13:
    li  gp, 13
    li  x4, 0
1:  li  x1, 0x0ff00ff0
    li  x2, 0xf0f0f0f0
    nop
    and x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x00f000f0
    bne x14, x7, failed
    li  x7, 15728880
    bne x14, x7, failed

and_14:
    li  gp, 14
    li  x4, 0
1:  li  x1, 0x00ff00ff
    li  x2, 0x0f0f0f0f
    nop    
    nop
    and x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x000f000f
    bne x14, x7, failed
    li  x7, 983055
    bne x14, x7, failed

and_15:
    li  gp, 15
    li  x4, 0
1:  li  x1, 0xff00ff00
    nop
    li  x2, 0x0f0f0f0f
    and x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0f000f00
    bne x14, x7, failed
    li  x7, 251662080
    bne x14, x7, failed

and_16:
    li  gp, 16
    li  x4, 0
1:  li  x1, 0x0ff00ff0
    nop
    li  x2, 0xf0f0f0f0
    nop
    and x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x00f000f0
    bne x14, x7, failed
    li  x7, 15728880
    bne x14, x7, failed

and_17:
    li  gp, 17
    li  x4, 0
1:  li  x1, 0x00ff00ff
    nop    
    nop
    li  x2, 0x0f0f0f0f
    and x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x000f000f
    bne x14, x7, failed
    li  x7, 983055
    bne x14, x7, failed

and_18:
    li  gp, 18
    li  x4, 0
1:  li  x2, 252645135
    li  x1, 4278255360
    and x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 251662080
    bne x14, x7, failed

and_19:
    li  gp, 19
    li  x4, 0
1:  li  x2, 4042322160
    li  x1, 267390960
    nop
    and x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 15728880
    bne x14, x7, failed

and_20:
    li  gp, 20
    li  x4, 0
1:  li  x2, 252645135
    li  x1, 16711935
    nop    
    nop
    and x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 983055
    bne x14, x7, failed

and_21:
    li  gp, 21
    li  x4, 0
1:  li  x2, 252645135
    nop
    li  x1, 4278255360
    and x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 251662080
    bne x14, x7, failed

and_22:
    li  gp, 22
    li  x4, 0
1:  li  x2, 4042322160
    nop
    li  x1, 267390960
    nop
    and x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 15728880
    bne x14, x7, failed

and_23:
    li  gp, 23
    li  x4, 0
1:  li  x2, 252645135
    nop    
    nop
    li  x1, 16711935
    and x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 983055
    bne x14, x7, failed

and_24:
    li  gp, 24
    li  x1, 4278255360
    and x2, x0, x1
    li  x7, 0
    bne x2, x7, failed

and_25:
    li  gp, 25
    li  x1, 16711935
    and x2, x1, x0
    li  x7, 0
    bne x2, x7, failed

and_26:
    li  gp, 26
    and x1, x0, x0
    li  x7, 0
    bne x1, x7, failed

and_27:
    li  gp, 27
    li  x1, 286331153
    li  x2, 572662306
    and x0, x1, x2
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
