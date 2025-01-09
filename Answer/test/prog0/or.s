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
# ### or test cases
# ######################################

#-------------------------------------------------------------
# Logical tests
#-------------------------------------------------------------

or_2:
    li  gp, 2
    li  x11, 0xff00ff00
    li  x12, 0x0f0f0f0f
    or x14, x11, x12
    li  x7, 0xff0fff0f
    bne x14, x7, failed
    li  x7, 4279238415
    bne x14, x7, failed

or_3:
    li  gp, 3
    li  x11, 0x0ff00ff0
    li  x12, 0xf0f0f0f0
    or x14, x11, x12
    li  x7, 0xfff0fff0
    bne x14, x7, failed
    li  x7, 4293984240
    bne x14, x7, failed

or_4:
    li  gp, 4
    li  x11, 0x00ff00ff
    li  x12, 0x0f0f0f0f
    or x14, x11, x12
    li  x7, 0x0fff0fff
    bne x14, x7, failed
    li  x7, 268374015
    bne x14, x7, failed

or_5:
    li  gp, 5
    li  x11, 0xf00ff00f
    li  x12, 0xf0f0f0f0
    or x14, x11, x12
    li  x7, 0xf0fff0ff
    bne x14, x7, failed
    li  x7, 4043305215
    bne x14, x7, failed

#-------------------------------------------------------------
# Source/Destination tests
#-------------------------------------------------------------

or_6:
    li  gp, 6
    li  x11, 0xff00ff00
    li  x12, 0x0f0f0f0f
    or x11, x11, x12
    li  x7, 0xff0fff0f
    bne x11, x7, failed
    li  x7, 4279238415
    bne x11, x7, failed

or_7:
    li  gp, 7
    li  x11, 0xff00ff00
    li  x12, 0x0f0f0f0f
    or x12, x11, x12
    li  x7, 0xff0fff0f
    bne x12, x7, failed
    li  x7, 4279238415
    bne x12, x7, failed

or_8:
    li  gp, 8
    li  x11, 0xff00ff00
    or x11, x11, x11
    li  x7, 0xff00ff00
    bne x11, x7, failed
    li  x7, 4278255360
    bne x11, x7, failed

#-------------------------------------------------------------
# Bypassing tests
#-------------------------------------------------------------

or_9:
    li  gp, 9
    li  x4, 0
1:  li  x1, 0xff00ff00
    li  x2, 0x0f0f0f0f
    or x14, x1, x2
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xff0fff0f
    bne x6, x7, failed
    li  x7, 4279238415
    bne x6, x7, failed

or_10:
    li  gp, 10
    li  x4, 0
1:  li  x1, 0x0ff00ff0
    li  x2, 0xf0f0f0f0
    or x14, x1, x2
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfff0fff0
    bne x6, x7, failed
    li  x7, 4293984240
    bne x6, x7, failed

or_11:
    li  gp, 11
    li  x4, 0
1:  li  x1, 0x00ff00ff
    li  x2, 0x0f0f0f0f
    or x14, x1, x2
    nop    
    nop
    addi  x6, x14, 0
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0fff0fff
    bne x6, x7, failed
    li  x7, 268374015
    bne x6, x7, failed

or_12:
    li  gp, 12
    li  x4, 0
1:  li  x1, 0xff00ff00
    li  x2, 0x0f0f0f0f
    or x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xff0fff0f
    bne x14, x7, failed
    li  x7, 4279238415
    bne x14, x7, failed

or_13:
    li  gp, 13
    li  x4, 0
1:  li  x1, 0x0ff00ff0
    li  x2, 0xf0f0f0f0
    nop
    or x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfff0fff0
    bne x14, x7, failed
    li  x7, 4293984240
    bne x14, x7, failed

or_14:
    li  gp, 14
    li  x4, 0
1:  li  x1, 0x00ff00ff
    li  x2, 0x0f0f0f0f
    nop    
    nop
    or x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0fff0fff
    bne x14, x7, failed
    li  x7, 268374015
    bne x14, x7, failed

or_15:
    li  gp, 15
    li  x4, 0
1:  li  x1, 0xff00ff00
    nop
    li  x2, 0x0f0f0f0f
    or x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xff0fff0f
    bne x14, x7, failed
    li  x7, 4279238415
    bne x14, x7, failed

or_16:
    li  gp, 16
    li  x4, 0
1:  li  x1, 0x0ff00ff0
    nop
    li  x2, 0xf0f0f0f0
    nop
    or x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0xfff0fff0
    bne x14, x7, failed
    li  x7, 4293984240
    bne x14, x7, failed

or_17:
    li  gp, 17
    li  x4, 0
1:  li  x1, 0x00ff00ff
    nop    
    nop
    li  x2, 0x0f0f0f0f
    or x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 0x0fff0fff
    bne x14, x7, failed
    li  x7, 268374015
    bne x14, x7, failed

or_18:
    li  gp, 18
    li  x4, 0
1:  li  x2, 252645135
    li  x1, 4278255360
    or x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 4279238415
    bne x14, x7, failed

or_19:
    li  gp, 19
    li  x4, 0
1:  li  x2, 4042322160
    li  x1, 267390960
    nop
    or x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 4293984240
    bne x14, x7, failed

or_20:
    li  gp, 20
    li  x4, 0
1:  li  x2, 252645135
    li  x1, 16711935
    nop    
    nop
    or x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 268374015
    bne x14, x7, failed

or_21:
    li  gp, 21
    li  x4, 0
1:  li  x2, 252645135
    nop
    li  x1, 4278255360
    or x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 4279238415
    bne x14, x7, failed

or_22:
    li  gp, 22
    li  x4, 0
1:  li  x2, 4042322160
    nop
    li  x1, 267390960
    nop
    or x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 4293984240
    bne x14, x7, failed

or_23:
    li  gp, 23
    li  x4, 0
1:  li  x2, 252645135
    nop    
    nop
    li  x1, 16711935
    or x14, x1, x2
    addi  x4, x4, 1
    li  x5, 2
    bne x4, x5, 1b
    li  x7, 268374015
    bne x14, x7, failed

or_24:
    li  gp, 24
    li  x1, 4278255360
    or x2, x0, x1
    li  x7, 4278255360
    bne x2, x7, failed

or_25:
    li  gp, 25
    li  x1, 16711935
    or x2, x1, x0
    li  x7, 16711935
    bne x2, x7, failed

or_26:
    li  gp, 26
    or x1, x0, x0
    li  x7, 0
    bne x1, x7, failed

or_27:
    li  gp, 27
    li  x1, 286331153
    li  x2, 572662306
    or x0, x1, x2
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
