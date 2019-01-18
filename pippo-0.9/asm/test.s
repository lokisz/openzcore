# 1 "asm_test.S"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "asm_test.S"
# 37 "asm_test.S"
# 1 "asm_test.h" 1
# 38 "asm_test.S" 2
# 1 "processor.h" 1
# 39 "asm_test.S" 2

.text
.global _start




_start:





core_initial:
    xor 0, 0, 0
    oris 4, 0, 0xffff
    mtspr 0x3D6, 4
# 65 "asm_test.S"
main_tests:
    addis 3, 0, 0xffff
    ori 3, 3, 0xc000
    ori 4, 0, 0xffff
    sth 4, 0(3)
    lwz 5, 0(3)
    and. 4, 4, 5
    beq test_ok
    b test_fail
# 99 "asm_test.S"
test_putchar:
    mtdcr 0x350, 31
    nop
    nop
    mfdcr 5, 0x353
    andi. 5, 5, 0x00000001
    bne (-8)
    addis 6, 0, 0x00000001@h
    ori 6, 6, 0x00000001@l
    mtdcr 0x352, 6
    blr




test_fail:
    addis 4, 0, 0x00000046@h
    ori 4, 4, 0x00000046@l
    mtdcr 0x350, 4
    mfdcr 5, 0x353
    andi. 5, 5, 0x00000001
    bne (-8)
    addis 6, 0, 0x00000001@h
    ori 6, 6, 0x00000001@l
    mtdcr 0x352, 6
    b 0




test_ok:
    addis 4, 0, 0x0000004f@h
    ori 4, 4, 0x0000004f@l
    mtdcr 0x350, 4
    mfdcr 5, 0x353
    andi. 5, 5, 0x00000001
    bne (-8)
    addis 6, 0, 0x00000001@h
    ori 6, 6, 0x00000001@l
    mtdcr 0x352, 6

    addis 4, 0, 0x0000004b@h
    ori 4, 4, 0x0000004b@l
    mtdcr 0x350, 4
    mfdcr 5, 0x353
    andi. 5, 5, 0x00000001
    bne (-8)
    addis 6, 0, 0x00000001@h
    ori 6, 6, 0x00000001@l
    mtdcr 0x352, 6
# 158 "asm_test.S"
    b 0





dbgexp_handler:
# 173 "asm_test.S"
    addis 4, 0, 0x00000045@h
    ori 4, 4, 0x00000045@l
    mtdcr 0x350, 4
    mfdcr 5, 0x353
    andi. 5, 5, 0x00000001
    bne (-8)
    addis 6, 0, 0x00000001@h
    ori 6, 6, 0x00000001@l
    mtdcr 0x352, 6
    b 0





    .space (0x1b14)



emulate_rom:
    addis 4, 0, 0x00000055@h
    ori 4, 4, 0x00000055@l
    mtdcr 0x350, 4
    mfdcr 5, 0x353
    andi. 5, 5, 0x00000001
    bne (-8)
    addis 6, 0, 0x00000001@h
    ori 6, 6, 0x00000001@l
    mtdcr 0x352, 6
    b 0




    .space (0x02d4)




exp_vector:

    ori 30, 0, 0xff00
    ori 31, 0, 0x0000
    bl dbgexp_handler
    rfci


    ori 30, 0, 0xff10
    ori 31, 0, 0x0001
    bl dbgexp_handler
    rfci


    ori 30, 0, 0xff20
    ori 31, 0, 0x0002
    bl dbgexp_handler
    rfi


    ori 30, 0, 0xff30
    ori 31, 0, 0x0003
    bl dbgexp_handler
    rfi


    ori 30, 0, 0xff40
    ori 31, 0, 0x0004
    bl dbgexp_handler
    rfi


    ori 30, 0, 0xff50
    ori 31, 0, 0x0005
    bl dbgexp_handler
    rfi


    ori 30, 0, 0xff60
    ori 31, 0, 0x0006
    bl dbgexp_handler
    rfi


    ori 30, 0, 0xff70
    ori 31, 0, 0x0007
    bl dbgexp_handler
    rfi


    ori 30, 0, 0xff80
    ori 31, 0, 0x0008
    bl dbgexp_handler
    rfi


    ori 30, 0, 0xff90
    ori 31, 0, 0x0009
    bl dbgexp_handler
    rfi


    ori 30, 0, 0xffa0
    ori 31, 0, 0x000a
    bl dbgexp_handler
    rfi


    ori 30, 0, 0xffb0
    ori 31, 0, 0x000b
    bl dbgexp_handler
    rfi


    ori 30, 0, 0xffc0
    ori 31, 0, 0x000c
    bl dbgexp_handler
    rfci


    ori 30, 0, 0xffd0
    ori 31, 0, 0x000d
    bl dbgexp_handler
    rfi


    ori 30, 0, 0xffe0
    ori 31, 0, 0x000e
    bl dbgexp_handler
    rfi



    bl dbgexp_handler
    rfci
    nop




    b core_initial







.data




.space (0x1800)




memory0: .long 0xffffd800,0xffffd804,0xffffd808,0xffffd80c,0xffffd810,0xffffd814,0xffffd818,0xffffd81c
memory1: .long 0x0000000f,0xfffffff0,0x000000ff,0xffffff00,0x0000ffff,0xffff0000,0xffffffff,0x00000000
memory2: .long 0x0000000f,0x000000f0,0x00000f00,0x0000f000,0x000f0000,0x00f00000,0x0f000000,0xf0000000
memory3: .long 0xfffffff0,0xffffff0f,0xfffff0ff,0xffff0fff,0xfff0ffff,0xff0fffff,0xf0ffffff,0x0fffffff,
memory4: .byte 0x62,0x61
         .align 2
memory5: .short 0xffff,0x1111
         .align 2
memory6: .short 0x7654,0x3456
memory7: .string "Hello, World"
         .align 2
memory8: .string "0123456789abcdefghijklmnopqrstuvwxyz"
         .align 2
memory9: .long 0xffffffff






.space (0x0734)
