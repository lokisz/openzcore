/*
 * File:        asm_xxx.S
 * Project:     pippo
 * Author:      kiss@pwrsemi
 * Maintainer:  kiss@pwrsemi
 * Checker:
 * Description:
 *      Template file for powerpc assembler program
 *      FEP/SIM utilities include test_putchar, test_fail, test_ok, to indicate the status of running tests
 *          Here FEP means FPGA Emulation Platform, SIM means SIMulation environment
 *      Memory usage for PSS(Processor Sub-System) under FEP/SIM environment: 
 *          uocm: configurable, currently 16KB, can be burn from pc online. address: 0xffff_c000 - 0xffff_ffff. 
 *              0x0000_0000 - 0xffff_bfff: Non-Available, there are only ucom memory for PSS
 *              0xffff_c000 - 0xffff_d7ff: RAM, 7KB, recommond usage - "Read/Write", for program run-time data
 *              0xffff_d800 - 0xffff_dfff: RAM, 1KB, recommond usage - "Read", for read only data
 *              0xffff_e000 - 0xffff_ffff: 8KB, recommond usage - "Execute", for code section
 *                  0xffff_e000 - 0xffff_fbff: test programs
 *                  0xffff_fc00 - 0xffff_feff: emulation rom
 *                  0xffff_ff00 - 0xffff_fff8: exception vector for special mode
 *                  0xffff_fffc: reset vector
 *      Definition for test output on console
 *          "OK" - test successed
 *          "F"  - test failed
 *          "E"  - un-expected exception happened
 *          "U"  - un-support instruction executed
 * Notes:
 *      1, you should compile twice, to get the real size of tests, then retarget program to fixed address
 *      2, for consistency of FEP and SIM, you should not use input from uartlite to contrl the test flow.
 *      3, for detailed description of PSS, see documentation
 * Tests Plan:
 *  Goal: 
 *  Register usage:
 *  dependence: 
 *  workflow: 
 */

#include "asm_test.h"
#include "processor.h"

.text
.global _start

//
// all tests located at top half of uocm
//
_start:

/////////////////////////////////////////////////////////////////////////////////
//
// system and core initialization
//
core_initial:
    xor     r0, r0, r0          // always set r0 to 0
    oris    r4, r0, 0xffff      // 
    mtspr   SPRN_EVPR, r4       // set exception prefix vector to 0xffff_0000
//    addis r0,0,0x0000
//    ori r5,r0,0x0040
//    mtmsr r5
//    isync

/////////////////////////////////////////////////////////////////////////////////
//
// Here is real test program
//
main_tests:
    addis r3, r0, 0xffff
    ori r3, r3, 0xc000      // r3: 0xffff_c000
    ori r4, r0, 0xffff      // r4: 0000_ffff    
    sth r4, 0(r3)           // mem(r3:0xffff_c000): ffff_0000
    lwz r5, 0(r3)           // r5: ffff_0000
    and.  r4, r4, r5        // r4: 0000_0000
    beq test_ok             // branch to test_ok if successed
    b test_fail             // branch to test_fail if failed
    
/////////////////////////////////////////////////////////////////////////////////
//
// FEP/SIM utilities of dsu_uartlite
//
// dsu_uartlite: there are four registers mapped as pippo's SPRs.
//      reg1: DCRN_DSUTX, 8 bit, Access-R/W. 
//      reg2: DCRN_DSURX, 8 bit, Access-R only. 
//      reg3: DCRN_DSUSTA, 8 bit, Access-R only.
//      reg4: DCRN_DSUCTRL, 8 bit, Access-R/W.
//      for detailed description, please check the doc
// output flow 
//      1, write the data to tx_data reg 
//      2, check the tx_busy bit of dsu_uartlite, if assert, wait
//      3, send the tx_start cmd
//      4, branch back via link register
// note
//      before check STA, programmer should insert 3 instruction before reading STA

//
// test_putchar
//  note: before branch to test_putchar, test must
//      1, write the tag byte to r31, to help diagnostic
//      2, save the address of next instruction to link register
//
test_putchar:
    mtdcr DCRN_DSUTX, r31
    nop 
    nop                         // data ready
    mfdcr r5, DCRN_DSUSTA
    andi. r5, r5, DSUTX_BUSY
    bne (-8)                    // check busy bit of register dsu_sta; if busy, wait    
    addis r6, r0, DSUTX_GO@h
    ori r6, r6, DSUTX_GO@l
    mtdcr DCRN_DSUCTRL, r6      // send tx command to register dsu_ctrl
    blr

//
// test_fail: error exist in test, send out "F"
//
test_fail:
    addis r4, r0, ASCII_F@h
    ori r4, r4, ASCII_F@l
    mtdcr DCRN_DSUTX, r4        // data ready
    mfdcr r5, DCRN_DSUSTA
    andi. r5, r5, DSUTX_BUSY
    bne (-8)                    // check busy bit of register dsu_sta; if busy, wait    
    addis r6, r0, DSUTX_GO@h
    ori r6, r6, DSUTX_GO@l
    mtdcr DCRN_DSUCTRL, r6      // send tx command to register dsu_ctrl
    b 0

//
// test_ok: test complete successful, send out "OK"
//      
test_ok:
    addis r4, r0, ASCII_O@h
    ori r4, r4, ASCII_O@l       
    mtdcr DCRN_DSUTX, r4        // data ready
    mfdcr r5, DCRN_DSUSTA
    andi. r5, r5, DSUTX_BUSY
    bne (-8)                    // check busy bit of register dsu_sta; if busy, wait
    addis r6, r0, DSUTX_GO@h
    ori r6, r6, DSUTX_GO@l
    mtdcr DCRN_DSUCTRL, r6      // send tx command to register dsu_ctrl

    addis r4, r0, ASCII_K@h
    ori r4, r4, ASCII_K@l
    mtdcr DCRN_DSUTX, r4        // data ready    
    mfdcr r5, DCRN_DSUSTA
    andi. r5, r5, DSUTX_BUSY
    bne (-8)                    // check busy bit of register dsu_sta; if busy, wait
    addis r6, r0, DSUTX_GO@h
    ori r6, r6, DSUTX_GO@l
    mtdcr DCRN_DSUCTRL, r6      // send tx command to register dsu_ctrl

    //mfdcr r5, DCRN_DSUSTA
    //andi. r5, r5, DSURX_DONE
    //beq (-8)                    // check rx_done bit of register dsu_sta; if no rx, wait    
    //addis r7, r0, DSUIRT_CLEAR@h
    //ori r7, r7, DSUIRT_CLEAR@l
    //mtdcr DCRN_DSUCTRL, r7       // clear rx_done bit of register dsu_ctrl via clear interrupt
    
    //b test_ok       
    b 0

/////////////////////////////////////////////////////////////////////////////////
//
// exception verification routines
//
dbgexp_handler:

    // according to the exception vector and number
    // verify the register for exception processing
    // ...
    //bne test_fail        // branch to test_fail if not equal
    //blr                 // branch back to the original vector

    // there should be no exception at tests, if happened, send out "E"
    addis r4, r0, ASCII_E@h
    ori r4, r4, ASCII_E@l
    mtdcr DCRN_DSUTX, r4        // data ready
    mfdcr r5, DCRN_DSUSTA
    andi. r5, r5, DSUTX_BUSY
    bne (-8)                    // check busy bit of register dsu_sta; if busy, wait    
    addis r6, r0, DSUTX_GO@h
    ori r6, r6, DSUTX_GO@l
    mtdcr DCRN_DSUCTRL, r6      // send tx command to register dsu_ctrl
    b 0

//
// retarget the emulate ROM to 0xffff_fc00
//  you should compile twice to get the size of tests, then modify the space
//
    .space (0x1b14) 

// currently, there should be no un-support instruction executed by pippo, 
// if happened, send out "U" to ocnsole
emulate_rom:
    addis r4, r0, ASCII_U@h
    ori r4, r4, ASCII_U@l
    mtdcr DCRN_DSUTX, r4        // data ready
    mfdcr r5, DCRN_DSUSTA
    andi. r5, r5, DSUTX_BUSY
    bne (-8)                    // check busy bit of register dsu_sta; if busy, wait    
    addis r6, r0, DSUTX_GO@h
    ori r6, r6, DSUTX_GO@l
    mtdcr DCRN_DSUCTRL, r6      // send tx command to register dsu_ctrl
    b 0

//
// retarget the excpetion and reset vector to 0xffff_ff00
//
    .space (0x02d4) 

//
// exception vector at special mode: just an example for table
//
exp_vector:
// pippo_EXCEPT_CI - 0xff00 - critical
    ori r30, r0, 0xff00     // save the exception vector "0xff00" to r30
    ori r31, r0, 0x0000     // save the exception number "0" to r31
    bl dbgexp_handler       // branch to the ISR, save the snia to lr
    rfci                    // return

// pippo_EXCEPT_CHK - 0xff10 - critical
    ori r30, r0, 0xff10     // save the exception vector "0xff10" to r30
    ori r31, r0, 0x0001     // save the exception number "1" to r31
    bl dbgexp_handler
    rfci

// pippo_EXCEPT_DSI - 0xff20 (non-critical)
    ori r30, r0, 0xff20     // save the exception vector "0xff20" to r30
    ori r31, r0, 0x0002     // save the exception number "2" to r31
    bl dbgexp_handler
    rfi

// pippo_EXCEPT_ISI - 0xff30 (non-critical)
    ori r30, r0, 0xff30     // save the exception vector "0xff30" to r30
    ori r31, r0, 0x0003     // save the exception number "3" to r31
    bl dbgexp_handler
    rfi

// pippo_EXCEPT_EXT - 0xff40 (non-critical) 
    ori r30, r0, 0xff40     // save the exception vector "0xff40" to r30
    ori r31, r0, 0x0004     // save the exception number "4" to r31
    bl dbgexp_handler
    rfi

// pippo_EXCEPT_ALIGN - 0xff50 (non-critical)
    ori r30, r0, 0xff50     // save the exception vector "0xff50" to r30
    ori r31, r0, 0x0005     // save the exception number "5" to r31
    bl dbgexp_handler
    rfi

// pippo_EXCEPT_PRG - 0xff60 (non-critical)
    ori r30, r0, 0xff60     // save the exception vector "0xff60" to r30
    ori r31, r0, 0x0006     // save the exception number "6" to r31
    bl dbgexp_handler
    rfi

// pippo_EXCEPT_NOFPU - 0xff70 (non-critical)
    ori r30, r0, 0xff70     // save the exception vector "0xff70" to r30
    ori r31, r0, 0x0007     // save the exception number "7" to r31
    bl dbgexp_handler
    rfi

// pippo_EXCEPT_SYSCALL - 0xff80 (non-critical)
    ori r30, r0, 0xff80     // save the exception vector "0xff80" to r30
    ori r31, r0, 0x0008     // save the exception number "8" to r31
    bl dbgexp_handler
    rfi

// pippo_EXCEPT_NOAPU - 0xff90 (non-critical)
    ori r30, r0, 0xff90     // save the exception vector "0xff90" to r30
    ori r31, r0, 0x0009     // save the exception number "9" to r31
    bl dbgexp_handler
    rfi

// pippo_EXCEPT_PIT - 0xffa0 (non-critical)
    ori r30, r0, 0xffa0     // save the exception vector "0xffa0" to r30
    ori r31, r0, 0x000a     // save the exception number "a" to r31
    bl dbgexp_handler
    rfi

// pippo_EXCEPT_FIT - 0xffb0 (non-critical)
    ori r30, r0, 0xffb0     // save the exception vector "0xffb0" to r30
    ori r31, r0, 0x000b     // save the exception number "b" to r31
    bl dbgexp_handler
    rfi

// pippo_EXCEPT_WATCHDOG - 0xffc0  (critical)
    ori r30, r0, 0xffc0     // save the exception vector "0xffc0" to r30
    ori r31, r0, 0x000c     // save the exception number "c" to r31
    bl dbgexp_handler
    rfci

// pippo_EXCEPT_DTLBMISS - 0xffd0 (non-critical)
    ori r30, r0, 0xffd0     // save the exception vector "0xffd0" to r30
    ori r31, r0, 0x000d     // save the exception number "d" to r31
    bl dbgexp_handler
    rfi

// pippo_EXCEPT_ITLBMISS - 0xffe0 (non-critical)
    ori r30, r0, 0xffe0     // save the exception vector "0xffe0" to r30
    ori r31, r0, 0x000e     // save the exception number "e" to r31
    bl dbgexp_handler
    rfi

// pippo_EXCEPT_DEBUG - 0xfff0 (critical)
// notes: there are only three instructions for debug
    bl dbgexp_handler
    rfci
    nop                     // to verify the fetcher work
    
//
// reset vector should be located at 0xffff_fffc
//
    b core_initial

/////////////////////////////////////////////////////////////////////////////////
// 
// ram, located at start of uocm (currently 0xffff_c000), which includes:
//  zone 1, "write/read" section for run-time data storage
//  zone 2, "read" section for read only data, consume last 2KB (currently 0xffff_d800 - 0xffff_dfff)
//
.data

//
// zone 1
//
.space (0x1800) 

//
// zone 2
//
memory0: .long  0xffffd800,0xffffd804,0xffffd808,0xffffd80c,0xffffd810,0xffffd814,0xffffd818,0xffffd81c
memory1: .long  0x0000000f,0xfffffff0,0x000000ff,0xffffff00,0x0000ffff,0xffff0000,0xffffffff,0x00000000
memory2: .long  0x0000000f,0x000000f0,0x00000f00,0x0000f000,0x000f0000,0x00f00000,0x0f000000,0xf0000000
memory3: .long  0xfffffff0,0xffffff0f,0xfffff0ff,0xffff0fff,0xfff0ffff,0xff0fffff,0xf0ffffff,0x0fffffff,
memory4: .byte  0x62,0x61
         .align 2
memory5: .short 0xffff,0x1111
         .align 2
memory6: .short 0x7654,0x3456
memory7: .string "Hello, World"
         .align 2
memory8: .string "0123456789abcdefghijklmnopqrstuvwxyz"
         .align 2
memory9: .long  0xffffffff

//
// enlarge .data section to 0xffff_e000 - the start of .text
//  you should compile twice to get the size of tests, then modify the space
//  the value equals (0xcffc - addr of last data word)
//
.space (0x0734) 