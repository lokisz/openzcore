/*
 * File:        processor.h
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description:
 *      common header for powerpc architecture and implementation
 */

// Porting from u-boot

#ifndef __ASM_PPC_PROCESSOR_H
#define __ASM_PPC_PROCESSOR_H

/* register definition */
/* define names to make the asm easier to read - some compilers don't have this built in */
#define r0 0
#define r1 1
#define r2 2
#define r3 3
#define r4 4
#define r5 5
#define r6 6
#define r7 7
#define r8 8
#define r9 9
#define r10 10
#define r11 11
#define r12 12
#define r13 13
#define r14 14
#define r15 15
#define r16 16
#define r17 17
#define r18 18
#define r19 19
#define r20 20
#define r21 21
#define r22 22
#define r23 23
#define r24 24
#define r25 25
#define r26 26
#define r27 27
#define r28 28
#define r29 29
#define r30 30
#define r31 31

/* Special Purpose Registers (SPRNs)*/

#define SPRN_XER	0x001	/* Fixed Point Exception Register */
#define SPRN_LR		0x008	/* Link Register */
#define SPRN_CTR	0x009	/* Count Register */

#define SPRN_TBRL	0x10C	/* Time Base Read Lower Register */
#define SPRN_TBRU	0x10D	/* Time Base Read Upper Register */

#define SPRN_USPRG0	0x100	/* User Special Purpose Register General 0 */

#define SPRN_SPRG4R	0x104	/* Special Purpose Register General 4 Read */
#define SPRN_SPRG5R	0x105	/* Special Purpose Register General 5 Read */
#define SPRN_SPRG6R	0x106	/* Special Purpose Register General 6 Read */
#define SPRN_SPRG7R	0x107	/* Special Purpose Register General 7 Read */

#define SPRN_SPRG0	0x110	/* Special Purpose Register General 0 */
#define SPRN_SPRG1	0x111	/* Special Purpose Register General 1 */
#define SPRN_SPRG2	0x112	/* Special Purpose Register General 2 */
#define SPRN_SPRG3	0x113	/* Special Purpose Register General 3 */
#define SPRN_SPRG4W	0x114	/* Special Purpose Register General 4 Write */
#define SPRN_SPRG5W	0x115	/* Special Purpose Register General 5 Write */
#define SPRN_SPRG6W	0x116	/* Special Purpose Register General 6 Write */
#define SPRN_SPRG7W	0x117	/* Special Purpose Register General 7 Write */

//exception handling register
#define SPRN_SRR0	0x01A	/* Save/Restore Register 0 */
#define SPRN_SRR1	0x01B	/* Save/Restore Register 1 */
#define SPRN_SRR2	0x3DE	/* Save/Restore Register 2 */
#define SPRN_SRR3 	0x3DF	/* Save/Restore Register 3 */
#define SPRN_EVPR	0x3D6	/* Exception Vector Prefix Register */
#define SPRN_DEAR	0x3D5	/* Data Error Address Register */
#define SPRN_MCSR	0x23c	/* Machine Check Syndrome register */
#define SPRN_ESR	0x3D4	/* Exception Syndrome Register */

#define SPRN_TBHI	0x3DC	/* Time Base High */
#define SPRN_TBHU	0x3CC	/* Time Base High User-mode */
#define SPRN_TBLO	0x3DD	/* Time Base Low */
#define SPRN_TBLU	0x3CD	/* Time Base Low User-mode */
#define SPRN_TBWL	0x11C	/* Time Base Write Lower Register */
#define SPRN_TBWU	0x11D	/* Time Base Write Upper Register */
#define SPRN_PIT	0x3DB	/* Programmable Interval Timer */
#define SPRN_TCR	0x3DA	/* Timer Control Register */
#define SPRN_TSR	0x3D8	/* Timer Status Register */

/* DSU-uartlite DCRs */
#define DCRN_DSUTX 		0x350
#define DCRN_DSURX 		0x351

#define DCRN_DSUCTRL 	0x352
#define     DSUTX_GO	    0x00000001
#define     DSUIRT_CLEAR    0x00000010

#define DCRN_DSUSTA 	0x353
#define     DSUTX_BUSY	    0x00000001
#define     DSURX_DONE	    0x00000010

/************************************************************************************
 *                                                              
 * PWRSemi notes: defines following are NOT in use currently, just keep for reference
 *
 ************************************************************************************/
 
#define   ESR_IMCP	0x80000000	/* Instr. Machine Check - Protection */
#define   ESR_IMCN	0x40000000	/* Instr. Machine Check - Non-config */
#define   ESR_IMCB	0x20000000	/* Instr. Machine Check - Bus error */
#define   ESR_IMCT	0x10000000	/* Instr. Machine Check - Timeout */
#define   ESR_PIL	0x08000000	/* Program Exception - Illegal */
#define   ESR_PPR	0x04000000	/* Program Exception - Priveleged */
#define   ESR_PTR	0x02000000	/* Program Exception - Trap */
#define   ESR_DST	0x00800000	/* Storage Exception - Data miss */
#define   ESR_DIZ	0x00400000	/* Storage Exception - Zone fault */
 
#define   TCR_WP(x)		(((x)&0x3)<<30)	/* WDT Period */
#define     WP_2_17		0		/* 2^17 clocks */
#define     WP_2_21		1		/* 2^21 clocks */
#define     WP_2_25		2		/* 2^25 clocks */
#define     WP_2_29		3		/* 2^29 clocks */
#define   TCR_WRC(x)		(((x)&0x3)<<28)	/* WDT Reset Control */
#define     WRC_NONE		0		/* No reset will occur */
#define     WRC_CORE		1		/* Core reset will occur */
#define     WRC_CHIP		2		/* Chip reset will occur */
#define     WRC_SYSTEM		3		/* System reset will occur */
#define   TCR_WIE		0x08000000	/* WDT Interrupt Enable */
#define   TCR_PIE		0x04000000	/* PIT Interrupt Enable */
#define   TCR_FP(x)		(((x)&0x3)<<24)	/* FIT Period */
#define     FP_2_9		0		/* 2^9 clocks */
#define     FP_2_13		1		/* 2^13 clocks */
#define     FP_2_17		2		/* 2^17 clocks */
#define     FP_2_21		3		/* 2^21 clocks */
#define   TCR_FIE		0x00800000	/* FIT Interrupt Enable */
#define   TCR_ARE		0x00400000	/* Auto Reload Enable */

#define   TSR_ENW		0x80000000	/* Enable Next Watchdog */
#define   TSR_WIS		0x40000000	/* WDT Interrupt Status */
#define   TSR_WRS(x)		(((x)&0x3)<<28)	/* WDT Reset Status */
#define     WRS_NONE		0		/* No WDT reset occurred */
#define     WRS_CORE		1		/* WDT forced core reset */
#define     WRS_CHIP		2		/* WDT forced chip reset */
#define     WRS_SYSTEM		3		/* WDT forced system reset */
#define   TSR_PIS		0x08000000	/* PIT Interrupt Status */
#define   TSR_FIS		0x04000000	/* FIT Interrupt Status */

#define ESR_ST          0x00800000      /* Store Operation */

#define SPRN_PID	0x3B1	/* Process ID */
#define SPRN_PVR	0x11F	/* Processor Version Register */

/* Processor Version Register */

/* Processor Version Register (PVR) field extraction */

#define PVR_VER(pvr)  (((pvr) >>  16) & 0xFFFF)	/* Version field */
#define PVR_REV(pvr)  (((pvr) >>   0) & 0xFFFF)	/* Revison field */

/*
 * AMCC has further subdivided the standard PowerPC 16-bit version and
 * revision subfields of the PVR for the PowerPC 403s into the following:
 */

#define PVR_FAM(pvr)	(((pvr) >> 20) & 0xFFF)	/* Family field */
#define PVR_MEM(pvr)	(((pvr) >> 16) & 0xF)	/* Member field */
#define PVR_CORE(pvr)	(((pvr) >> 12) & 0xF)	/* Core field */
#define PVR_CFG(pvr)	(((pvr) >>  8) & 0xF)	/* Configuration field */
#define PVR_MAJ(pvr)	(((pvr) >>  4) & 0xF)	/* Major revision field */
#define PVR_MIN(pvr)	(((pvr) >>  0) & 0xF)	/* Minor revision field */

/* Processor Version Numbers */

#define PVR_403GA	0x00200000
#define PVR_403GB	0x00200100
#define PVR_403GC	0x00200200
#define PVR_403GCX	0x00201400
#define PVR_405GP	0x40110000
#define PVR_405GP_RB	0x40110040
#define PVR_405GP_RC	0x40110082
#define PVR_405GP_RD	0x401100C4
#define PVR_405GP_RE	0x40110145  /* same as pc405cr rev c */
#define PVR_405CR_RA	0x40110041
#define PVR_405CR_RB	0x401100C5
#define PVR_405CR_RC	0x40110145  /* same as pc405gp rev e */
#define PVR_405EP_RA	0x51210950
#define PVR_405GPR_RB	0x50910951
#define PVR_440GP_RB	0x40120440
#define PVR_440GP_RC	0x40120481
#define PVR_440EP_RA	0x42221850
#define PVR_440EP_RB	0x422218D3 /* 440EP rev B and 440GR rev A have same PVR */
#define PVR_440GR_RA	0x422218D3 /* 440EP rev B and 440GR rev A have same PVR */
#define PVR_440GX_RA	0x51B21850
#define PVR_440GX_RB	0x51B21851
#define PVR_440GX_RC	0x51B21892
#define PVR_440GX_RF	0x51B21894
#define PVR_405EP_RB	0x51210950
#define PVR_440SP_RA	0x53221850
#define PVR_440SP_RB	0x53221891
#define PVR_601		0x00010000
#define PVR_602		0x00050000
#define PVR_603		0x00030000
#define PVR_603e	0x00060000
#define PVR_603ev	0x00070000
#define PVR_603r	0x00071000
#define PVR_604		0x00040000
#define PVR_604e	0x00090000
#define PVR_604r	0x000A0000
#define PVR_620		0x00140000
#define PVR_740		0x00080000
#define PVR_750		PVR_740
#define PVR_740P	0x10080000
#define PVR_750P	PVR_740P
#define PVR_7400        0x000C0000
#define PVR_7410        0x800C0000
#define PVR_7450        0x80000000

/* Machine State Register (MSR) Fields */

#define MSR_UCLE    (1<<26)     /* User-mode cache lock enable (e500) */
#define MSR_VEC		(1<<25)		/* Enable AltiVec(74xx) */
#define MSR_SPE     (1<<25)     /* Enable SPE(e500) */
#define MSR_POW		(1<<18)		/* Enable Power Management */
#define MSR_WE		(1<<18)		/* Wait State Enable */
#define MSR_TGPR	(1<<17)		/* TLB Update registers in use */
#define MSR_CE		(1<<17)		/* Critical Interrupt Enable */
#define MSR_ILE		(1<<16)		/* Interrupt Little Endian */
#define MSR_EE		(1<<15)		/* External Interrupt Enable */
#define MSR_PR		(1<<14)		/* Problem State / Privilege Level */
#define MSR_FP		(1<<13)		/* Floating Point enable */
#define MSR_ME		(1<<12)		/* Machine Check Enable */
#define MSR_FE0		(1<<11)		/* Floating Exception mode 0 */
#define MSR_SE		(1<<10)		/* Single Step */
#define MSR_DWE     (1<<10)     /* Debug Wait Enable (4xx) */
#define MSR_UBLE    (1<<10)     /* BTB lock enable (e500) */
#define MSR_BE		(1<<9)		/* Branch Trace */
#define MSR_DE		(1<<9) 		/* Debug Exception Enable */
#define MSR_FE1		(1<<8)		/* Floating Exception mode 1 */
#define MSR_IP		(1<<6)		/* Exception prefix 0x000/0xFFF */
#define MSR_IR		(1<<5) 		/* Instruction Relocate */
#define MSR_IS      (1<<5)      /* Book E Instruction space */
#define MSR_DR		(1<<4) 		/* Data Relocate */
#define MSR_DS      (1<<4)      /* Book E Data space */
#define MSR_PE		(1<<3)		/* Protection Enable */
#define MSR_PX		(1<<2)		/* Protection Exclusive Mode */
#define MSR_PMM     (1<<2)      /* Performance monitor mark bit (e500) */
#define MSR_RI		(1<<1)		/* Recoverable Exception */
#define MSR_LE		(1<<0) 		/* Little Endian */


#endif

