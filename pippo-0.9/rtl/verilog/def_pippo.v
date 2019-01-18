/*
 * File:        def_pippo.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description:
 *      Part 1: special defines for development
 *      Part 2: design-specific configuration
 *      Part 3: implementation specific configuration
 *      Part 4: general definition for PWR and implementations.
 *      Part 5: system configuration
 */

///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Part 1: special defines for development
//

// Dump VCD
//`define pippo_VCD_DUMP

// Generate debug messages during simulation
//`define pippo_VERBOSE


///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Part 2: design-specific configuration
//

//
// for ASIC implementation
//
//`define pippo_ASIC

//
// for FPGA implementation
//
//`define pippo_FPGA


// d$ configuration
//`define pippo_DC_ENABLE

// i$ configuration
//`define pippo_IC_ENABLE
    
// i-mmu 
//`define pippo_IMMU_ENABLE
    
// d-mmu
//`define pippo_DMMU_ENABLE
    
//
// inst/data-ocm word bandwidth of address
//  capacity of ocm: 2**(IOCM_Word_BW+2-10) kilo-byte
//      9  equals 2KB
//      10 equals 4KB
//      ...
`define IOCM_Word_BW        12      // Word address
`define DOCM_Word_BW        12      // Word address

`define UOCM_Word_BW        12      // Word address

`define pippo_IMC_MASK	        32'hffff_c000
`define pippo_IMC_ADDR	        32'hffff_c000
`define pippo_DMC_MASK	        32'hffff_c000
`define pippo_DMC_ADDR	        32'hffff_c000

// multiplier
`define pippo_MULT_IMPLEMENTED

//
// Register pippo WISHBONE outputs
// (must be defined/enabled)
//
`define pippo_REGISTERED_OUTPUTS

//
// Interconnection
//
// WISHBONE B3 compatible interface
//`define pippo_WB_B3


///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Part 3: implementation specific configuration
//

//
// reset
//
`define pippo_RST_EVENT     posedge
`define pippo_RST_VALUE     1'b1

//
// Enables default statement in some case blocks
// and disables Synopsys synthesis directive full_case
//
// By default it is enabled. When disabled it
// can increase clock frequency.
//
`define pippo_CASE_DEFAULT

//
// Clock ratio between Core/Interconnection
//

// support CBU:Core 1:2
`define pippo_CLKDIV_2_SUPPORTED
// support CBU:Core 1:4
//`define pippo_CLKDIV_4_SUPPORTED

//
// dsu uartlite
//
//`define FPGA_50MHZ    1
//`define FPGA_32MHZ    1
`define FPGA_64MHZ      1
`define BIGMODEL	    1
`define dsu_RST_EVENT   posedge
`define dsu_RST_VALUE   1'b1


///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Part 4: general definition for PWR architecture 
//          and implementations. 
//  Note: User CAN'T CHANGE
//

// instruction:
//      ori 0,0,0;  32'h60000_0000
// is defined as NOP at PWR architecture
`define pippo_PWR_NOP           32'b0110_0000_0000_0000_0000_0000_0000_0000
//
// Operand width / register file address width
//
`define OPERAND_WIDTH		32
`define GPR_ADDR_WIDTH	    5
`define SPR_ADDR_WIDTH	    10

//////////////////////////////////////////////
//
// PWR ISA and uops
//

//
// Binary compatiable level
//
//`define BINARY_401
`define BINARY_405
//`define BINARY_440
//`define BINARY_460
//`define BINARY_476
//`define BINARY_603e
//`define BINARY_750
//`define BINARY_7410
//`define BINARY_e200
//`define BINARY_e500

// Instruction Form
`define OPCD_width   [31:26]
`define XO_width     [10:1]     // X form have 9 bits only

//
// PWR ISA primary opcode (OPCD) and secondary opcode(XO)
// notes: 
//  1, where lower-case x means: 
//      [o]: OE field, to enable setting OV and SO in the XER
//      [.]: Rc field, record bit to record CR
//      and for branch instructions:
//      [l]: LK field, link bit - 1 means update LR
//      [a]: AA field, absolute address bit - 1 means 
//  2, Different form instructions have same OPCODE, with different XO field width
//      X-form have 10-bit width XO field
//      XO-form have 9-bit width XO filed
//
`ifdef BINARY_405
    `define TWI_OPCD                6'b000011  
    `define MULHHWUx_OPCD           6'b000100  
        `define MULHHWUx_XO           10'd8
    `define MULHHWx_OPCD            6'b000100
        `define MULHHWx_XO            10'd40
    `define MULCHWUx_OPCD           6'b000100
        `define MULCHWUx_XO           10'd136  
    `define MULCHWx_OPCD            6'b000100
        `define MULCHWx_XO            10'd168
    `define MULLHWUx_OPCD           6'b000100
        `define MULLHWUx_XO           10'd392 
    `define MULLHWx_OPCD            6'b000100
        `define MULLHWx_XO            10'd424
    `define MACHHWUx_OPCD           6'b000100
        `define MACHHWUx_XO           9'd12  // 524
    `define MACHHWx_OPCD            6'b000100
        `define MACHHWx_XO            9'd44  // 556
    `define NMACHHWx_OPCD           6'b000100
        `define NMACHHWx_XO           9'd46  // 558
    `define MACHHWSUx_OPCD          6'b000100
        `define MACHHWSUx_XO          9'd76  // 588
    `define MACHHWSx_OPCD           6'b000100
        `define MACHHWSx_XOD          9'd108 // 620
    `define NMACHHWSx_OPCD          6'b000100
        `define NMACHHWSx_XO          9'd110 // 622
    `define MACCHWUx_OPCD           6'b000100
        `define MACCHWUx_XO           9'd140 // 652  
    `define MACCHWx_OPCD            6'b000100
        `define MACCHWx_XO            9'd172 // 684  
    `define NMACCHWUx_OPCD          6'b000100
        `define NMACCHWUx_XO          9'd174 // 686  
    `define MACCHWSUx_OPCD          6'b000100
        `define MACCHWSUx_XO          9'd204 // 716
    `define MACCHWSx_OPCD           6'b000100
        `define MACCHWSx_XO           9'd236 // 748  
    `define NMACCHWSx_OPCD          6'b000100
        `define NMACCHWSx_XO          9'd238 // 750          
    `define MACCLHWUx_OPCD          6'b000100
        `define MACCLHWUx_XO          9'd396 // 908                 
    `define MACLHWx_OPCD            6'b000100
        `define MACLHWx_XO            9'd428 // 940
    `define NMACLHWx_OPCD           6'b000100
        `define NMACLHWx_XO           9'd430 // 942
    `define MACLHWSx_OPCD           6'b000100
        `define MACLHWSx_XO           9'd492 // 972
    `define MACLHWSUx_OPCD          6'b000100
        `define MACLHWSUx_XO          9'd460 // 1004
    `define NMACLHWSx_OPCD          6'b000100
        `define NMACLHWSx_XO          9'd494 // 1006                
    `define MULLI_OPCD              6'b000111
    `define SUBFIC_OPCD             6'b001000
    `define CMPLI_OPCD              6'b001010
    `define CMPI_OPCD               6'b001011
    `define ADDIC_OPCD              6'b001100
    `define ADDICx_OPCD             6'b001101    
    `define ADDI_OPCD               6'b001110
    `define ADDIS_OPCD              6'b001111
    `define BCx_OPCD                6'b010000
    `define SC_OPCD                 6'b010001
    `define Bx_OPCD                 6'b010010
    `define MCRF_OPCD               6'b010011
        `define MCRF_XO                 10'd0
    `define BCLRx_OPCD              6'b010011
        `define BCLRx_XO                10'd16
    `define CRNOR_OPCD              6'b010011
        `define CRNOR_XO                10'd33
    `define RFI_OPCD                6'b010011
        `define RFI_XO                  10'd50
    `define RFCI_OPCD               6'b010011
        `define RFCI_XO                 10'd51
    `define CRANDC_OPCD             6'b010011
        `define CRANDC_XO               10'd129
    `define ISYNC_OPCD              6'b010011
        `define ISYNC_XO                10'd150
    `define CRXOR_OPCD              6'b010011
        `define CRXOR_XO                10'd193
    `define CRNAND_OPCD             6'b010011
        `define CRNAND_XO               10'd225
    `define CRAND_OPCD              6'b010011
        `define CRAND_XO                10'd257
    `define CREQV_OPCD              6'b010011
        `define CREQV_XO                10'd289
    `define CRORC_OPCD              6'b010011
        `define CRORC_XO                10'd417
    `define CROR_OPCD               6'b010011
        `define CROR_XO                 10'd449
    `define BCCTRx_OPCD             6'b010011
        `define BCCTRx_XO               10'd528
    `define RLWIMIx_OPCD            6'b010100
    `define RLWINMx_OPCD            6'b010101
    `define RLWNMx_OPCD             6'b010111
    `define ORI_OPCD                6'b011000
    `define ORIS_OPCD               6'b011001
    `define XORI_OPCD               6'b011010
    `define XORIS_OPCD              6'b011011
    `define ANDIx_OPCD              6'b011100
    `define ANDISx_OPCD             6'b011101
    `define CMP_OPCD                6'b011111
        `define CMP_XO                  10'd0
    `define TW_OPCD                 6'b011111
        `define TW_XO                   10'd4
    `define SUBFCx_OPCD             6'b011111
        `define SUBFCx_XO               9'd8 //520
    `define ADDCx_OPCD              6'b011111
        `define ADDCx_XO                9'd10 //522
    `define MULHWUx_OPCD            6'b011111
        `define MULHWUx_XO              9'd11
    `define MFCR_OPCD               6'b011111
        `define MFCR_XO                 10'd19
    `define LWARX_OPCD              6'b011111
        `define LWARX_XO                10'd20
    `define LWZX_OPCD               6'b011111
        `define LWZX_XO                 10'd23
    `define SLWx_OPCD               6'b011111
        `define SLWx_XO                 10'd24
    `define CNTLZWx_OPCD            6'b011111
        `define CNTLZWx_XO              10'd26
    `define ANDx_OPCD               6'b011111
        `define ANDx_XO                 10'd28
    `define CMPL_OPCD               6'b011111
        `define CMPL_XO                 10'd32
    `define SUBFx_OPCD              6'b011111
        `define SUBFx_XO                9'd40 //552
    `define DCBST_OPCD              6'b011111
        `define DCBST_XO                10'd54
    `define LWZUX_OPCD              6'b011111
        `define LWZUX_XO                10'd55
    `define ANDCx_OPCD              6'b011111    
        `define ANDCx_XO                10'd60
    `define MULHWx_OPCD             6'b011111    
        `define MULHWx_XO               9'd75
    `define MFMSR_OPCD              6'b011111
        `define MFMSR_XO                10'd83
    `define DCBF_OPCD               6'b011111
        `define DCBF_XO                 10'd86
    `define LBZX_OPCD               6'b011111
        `define LBZX_XO                 10'd87
    `define NEGx_OPCD               6'b011111
        `define NEGx_XO                 9'd104 //616
    `define LBZUX_OPCD              6'b011111
        `define LBZUX_XO                10'd119
    `define NORx_OPCD               6'b011111
        `define NORx_XO                 10'd124
    `define WRTEE_OPCD              6'b011111
        `define WRTEE_XO                10'd131
    `define SUBFEx_OPCD             6'b011111
        `define SUBFEx_XO               9'd136 //648
    `define ADDEx_OPCD              6'b011111
        `define ADDEx_XO                9'd138 //650
    `define MTCRF_OPCD              6'b011111
        `define MTCRF_XO                10'd144
    `define MTMSR_OPCD              6'b011111
        `define MTMSR_XO                10'd146
    `define STWCXx_OPCD             6'b011111
        `define STWCXx_XO               10'd150
    `define STWX_OPCD               6'b011111
        `define STWX_XO                 10'd151
    `define WRTEEI_OPCD             6'b011111
        `define WRTEEI_XO               10'd163
    `define STWUX_OPCD              6'b011111
        `define STWUX_XO                10'd183
    `define SUBFZEx_OPCD            6'b011111
        `define SUBFZEx_XO              9'd200 //712
    `define ADDZEx_OPCD             6'b011111
        `define ADDZEx_XO               9'd202 //714
    `define STBX_OPCD               6'b011111
        `define STBX_XO                 10'd215
    `define SUBFMEx_OPCD            6'b011111
        `define SUBFMEx_XO              9'd232 //744
    `define ADDMEx_OPCD             6'b011111
        `define ADDMEx_XO               9'd234 //746
    `define MULLWx_OPCD             6'b011111
        `define MULLWx_XO               9'd235 //747
    `define DCBTST_OPCD             6'b011111
        `define DCBTST_XO               10'd246 
    `define STBUX_OPCD              6'b011111
        `define STBUX_XO                10'd247
    `define ICBT_OPCD               6'b011111
        `define ICBT_XO                 10'd262
    `define ADDx_OPCD               6'b011111
        `define ADDx_XO                 9'd266 //778
    `define DCBT_OPCD               6'b011111
        `define DCBT_XO                 10'd278 
    `define LHZX_OPCD               6'b011111
        `define LHZX_XO                 10'd279
    `define EQVx_OPCD               6'b011111
        `define EQVx_XO                 10'd284
    `define LHZUX_OPCD              6'b011111
        `define LHZUX_XO                10'd311
    `define XORx_OPCD               6'b011111
        `define XORx_XO                 10'd316
    `define MFDCR_OPCD              6'b011111
        `define MFDCR_XO                10'd323
    `define MFSPR_OPCD              6'b011111
        `define MFSPR_XO                10'd339
    `define LHAX_OPCD               6'b011111
        `define LHAX_XO                 10'd343
    `define TLBIA_OPCD              6'b011111
        `define TLBIA_XO                10'd370
    `define MFTB_OPCD               6'b011111
        `define MFTB_XO                 10'd371
    `define LHAUX_OPCD              6'b011111
        `define LHAUX_XO                10'd375
    `define STHX_OPCD               6'b011111
        `define STHX_XO                 10'd407
    `define ORCx_OPCD               6'b011111
        `define ORCx_XO                 10'd412
    `define STHUX_OPCD              6'b011111
        `define STHUX_XO                10'd439
    `define ORx_OPCD                6'b011111
        `define ORx_XO                  10'd444
    `define MTDCR_OPCD              6'b011111
        `define MTDCR_XO                10'd451
    `define DCCCI_OPCD              6'b011111
        `define DCCCI_XO                10'd454
    `define DIVWUx_OPCD             6'b011111
        `define DIVWUx_XO               9'd459 //971
    `define MTSPR_OPCD              6'b011111
        `define MTSPR_XO                10'd467
    `define DCBI_OPCD               6'b011111
        `define DCBI_XO                 10'd470
    `define NANDx_OPCD              6'b011111
        `define NANDx_XO                10'd476
    `define DCREAD_OPCD             6'b011111
        `define DCREAD_XO               10'd486
    `define DIVWx_OPCD              6'b011111
        `define DIVWx_XO                9'd491 //1003
    `define MCRXR_OPCD              6'b011111
        `define MCRXR_XO                10'd512
    `define LSWX_OPCD               6'b011111
        `define LSWX_XO                 10'd533
    `define LWBRX_OPCD              6'b011111
        `define LWBRX_XO                10'd534
    `define SRWx_OPCD               6'b011111
        `define SRWx_XO                 10'd536
    `define TLBSYNC_OPCD            6'b011111
        `define TLBSYNC_XO              10'd566
    `define LSWI_OPCD               6'b011111
        `define LSWI_XO                 10'd597
    `define SYNC_OPCD               6'b011111
        `define SYNC_XO                 10'd598
    `define STSWX_OPCD              6'b011111
        `define STSWX_XO                10'd661
    `define STWBRX_OPCD             6'b011111
        `define STWBRX_XO               10'd662
    `define STSWI_OPCD              6'b011111
        `define STSWI_XO                10'd725
    `define DCBA_OPCD               6'b011111
        `define DCBA_XO                 10'd758
    `define LHBRX_OPCD              6'b011111
        `define LHBRX_XO                10'd790
    `define SRAWx_OPCD              6'b011111
        `define SRAWx_XO                10'd792
    `define SRAWIx_OPCD             6'b011111
        `define SRAWIx_XO               10'd824
    `define EIEIO_OPCD              6'b011111
        `define EIEIO_XO                10'd854
    `define TLBSXx_OPCD             6'b011111
        `define TLBSXx_XO               10'd914
    `define STHBRX_OPCD             6'b011111
        `define STHBRX_XO               10'd918
    `define EXTSHx_OPCD             6'b011111
        `define EXTSHx_XO               10'd922
    `define TLBRE_OPCD              6'b011111
        `define TLBRE_XO                10'd946
    `define EXTSBx_OPCD             6'b011111
        `define EXTSBx_XO               10'd954
    `define ICCCI_OPCD              6'b011111
        `define ICCCI_XO                10'd966
    `define TLBWE_OPCD              6'b011111
        `define TLBWE_XO                10'd978
    `define ICBI_OPCD               6'b011111
        `define ICBI_XO                 10'd982
    `define ICREAD_OPCD             6'b011111
        `define ICREAD_XO               10'd998
    `define DCBZ_OPCD               6'b011111
        `define DCBZ_XO                 10'd1014
    `define LWZ_OPCD                6'b100000
    `define LWZU_OPCD               6'b100001
    `define LBZ_OPCD                6'b100010
    `define LBZU_OPCD               6'b100011
    `define STW_OPCD                6'b100100
    `define STWU_OPCD               6'b100101
    `define STB_OPCD                6'b100110
    `define STBU_OPCD               6'b100111
    `define LHZ_OPCD                6'b101000
    `define LHZU_OPCD               6'b101001
    `define LHA_OPCD                6'b101010
    `define LHAU_OPCD               6'b101011
    `define STH_OPCD                6'b101100
    `define STHU_OPCD               6'b101101
    `define LMW_OPCD                6'b101110
    `define STMW_OPCD               6'b101111
`endif  // `define BINARY_405

///////////////////////////////////////////////////////
//
// decoded uops and execution op
// notes: At current implementation(pippo), uops 
//        include execution op and some bits for 
//        operand selection or additional write-back 
//        information

//
// ALUOPs
//

// alu_uops = {OE, Rc, alu_op};
`define ALUUOPS_WIDTH	8
`define ALUOP_WIDTH	    6

`define ALUOP_NOP	    6'b000000

// Logic
`define ALUOP_AND	    6'b000001
`define ALUOP_ANDC	    6'b000010
`define ALUOP_NAND	    6'b000011
`define ALUOP_NOR	    6'b000100
`define ALUOP_OR	    6'b000101
`define ALUOP_ORC	    6'b000110
`define ALUOP_XOR	    6'b000111
`define ALUOP_EQV       6'b001000
`define ALUOP_NEG       6'b001001

// Arithmetic
`define ALUOP_ADD	    6'b001010
`define ALUOP_ADDC	    6'b001011
`define ALUOP_ADDE      6'b001100
`define ALUOP_SUBF	    6'b001101
`define ALUOP_SUBFC     6'b001110
`define ALUOP_SUBFE     6'b001111
`define ALUOP_MULHW     6'b010000
`define ALUOP_MULLI     6'b010010
`define ALUOP_MULLW     6'b010011
`define ALUOP_MULHWU    6'b010100
//`define ALUOP_DIVW    6'b000000
//`define ALUOP_DIVWU	6'b000000


// Shift & Rotate
// bit[5:4]: shrot
// bit[3]: rotate
// bit[1]: right
// bit[0]: algrithm
`define ALUOP_SLW       6'b110000
`define ALUOP_SRW       6'b110010
`define ALUOP_SRAW      6'b110011
`define ALUOP_SRAWI     6'b110111
`define ALUOP_RLWIMI    6'b111000
`define ALUOP_RLWNM     6'b111100
`define ALUOP_RLWINM    6'b111110

// Misc
`define ALUOP_EXTSB     6'b100000
`define ALUOP_EXTSH     6'b100001
`define ALUOP_CNTLZW    6'b100010

//
// LSUOPs
// Bit 4: update form
// Bit 3: 0 load, 1 store
// Bits 2-1: 00 doubleword, 01 byte, 10 halfword, 11 singleword
// Bit 0: sign extend
// list2do: update, byte-reverse...
`define LSUUOPS_WIDTH	5
`define LSUOP_WIDTH		4

`define LSUOP_NOP		4'b0000
`define LSUOP_LBZ		4'b0001
`define LSUOP_LHZ		4'b0010
`define LSUOP_LHA		4'b0011
`define LSUOP_LWZ		4'b0100
`define LSUOP_LHZB		4'b0110
`define LSUOP_LWZB		4'b0111
`define LSUOP_STB		4'b1000
`define LSUOP_STH		4'b1001
`define LSUOP_STW		4'b1010
`define LSUOP_STHB		4'b1100
`define LSUOP_STWB		4'b1110

//
// BPU uops
//

//bpu_uops = {AA, LK, bpu_op}
`define BPUUOPS_WIDTH	5
`define BPUOP_WIDTH		3

`define BPUOP_NOP       3'b000
`define BPUOP_BIMM      3'b101
`define BPUOP_BCIMM     3'b001
`define BPUOP_BCLR      3'b010
`define BPUOP_BCCTR     3'b011
   
//
// CR op
//
`define CRUOPS_WIDTH   4
`define CROP_WIDTH     4

`define CROP_NOP       4'b0000
`define CROP_AND       4'b0001
`define CROP_ANDC      4'b0010
`define CROP_EQV       4'b0011
`define CROP_NAND      4'b0100
`define CROP_NOR       4'b0101
`define CROP_OR        4'b0110
`define CROP_XOR       4'b1000
`define CROP_ORC       4'b1001
`define CROP_CMPL      4'b1010
`define CROP_CMP       4'b1100      // (cr_uops[3] & cr_uops[2]) assertion(1'b1) means cmp, (1'b0 means cmpl)
`define CROP_TRAP      4'b1111      // for twi/tw compare

//
// SPRs op
//
`define REGUOPS_WIDTH   4
`define REGOP_WIDTH     4

`define REGOP_NOP       4'b0000
`define REGOP_MFSPR     4'b0001
`define REGOP_MTSPR     4'b0010
`define REGOP_MFMSR     4'b0011
`define REGOP_MTMSR     4'b0100
`define REGOP_MFCR      4'b0101
`define REGOP_MTCRF     4'b0110
`define REGOP_MCRXR     4'b0111
`define REGOP_MCRF      4'b1000
`define REGOP_WRTEE     4'b1001

//
// Register File Write-Back OPs
//
// Bit 0: rf write port-a enable
// Bit 1: rf write port-b enable
// Bits 3-2: Write-back Muxes
`define RFWBUOPS_WIDTH	4
`define RFWBOP_WIDTH	4

`define RFWBOP_NOP		4'b0000
`define RFWBOP_ALU		4'b0001
`define RFWBOP_SPRS		4'b1001
`define RFWBOP_LSU		4'b0101
`define RFWBOP_LSUEA	4'b1110 //[TBD]
`define RFWBOP_LSUTWO	4'b1111

//
// Execution cycles per instruction
//
`define MULTICYCLE_WIDTH	    2

`define EXTEND_ZERO_CYCLES		    2'd0
`define EXTEND_ONE_CYCLES		    2'd1
`define EXTEND_TWO_CYCLES		    2'd2
`define EXTEND_THREE_CYCLES		    2'd3

//
// Operand Muxes
//
`define OPSEL_WIDTH		    2

`define OPSEL_RF			2'd0
`define OPSEL_IMM			2'd1
`define OPSEL_WBFWD 		2'd2

/////////////////////////////////////////////////////
//
// Exceptions
//

// Exception vectors N: "0xPPPPVVVV", where 
//  P represents exception prefix, which come from EVPR register
//  V represents length of the individual vector space,

//
// N part width
//
`define pippo_EXCEPT_WIDTH  16

//
// Definition of exception type/vectors
//      To avoid implementation of a certain exception,
//      simply comment out corresponding line
//
`define BINARY_405_EXP_Special

`define pippo_EXCEPT_NONE   `pippo_EXCEPT_WIDTH'hfff8
    
`ifdef BINARY_405_EXP
    `define pippo_RESET_VECTOR      32'hffff_fffc
    `define pippo_EXCEPT_CI 		`pippo_EXCEPT_WIDTH'h0100
    `define pippo_EXCEPT_CHK		`pippo_EXCEPT_WIDTH'h0200
    `define pippo_EXCEPT_DSI		`pippo_EXCEPT_WIDTH'h0300
    `define pippo_EXCEPT_ISI		`pippo_EXCEPT_WIDTH'h0400
    `define pippo_EXCEPT_EXT		`pippo_EXCEPT_WIDTH'h0500
    `define pippo_EXCEPT_ALIGN		`pippo_EXCEPT_WIDTH'h0600
    `define pippo_EXCEPT_PRG		`pippo_EXCEPT_WIDTH'h0700
    `define pippo_EXCEPT_NOFPU     	`pippo_EXCEPT_WIDTH'h0800
    `define pippo_EXCEPT_SYSCALL	`pippo_EXCEPT_WIDTH'h0C00
    `define pippo_EXCEPT_NOAPU  	`pippo_EXCEPT_WIDTH'h0F20
    `define pippo_EXCEPT_PIT		`pippo_EXCEPT_WIDTH'h1000
    `define pippo_EXCEPT_FIT		`pippo_EXCEPT_WIDTH'h1010
    `define pippo_EXCEPT_WATCHDOG	`pippo_EXCEPT_WIDTH'h1020
    `define pippo_EXCEPT_DTLBMISS	`pippo_EXCEPT_WIDTH'h1100
    `define pippo_EXCEPT_ITLBMISS	`pippo_EXCEPT_WIDTH'h1200
    `define pippo_EXCEPT_DEBUG  	`pippo_EXCEPT_WIDTH'h2000
`endif

// special mode for minimal system configuration and system debug
`ifdef BINARY_405_EXP_Special
    `define pippo_RESET_VECTOR      32'hffff_fffc
    `define pippo_EXCEPT_CI 		`pippo_EXCEPT_WIDTH'hff00
    `define pippo_EXCEPT_CHK		`pippo_EXCEPT_WIDTH'hff10
    `define pippo_EXCEPT_DSI		`pippo_EXCEPT_WIDTH'hff20
    `define pippo_EXCEPT_ISI		`pippo_EXCEPT_WIDTH'hff30
    `define pippo_EXCEPT_EXT		`pippo_EXCEPT_WIDTH'hff40
    `define pippo_EXCEPT_ALIGN		`pippo_EXCEPT_WIDTH'hff50
    `define pippo_EXCEPT_PRG		`pippo_EXCEPT_WIDTH'hff60
    `define pippo_EXCEPT_NOFPU     	`pippo_EXCEPT_WIDTH'hff70
    `define pippo_EXCEPT_SYSCALL	`pippo_EXCEPT_WIDTH'hff80
    `define pippo_EXCEPT_NOAPU  	`pippo_EXCEPT_WIDTH'hff90
    `define pippo_EXCEPT_PIT		`pippo_EXCEPT_WIDTH'hffa0
    `define pippo_EXCEPT_FIT		`pippo_EXCEPT_WIDTH'hffb0
    `define pippo_EXCEPT_WATCHDOG	`pippo_EXCEPT_WIDTH'hffc0
    `define pippo_EXCEPT_DTLBMISS	`pippo_EXCEPT_WIDTH'hffd0
    `define pippo_EXCEPT_ITLBMISS	`pippo_EXCEPT_WIDTH'hffe0
    `define pippo_EXCEPT_DEBUG  	`pippo_EXCEPT_WIDTH'hfff0
`endif

`ifdef BINARY_401
    `define pippo_EXCEPT_RESET		`pippo_EXCEPT_WIDTH'h0100
    `define pippo_EXCEPT_CHK		`pippo_EXCEPT_WIDTH'h0200
    `define pippo_EXCEPT_DSI		`pippo_EXCEPT_WIDTH'h0300
    `define pippo_EXCEPT_ISI		`pippo_EXCEPT_WIDTH'h0400
    `define pippo_EXCEPT_EXT		`pippo_EXCEPT_WIDTH'h0500
    `define pippo_EXCEPT_ALIGN		`pippo_EXCEPT_WIDTH'h0600
    `define pippo_EXCEPT_SYSCALL	`pippo_EXCEPT_WIDTH'h0C00
    `define pippo_EXCEPT_PIT		`pippo_EXCEPT_WIDTH'h1000
    `define pippo_EXCEPT_FIT		`pippo_EXCEPT_WIDTH'h1010
    `define pippo_EXCEPT_WATCHDOG	`pippo_EXCEPT_WIDTH'h1020
    `define pippo_EXCEPT_DTLBMISS	`pippo_EXCEPT_WIDTH'h1100
    `define pippo_EXCEPT_ITLBMISS	`pippo_EXCEPT_WIDTH'h1200
    `define pippo_EXCEPT_DEBUG  	`pippo_EXCEPT_WIDTH'h2000
`endif

///////////////////////////////////////////////////////
//
// Register sets
//  Note: CR and MSR are not mapped to SPR address
//

//
// Machine State Register (Supervisor state)
//
//      [31:26], [24:20], [16],[7:6].[3:0]: Reserved
//      [25]-AP: Auxiliary Processor Available
//          0 APU not available.
//          1 APU available.
//      [19]-APE: APU Exception Enable
//          0 APU exception disabled.
//          1 APU exception enabled.
//      [18]-WE: Wait State Enable
//          0 The processor is not in the wait state.
//          1 The processor is in the wait state.
//          If MSR[WE] = 1, the processor remains in the wait state until an interrupt is taken, a reset occurs, or an external debug tool clears WE
//      [17]-CE: Critical Interrupt Enable
//          0 Critical interrupts are disabled.
//          1 Critical interrupts are enabled
//          Controls the critical interrupt input and watchdog timer first time-out interrupts.
//      [15]-EE: External Interrupt Enable
//          0 Asynchronous interruptsare disabled.
//          1 Asynchronous interrupts are enabled.
//          Controls the non-critical external interrupt input, PIT, and FIT interrupts.
//      [14]-PR: Problem State
//          0 Supervisor state (all instructions allowed).
//          1 Problem state (some instructions not allowed).
//      [13]-FP: Floating Point Available
//      [12]-ME: Machine Check Enable
//          0 Machine check interrupts are disabled.
//          1 Machine check interrupts are enabled.
//      [11]-FE0: Floating-point exception mode 0
//      [10]-DWE: Debug Wait Enable
//          0 Debug wait mode is disabled.
//          1 Debug wait mode is enabled.
//      [9]-DE: Debug Interrupts Enable
//          0 Debug interrupts are disabled.
//          1 Debug interrupts are enabled.
//      [8]-FE1: Floating-point exception mode 1
//      [5]-IR: Instruction Relocate
//          0 Instruction address translation is disabled.
//          1 Instruction address translation is enabled.
//      [4]-DR: Data Relocate
//          0 Data address translation is disabled.
//          1 Data address translation is enabled.  
          
`define pippo_MSR_RESET     32'd0

// Bit Fields
`define pippo_MSR_AP_BITS    25   
`define pippo_MSR_APE_BITS   19
`define pippo_MSR_WE_BITS    18
`define pippo_MSR_CE_BITS    17
`define pippo_MSR_EE_BITS    15
`define pippo_MSR_PR_BITS    14
`define pippo_MSR_FP_BITS    13
`define pippo_MSR_ME_BITS    12
`define pippo_MSR_FE0_BITS   11	// Unimplemented
`define pippo_MSR_DWE_BITS   10	
`define pippo_MSR_DE_BITS    9	
`define pippo_MSR_FE1_BITS   8	// Unimplemented
`define pippo_MSR_IR_BITS    5
`define pippo_MSR_DR_BITS    4

//
// CR
//
`define pippo_CR_CR0_LT_BITS   31
`define pippo_CR_CR0_GT_BITS   30
`define pippo_CR_CR0_EQ_BITS   29
`define pippo_CR_CR0_SO_BITS   28

//
// SPRs: address, field bits, fields
//

//
// UISA Model
//

// XER
`define pippo_SPR_XER		    10'h001

`define pippo_SPR_XER_SO_BITS   31
`define pippo_SPR_XER_OV_BITS   28
`define pippo_SPR_XER_CA_BITS   29
`define pippo_SPR_XER_TBC_BITS  6:0

`define pippo_SPR_XER_SO        
`define pippo_SPR_XER_OV        
`define pippo_SPR_XER_CA        
`define pippo_SPR_XER_TBC       

`define pippo_SPR_LR		    10'h008
`define pippo_SPR_CTR		    10'h009

`define pippo_SPR_USPRG0		10'h100

`define pippo_SPR_SPRG4U		10'h104
`define pippo_SPR_SPRG5U		10'h105
`define pippo_SPR_SPRG6U		10'h106
`define pippo_SPR_SPRG7U		10'h107

//
// Supervisor Model
//

//
//  Exception Handling Registers
//
`define pippo_SPR_EVPR 		    10'h3D5
`define pippo_SPR_DEAR 		    10'h3D5     
`define pippo_SPR_SRR0 		    10'h01A
`define pippo_SPR_SRR1 		    10'h01B
`define pippo_SPR_SRR2 		    10'h3DE
`define pippo_SPR_SRR3 		    10'h3DF
`define pippo_SPR_ESR 		    10'h3D4
`define pippo_SPR_MCSR  	    10'h23C

// bit fields
`define pippo_SPR_ESR_MCI_BITS  31  // machine check - instruction
`define pippo_SPR_ESR_PIL_BITS  27  // program interrupt - illegal
`define pippo_SPR_ESR_PPR_BITS  26  // program interrupt - privileged
`define pippo_SPR_ESR_PTR_BITS  25  // program interrupt - trap
`define pippo_SPR_ESR_PEU_BITS  24  // program interrupt - unimplemented
`define pippo_SPR_ESR_DST_BITS  23  // data storage interrupt - store fault
//`define pippo_SPR_ESR_DIZ_BITS    // un-supported in pippo
//`define pippo_SPR_ESR_PFP_BITS    // un-supported in pippo
//`define pippo_SPR_ESR_PAP_BITS    // un-supported in pippo
//`define pippo_SPR_ESR_U0F_BITS    // un-supported in pippo

`define pippo_SPR_MCSR_MCS_BITS     31
`define pippo_SPR_MCSR_IPLBE_BITS   30
`define pippo_SPR_MCSR_DPLBE_BITS   29
//`define pippo_SPR_MCSR_TLBE_BITS      // un-supported in pippo
//`define pippo_SPR_MCSR_ICPE_BITS      // un-supported in pippo     
//`define pippo_SPR_MCSR_DCLPE_BITS     // un-supported in pippo    
//`define pippo_SPR_MCSR_DCFPE_BITS     // un-supported in pippo     
//`define pippo_SPR_MCSR_IMCE_BITS      // un-supported in pippo
//`define pippo_SPR_MCSR_TLBS_BITS      // un-supported in pippo


//
//  SPR General Registers
//
`define pippo_SPR_SPRG0 		10'h110
`define pippo_SPR_SPRG1 		10'h111
`define pippo_SPR_SPRG2 		10'h112
`define pippo_SPR_SPRG3 		10'h113
`define pippo_SPR_SPRG4 		10'h114
`define pippo_SPR_SPRG5 		10'h115
`define pippo_SPR_SPRG6 		10'h116
`define pippo_SPR_SPRG7 		10'h117

//  Timer Facilities
`define pippo_SPR_TBL           10'h11C 		  
`define pippo_SPR_TBLU          10'h10C 		  
`define pippo_SPR_TBU           10'h11D
`define pippo_SPR_TBUU          10'h10D
`define pippo_SPR_TCR           10'h3DA
`define pippo_SPR_TSR           10'h3D8
`define pippo_SPR_PIT           10'h3DB

`define pippo_TSR_ENW_BITS      31
`define pippo_TSR_WIS_BITS      30
`define pippo_TSR_WRS_BITS      29:28
`define pippo_TSR_PIS_BITS      27
`define pippo_TSR_FIS_BITS      26

`define pippo_TCR_WP_BITS       31:30
`define pippo_TCR_WRC_BITS      29:28
`define pippo_TCR_WIE_BITS      27
`define pippo_TCR_PIE_BITS      26
`define pippo_TCR_FP_BITS       25:24
`define pippo_TCR_FIE_BITS      23
`define pippo_TCR_ARE_BITS      22

//  Processor Version Register
`define pippo_SPR_PVR 		    10'h11F

`define pippo_SPR_PVR_OWN_BITS  31:20       // Owner Identifier: Identifies the owner of a core
`define pippo_SPR_PVR_PCF_BITS  19:16       // Processor Core Family: Identifies the processor core family.
`define pippo_SPR_PVR_CAS_BITS  15:10       // Cache Array Sizes: Identifies the cache array sizes.
`define pippo_SPR_PVR_PCV_BITS  9:6         // Processor Core Version: Identifies the core version for a specific combination of PVR[PCF] and PVR[CAS]
`define pippo_SPR_PVR_AID_BITS  5:0         // ASIC Identifier: Assigned sequentially; identifies an ASIC function, version, and technology

`define pippo_SPR_PVR_OWN       12'd0
`define pippo_SPR_PVR_PCF       4'd0
`define pippo_SPR_PVR_CAS       6'd0
`define pippo_SPR_PVR_PCV       4'd0
`define pippo_SPR_PVR_AID       6'd0

////////////////////////////////////////////
//
// Reset Value
//
`define pippo_SPR_XER_RESET     32'd0
`define pippo_SPR_USPRG0_RESET	32'd0
`define pippo_SPR_SPRG0_RESET	32'd0
`define pippo_SPR_SPRG1_RESET	32'd0
`define pippo_SPR_SPRG2_RESET	32'd0
`define pippo_SPR_SPRG3_RESET	32'd0
`define pippo_SPR_SPRG4_RESET	32'd0
`define pippo_SPR_SPRG5_RESET	32'd0
`define pippo_SPR_SPRG6_RESET	32'd0
`define pippo_SPR_SPRG7_RESET	32'd0

//
// un-supported SPRs at pippo
//

//  Core Configuration Registers
`define pippo_SPR_CCR0		    10'h3B3
`define pippo_SPR_CCR1		    10'h378
//  Storage Attribute Control Registers
`define pippo_SPR_DCCR          10'h3FA
`define pippo_SPR_DCWR          10'h3BA
`define pippo_SPR_ICCR          10'h3FB
`define pippo_SPR_SGR           10'h3B9
`define pippo_SPR_SLER          10'h3BB
`define pippo_SPR_SU0R          10'h3BC
//  Memory Management Registers
`define pippo_SPR_PID 		    10'h3B1
`define pippo_SPR_ZPR 		    10'h3B0
//  Debugger Registers		  
`define pippo_SPR_DBSR          10'h3F0
`define pippo_SPR_DBCR0         10'h3F2
`define pippo_SPR_DBCR1         10'h3BD
`define pippo_SPR_DAC1          10'h3F6
`define pippo_SPR_DAC2          10'h3F7
`define pippo_SPR_DVC1          10'h3B6
`define pippo_SPR_DVC2          10'h3B7
`define pippo_SPR_IAC1          10'h3F4
`define pippo_SPR_IAC2          10'h3F5
`define pippo_SPR_IAC3          10'h3B4
`define pippo_SPR_IAC4          10'h3B5
`define pippo_SPR_ICDBR         10'h3D3

////////////////////////////////////////////////////////
//
// pippo-specific defines
//


//
// Emulation 
//
`define pippo_EMU_ROMADDR		32'hffff_fc00

`define pippo_SPR_EIR 		10'h090         // emulating instruction register

//
// pippo-specific SPRs: debug support unit(uartlite)
//
`define pippo_SPR_DSUTX 		10'h350
`define pippo_SPR_DSURX 		10'h351
`define pippo_SPR_DSUCTRL 		10'h352
`define pippo_SPR_DSUSTA 		10'h353
