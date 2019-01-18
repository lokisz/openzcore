/*
 * File:        def_pippo.v
 * Project:     pippo
 * Designer:    fang@ali
 * Mainteiner:  fang@ali
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
//  Part 4: general definition for RISC-V architecture and implementations. 
//  Note: User CAN'T CHANGE
//

// arch version
`define RV32                32
`define RV64                64

// NOP inst: 
//      addi x0, x0, 0;
// is defined as NOP at RISC-V architecture
`define pippo_RISCV_NOP     32'b0000_0000_0000_0000_0000_0000_0001_0011
//
// Operand width / register file address width
//
`define OPERAND_WIDTH		`RV64
`define GPR_ADDR_WIDTH	    5
`define SPR_ADDR_WIDTH	    12

//////////////////////////////////////////////
//
// RISC-V ISA and uops
//

//
// Binary compatiable level
//
`define BINARY_pippo

//
// RISC-V Instruction Form
//
// opcode bit place in instructions
`define OPCD_BITADDR            6:0
`define OPCD_F3_BITADDR         14:12     
`define OPCD_F7_BITADDR         31:25     
// system special register access
`define SPR_12BITADDR           31:20
// register operand bit palce in instructions
`define OPERAND_RS1_BITADDR     19:15
`define OPERAND_RS2_BITADDR     24:20
`define OPERAND_RD_BITADDR      11:7
//  five types imm operand in instructions, used at decode stage:
`define I_IMM       {21{id_inst[31]}, id_inst[30:20]}
`define S_IMM       {21{id_inst[31]}, id_inst[30:25, id_inst[11:7]}
`define SB_IMM      {20{id_inst[31]}, id_inst[7], id_inst[30:25], id_inst[11:8]}    // can't used by conditional branch decode
`define U_IMM       {id_inst[31:12], 12{1'b0}}
`define UJ_IMM      {12{id_inst[31]}, id_inst[19:12], id_inst[20], id_inst[30:21], 1'b0}
// special imm extension for csr instructions
`define CSR_IMM     {27{1'b0}, id_inst[19:15]}

//
// RISC-V ISA primary opcode (OPCD)
//
`ifdef BINARY_pippo
    `define LOAD_OPCD               7'b000_0011  
      `define LB_FUNC3_OPCD           3'b000
      `define LH_FUNC3_OPCD           3'b001
      `define LW_FUNC3_OPCD           3'b010
      `define LBU_FUNC3_OPCD          3'b100
      `define LHU_FUNC3_OPCD          3'b101
      `define LWU_FUNC3_OPCD          3'b110    // RV64I
      `define LD_FUNC3_OPCD           3'b011    // RV64I
      
            
    `define STORE_OPCD              7'b010_0011
      `define SB_FUNC3_OPCD           3'b000
      `define SH_FUNC3_OPCD           3'b001
      `define SW_FUNC3_OPCD           3'b010
      `define SD_FUNC3_OPCD           3'b011    // RV64I
        
    `define BRANCH_OPCD             7'b110_0011
      `define BEQ_FUNC3_OPCD          3'b000
      `define BNE_FUNC3_OPCD          3'b001
      `define BLT_FUNC3_OPCD          3'b100
      `define BGE_FUNC3_OPCD          3'b101
      `define BLTU_FUNC3_OPCD         3'b110
      `define BGEU_FUNC3_OPCD         3'b111

    `define JALR_OPCD               7'b110_0111 
      `define JALR_FUNC3_OPCD         3'b000 

    `define MISCMEM_OPCD            7'b000_1111
      `define FENCE_FUNC3_OPCD        3'b000 
      `define FENCEI_FUNC3_OPCD       3'b001     

    `define JAL_OPCD                7'b110_1111

    `define OPIMM_OPCD              7'b001_0011
      `define ADDI_FUNC3_OPCD         3'b000
      `define SLTI_FUNC3_OPCD         3'b010
      `define SLTIU_FUNC3_OPCD        3'b011
      `define XORI_FUNC3_OPCD         3'b100
      `define ORI_FUNC3_OPCD          3'b110
      `define ANDI_FUNC3_OPCD         3'b111
      `define SLLI_FUNC3_OPCD         3'b001
        `define SLLI_FUNC7_OPCD         7'b000_000x     // RV64I
      `define SRLI_FUNC3_OPCD         3'b101
      `define SRAI_FUNC3_OPCD         3'b101
        `define SRLI_FUNC7_OPCD         7'b000_000x     // RV64I
        `define SRAI_FUNC7_OPCD         7'b010_000x     // RV64I

    `define OP_OPCD                 7'b011_0011
      `define ADD_FUNC3_OPCD          3'b000
      `define SUB_FUNC3_OPCD          3'b000
        `define ADD_FUNC7_OPCD          7'b000_0000
        `define SUB_FUNC7_OPCD          7'b010_0000
      `define SLL_FUNC3_OPCD          3'b001
        `define SLI_FUNC7_OPCD          7'b000_0000
      `define SLT_FUNC3_OPCD          3'b010
        `define SLT_FUNC7_OPCD          7'b000_0000
      `define SLTU_FUNC3_OPCD         3'b011
        `define SLTU_FUNC7_OPCD         7'b000_0000
      `define XOR_FUNC3_OPCD          3'b100
        `define XOR_FUNC7_OPCD          7'b000_0000
      `define SRL_FUNC3_OPCD          3'b101
      `define SRA_FUNC3_OPCD          3'b101
        `define SRL_FUNC7_OPCD          7'b000_0000
        `define SRA_FUNC7_OPCD          7'b010_0000
      `define OR_FUNC3_OPCD           3'b110
        `define OR_FUNC7_OPCD           7'b000_0000
      `define AND_FUNC3_OPCD          3'b111
        `define AND_FUNC7_OPCD          7'b000_0000

    `define SYSTEM_OPCD             7'b111_0011
      `define SCALL_FUNC3_OPCD        3'b000 
      `define SBREAK_FUNC3_OPCD       3'b000 
      `define SRET_FUNC3_OPCD         3'b000                    // supervisor inst.
        `define SCALL_FUNC12_OPCD       12'b0000_0000_0000
        `define SBREAK_FUNC12_OPCD      12'b0000_0000_0001
        `define SRET_FUNC12_OPCD        12'b1000_0000_0000
      `define CSRRW_FUNC3_OPCD        3'b001                    // supervisor inst.
      `define CSRRS_FUNC3_OPCD        3'b010                    // user & supervisor inst.
        `define RDCYCLE_FUNC12_OPCD     12'b1100_0000_0000
        `define RDCYCLEH_FUNC12_OPCD    12'b1100_1000_0000
        `define RDTIME_FUNC12_OPCD      12'b1100_0000_0001
        `define RDTIMEH_FUNC12_OPCD     12'b1100_1000_0001
        `define RDINSTRET_FUNC12_OPCD   12'b1100_0000_0010
        `define RDINSTRETH_FUNC12_OPCD  12'b1100_1000_0010
      `define CSRRC_FUNC3_OPCD        3'b011                    // supervisor inst.
      `define CSRRWI_FUNC3_OPCD       3'b101                    // supervisor inst.
      `define CSRRSI_FUNC3_OPCD       3'b110                    // supervisor inst.
      `define CSRRCI_FUNC3_OPCD       3'b111                    // supervisor inst.

    `define AUIPC_OPCD              7'b001_0111
    `define LUI_OPCD                7'b011_0111

    `define OPIMM32_OPCD            7'b001_1011             // RV64I
      `define ADDIW_FUNC3_OPCD        3'b000 
      `define SLLIW_FUNC3_OPCD        3'b001 
        `define SLLIW_FUNC7_OPCD        7'b000_0000
      `define SRLIW_FUNC3_OPCD        3'b101 
      `define SRAIW_FUNC3_OPCD        3'b101 
        `define SRLIW_FUNC7_OPCD        7'b000_0000
        `define SRAIW_FUNC7_OPCD        7'b010_0000
      
    `define OP32_OPCD               7'b011_1011             // RV64I
      `define ADDW_FUNC3_OPCD         3'b000 
      `define SUBW_FUNC3_OPCD         3'b000 
        `define ADDW_FUNC7_OPCD         7'b000_0000
        `define SUBW_FUNC7_OPCD         7'b010_0000
      `define SLLW_FUNC3_OPCD         3'b001
        `define SLLW_FUNC7_OPCD         7'b000_0000
      `define SRLW_FUNC3_OPCD         3'b101 
      `define SRAW_FUNC3_OPCD         3'b101 
        `define SRLW_FUNC7_OPCD         7'b000_0000
        `define SRAW_FUNC7_OPCD         7'b010_0000
    
`endif  // `define BINARY_pippo

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
`define ALUOP_OR	    6'b000101
`define ALUOP_XOR	    6'b000111
`define ALUOP_SLT	    6'b000101
`define ALUOP_SLTU	    6'b000101

// Arithmetic
`define ALUOP_ADD	    6'b001010
`define ALUOP_ADDW      6'b001100
`define ALUOP_SUB	    6'b001101
`define ALUOP_SUBW      6'b001110

//`define ALUOP_MULHW     6'b010000
//`define ALUOP_MULLI     6'b010010
//`define ALUOP_MULLW     6'b010011
//`define ALUOP_MULHWU    6'b010100
//`define ALUOP_DIVW    6'b000000
//`define ALUOP_DIVWU	6'b000000

// Shift
// bit[x]: enable
// bit[3]: 32b mode
// bit[1]: right
// bit[0]: algrithm
`define ALUOP_SHTEN_BIT x   
`define ALUOP_M32B_BIT  2   
`define ALUOP_LFT_BIT   1   
`define ALUOP_AGM_BIT   0   

`define ALUOP_SLL       6'b001_010
`define ALUOP_SRL       6'b001_000
`define ALUOP_SRA       6'b001_001
`define ALUOP_SLLW      6'b001_110
`define ALUOP_SRLW      6'b001_100
`define ALUOP_SRAW      6'b001_101

//
// LSUOPs
//
// Bit 5: 0 load, 1 store
// Bit 4: unsigned extention
// Bit 3: 64bit access
// Bit 2: 32bit access
// Bit 1: 16bit access
// Bit 0: 8bit access
`define LSUUOPS_WIDTH	6
`define LSUOP_WIDTH		6

`define LSUOP_NOP		6'b00_0000

`define LSUOP_LB 		6'b00_0001
`define LSUOP_LH 		6'b00_0010
`define LSUOP_LW 		6'b00_0100
`define LSUOP_LBU 		6'b01_0001
`define LSUOP_LHU		6'b01_0010
`define LSUOP_LWU		6'b01_0100
`define LSUOP_LD		6'b00_1000

`define LSUOP_SB		6'b10_0001
`define LSUOP_SH		6'b10_0010
`define LSUOP_SW		6'b10_0100
`define LSUOP_SD		6'b10_1000

//
// BPU uops
//
// bit[7]: 1 means jump(unconditional branch), o means conditional branch
// bit[6]: 1 means signed compare, 0 means unsigned compare
`define BPUUOPS_WIDTH	8
`define BPUOP_WIDTH		8
`define BPUOP_JUMP_BIT  7   
`define BPUOP_SCMP_BIT  6   

`define BPUOP_NOP       8'b0000_0000
`define BPUOP_PCIMM     8'b1000_0001
`define BPUOP_REGIMM    8'b1000_0010
`define BPUOP_CBEQ      8'b0000_0100
`define BPUOP_CBNE      8'b0000_1000
`define BPUOP_CBLT      8'b0001_0000
`define BPUOP_CBGE      8'b0010_0000
`define BPUOP_CBLTU     8'b0101_0000
`define BPUOP_CBGEU     8'b0110_0000

//
// SPRs op
//
`define REGUOPS_WIDTH   4
`define REGOP_WIDTH     4
`define REGOP_NOP       4'b0000

`define REGOP_CSRRW     4'b1001
`define REGOP_CSRRS     4'b1010
`define REGOP_CSRRC     4'b1100

//
// Register File Write-Back OPs
//
`define RFWBUOPS_WIDTH	4
`define RFWBOP_WIDTH	4

`define RFWBOP_NOP		4'b0000
`define RFWBOP_ALU		4'b0001
`define RFWBOP_LSU		4'b0010
`define RFWBOP_SPRS		4'b0100
`define RFWBOP_BPU  	4'b1000

//
// Execution cycles per instruction
//
`define MULTICYCLE_WIDTH	    4

`define EXTEND_ZERO_CYCLES		    4'b0000
`define EXTEND_ONE_CYCLES		    4'b0001
`define EXTEND_TWO_CYCLES		    4'b0010
`define EXTEND_THREE_CYCLES		    4'b0100

//
// Operand Muxes
//
`define OPSEL_WIDTH		    3

`define OPSEL_RF			3'b001
`define OPSEL_IMM			3'b010
`define OPSEL_WBFWD 		3'b100

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
//`define pippo_EXCEPT_WIDTH  16

//
// Definition of exception type/vectors
//      To avoid implementation of a certain exception,
//      simply comment out corresponding line
//
`define BINARY_RV64_EXP

`define pippo_EXCEPT_NONE   `pippo_EXCEPT_WIDTH'hfff8
    
// special mode for minimal system configuration and system debug
`ifdef BINARY_RV64_EXP
    `define pippo_EXCEPT_ALIGH_INST
    `define pippo_EXCEPT_IBUSERR
    `define pippo_EXCEPT_ILLEGAL
    `define pippo_EXCEPT_SVMCHECK
    `define pippo_EXCEPT_FPDISABLE
    `define pippo_EXCEPT_SCALL
    `define pippo_EXCEPT_SBREAK
    `define pippo_EXCEPT_ALIGN_LOAD
    `define pippo_EXCEPT_ALIGH_STORE
    `define pippo_EXCEPT_DBUSERR_LOAD
    `define pippo_EXCEPT_DBUSERR_STORE
    `define pippo_EXCEPT_EXTI0
    `define pippo_EXCEPT_EXTI1
    `define pippo_EXCEPT_EXTI2
    `define pippo_EXCEPT_EXTI3
    `define pippo_EXCEPT_EXTI4
    `define pippo_EXCEPT_EXTI5
    `define pippo_EXCEPT_EXTI6
    `define pippo_EXCEPT_EXTI7
`endif

///////////////////////////////////////////////////////
//
// Register sets
//

//
// All CSRs address at 12bit address space
//  compatiable with Rocket
//`define pippo_CSR_FFFLAG	12'h001
//`define pippo_CSR_FRM		12'h002
//`define pippo_CSR_FCSR	12'h003
`define pippo_CSR_STATS		12'h0c0
`define pippo_CSR_SUP0		12'h500
`define pippo_CSR_SUP1		12'h501
`define pippo_CSR_EPC		12'h502
`define pippo_CSR_BADADDR	12'h503
//`define pippo_CSR_PTBR	12'h504
//`define pippo_CSR_ASID	12'h505
`define pippo_CSR_COUNT		12'h506
`define pippo_CSR_COMPARE	12'h507
`define pippo_CSR_EVEC		12'h508
`define pippo_CSR_CAUSE		12'h509
`define pippo_CSR_STATUS	12'h50a
`define pippo_CSR_HARTID    12'h50b
`define pippo_CSR_IMPL		12'h50c
`define pippo_CSR_FATC		12'h50d
`define pippo_CSR_SEND_IPI	12'h50e
`define pippo_CSR_CLEAR_IPI 12'h50f
`define pippo_CSR_RESET		12'h51d
`define pippo_CSR_TOHOST	12'h51e
`define pippo_CSR_FROMHOST  12'h51f
`define pippo_CSR_CYCLE		12'hc00
`define pippo_CSR_TIME		12'hc01
`define pippo_CSR_INSTRET	12'hc02
`define pippo_CSR_UARCH0    12'hcc0
`define pippo_CSR_UARCH1	12'hcc1
`define pippo_CSR_UARCH2	12'hcc2
`define pippo_CSR_UARCH3	12'hcc3
`define pippo_CSR_UARCH4	12'hcc4
`define pippo_CSR_UARCH5	12'hcc5
`define pippo_CSR_UARCH6	12'hcc6
`define pippo_CSR_UARCH7	12'hcc7
`define pippo_CSR_UARCH8	12'hcc8
`define pippo_CSR_UARCH9	12'hcc9
`define pippo_CSR_UARCH10	12'hcca
`define pippo_CSR_UARCH11	12'hccb
`define pippo_CSR_UARCH12	12'hccc
`define pippo_CSR_UARCH13	12'hccd
`define pippo_CSR_UARCH14	12'hcce
`define pippo_CSR_UARCH15	12'hccf
`define pippo_CSR_COUNTH	12'h586
`define pippo_CSR_CYCLEH	12'hc80
`define pippo_CSR_TIMEH		12'hc81
`define pippo_CSR_INSTRETH	12'hc82

//
// user model
//

//
// Machine State Register (Supervisor state)
//
//      [31:24]: Reserved
//      [23:16]: IM-interrupts mask bits
//          0 interrupt disabled.
//          1 interrupt enable.
//      [15:8]: Reserved
//      [7]: VM-virtual memory
//          0 memory address translation is disabled.
//          1 memory address translation is enabled.
//      [6]: S64-executes RV64 isa in supervisor mode
//          0 executes RV32 isa in supervisor mode
//          1 executes RV64 isa in supervisor mode
//      [5]: U64- executes RV64 isa in user mode
//          0 executes RV32 isa in user mode
//          1 executes RV64 isa in user mode
//      [4]: S-state
//          0 user mode
//          1 supervisor mode
//      [3]: PS-stack for S field
//          When an exception is taken, the PS bit is set to the value of the S bit. 
//          When an eret instruction is executed, the S bit is set to the value of the PS bit; the PS bit is unchanged.
//      [2]: Reserved
//      [1]: EF-enables or disables floating-point instructions
//          0 disable
//          1 enable 
//      [0]: ET-globally enables or disable exceptions
//          0 interrupts are not taken, and any trap causes the processor to enter error mode
//          1 interrupts are enable
//          notes: The ET bit is cleared when an exception is taken and set when an eret instruction is executed
`define pippo_MSR_RESET     32'd0

// Bit Fields
`define pippo_MSR_IM7_BITS  23
`define pippo_MSR_IM6_BITS  22   
`define pippo_MSR_IM5_BITS  21   
`define pippo_MSR_IM4_BITS  20   
`define pippo_MSR_IM3_BITS  19   
`define pippo_MSR_IM2_BITS  18   
`define pippo_MSR_IM1_BITS  17   
`define pippo_MSR_IM0_BITS  16   
`define pippo_MSR_VM_BITS   7
`define pippo_MSR_S64_BITS  6
`define pippo_MSR_U64_BITS  5
`define pippo_MSR_S_BITS    4
`define pippo_MSR_PS_BITS   3
`define pippo_MSR_EF_BITS   1
`define pippo_MSR_ET_BITS   0

//
// Supervisor Model
//

//
//  Exception Handling Registers
//
// bit fields
`define pippo_SPR_ESR_MCI_BITS  31  // machine check - instruction
`define pippo_SPR_ESR_PIL_BITS  27  // program interrupt - illegal
`define pippo_SPR_ESR_PPR_BITS  26  // program interrupt - privileged
`define pippo_SPR_ESR_PTR_BITS  25  // program interrupt - trap
`define pippo_SPR_ESR_PEU_BITS  24  // program interrupt - unimplemented
`define pippo_SPR_ESR_DST_BITS  23  // data storage interrupt - store fault

`define pippo_SPR_MCSR_MCS_BITS     31
`define pippo_SPR_MCSR_IPLBE_BITS   30
`define pippo_SPR_MCSR_DPLBE_BITS   29


//
//  SPR General Registers
//

//
//  Timer Facilities
//
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
`define pippo_SPR_USPRG0_RESET	32'd0
`define pippo_SPR_SPRG0_RESET	32'd0
`define pippo_SPR_SPRG1_RESET	32'd0
`define pippo_SPR_SPRG2_RESET	32'd0
`define pippo_SPR_SPRG3_RESET	32'd0
`define pippo_SPR_SPRG4_RESET	32'd0
`define pippo_SPR_SPRG5_RESET	32'd0
`define pippo_SPR_SPRG6_RESET	32'd0
`define pippo_SPR_SPRG7_RESET	32'd0

////////////////////////////////////////////////////////
//
// pippo-specific defines
//


//
// Emulation 
//
`define pippo_EMU_ROMADDR		32'hffff_fc00

`define pippo_SPR_EIR 		12'h090         // emulating instruction register

//
// pippo-specific SPRs: debug support unit(uartlite)
//
`define pippo_SPR_DSUTX 		12'h350
`define pippo_SPR_DSURX 		12'h351
`define pippo_SPR_DSUCTRL 		12'h352
`define pippo_SPR_DSUSTA 		12'h353
