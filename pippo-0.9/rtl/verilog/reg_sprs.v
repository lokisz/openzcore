/*
 * File:        reg_sprs.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Description:
    一，功能描述
        a）为显性和隐性系统寄存器访问指令（mfspr/mtspr，mtmsr/mfmsr，mfcr，mtcrf，mcrxr，wrtee/wrteei，rfi/rfci）
           和其他需要访问的情况（中断处理），提供访问统一接口－地址和数据。
                执行mtspr时，其他模块的SPRs写数据来自spr_dat_wr_o（经Operandmux的GPR数据）
        b）实现部分SPRs：
            MSR：
                写操作－mtmsr（gpr），wrtee/wrteei更新MSR[EE]（gpr或imm），rfi/rfci（srr1/srr3寄存器），中断发生
                读操作－mfmsr（读至gpr），中断发生（读至srr寄存器）
                注意，PWR架构中，根据中断发生后，MSR必须根据中断类型进行更新
            CR：除寄存器操作指令，CR的更新还来自ALU模块和LSU模块相关指令的执行
                写操作－mtcrf（gpr），mcrf，mcrxr（XER[31:28]），stwcx.指令（CR0），[o]格式的ALU指
                        令（更新CR0），比较和CR运算指令
                读操作－mfcr，mcrf，条件分支指令
            XER：
                写操作－mtspr，mcrxr（XER[31:28]清零），[.]格式ALU指令更新XER[SO, OV]，Carrying格式ALU指令更新XER[CA]
                读操作－mfspr，Extending格式的ALU指令读取XER[CA]
                [OV]：表示溢出，运算结果超出范围。XER[OV] = 1 indicates overflow. For arithmetic operations, 
                      this occurs when an operation has a carry-in to the most-significant bit of the result that
                      does not equal the carry-out of the most-significant bit (that is, the exclusive-or of the 
                      carry-in and the carry-out is 1).
                      Multiply and divide instructions (mullwo, mullwo., divwo, divwo., divwuo, divwuo)未定义
                [CA]：表示进位。
            其他SPRs实现分布如下：
                CTR和LR在BPU模块实现
	            中断处理相关SPRs在except模块实现，包括DEAR, ESR, EVPR, MCSR, SRR0, SRR1, SRR2和SRR3等
	            各系统模块相关的SPRs，如Cache，MMU和Timer等，分别在相应模块实现
            实现DSU的SPRs访问接口，链接uartlite模块；
                DSUTX, DSURX, DSUCTRL, DSUSTA
        c）检查SPRs访问相关的处理器状态，发出响应中断请求sig_svm_check
    注：如无特别说明，这里所说的SPRs包括MSR和CR
    二，PWR架构SPRs有
        Original ppc405 register sets excluding GPRs including:
            a. MSR/CR
            b. SPRs:
            b.1) Branch Control
                    CTR, LR - User
            b.2) Fixed-point Exception
                    XER - User
            b.3) Interrupts and Exceptions (implemented at except module)
                    DEAR, ESR, EVPR, MCSR, SRR0, SRR1, SRR2, SRR3 - Privileged
            b.4) General-Purpose SPR
                    USPRG0 - User
                    SPRG0, SPRG1, SPRG2, SPRG3 - Privileged
                    SPRG4, SPRG5, SPRG6, SPRG7 - User read, privileged write
            b.5) Timer Facilities at timer units
                    TBL, TBU - Privileged, write only
                    PIT, TCR, TSR - Priveleged  
        Currently, pippo core implemented SPRs above.
 *      Following SPRs are implemented by core-out units
            b.6) Processor Version (Read only)
                    PVR - Privileged, read-only
 *          c.7) Configuration (MMU/Cache)
 *                  CCR0, CCR1 - Privileged
 *          c.8) Storage Attributes Control Registers at mmu and cache units, for memory access at real-address mode
 *                  DCCR, DCWR, ICCR, SGR, SLER, SU0R
 *          c.9) Zone Protection at mmu/mpu units
 *                  ZPR 
 *          c.10) Debug at debug unit
 *                  DAC1, DAC2
 *                  DBCR0, DBCR1
 *                  DBSR
 *                  DVC1, DVC2
 *                  IAC1, IAC2, IAC3, IAC4
 *                  ICDBDR
 * Task.I
        [TBD]对于未实现的SPRs，例如CCR0/CCR1等，执行mfspr返回全0，执行mtspr行为无定义－类似空操作；
        MSR/XER的更新值位置补全
        [TBD]CR[CR0]的[SO]域是XER[SO]的复制，如何理解？
            1，[.]格式ALU指令更新XER[SO]和XER[OV]，则CR0[SO]是否同步更新？
            2，CR的写操作引起CR0[SO]更新，是否反应至XER[OV]
 * List2do:
 *      SPRs地址译码统一在本模块实现？然后送出we和data_wr，或rde和接收data_rd
 *          地址来自ex_inst，注意连接关系在顶层模块连接时实现，例如SPRs地址的置换
 *      写回SPRs时，是否也要增加流水控制逻辑？
 *          1，SPRs访问指令都是单周期执行完成，进入EXE阶段，就会正常wb，不会产生wb_freeze的情况
 *          2，[TBD]SPRs访问指令可能产生中断－非supervisor态执行previlleged操作（访问supervisor的SPRs）等，需要根据flushpipe取消写回？
 *      msr更新值是否需要送至except模块
 *      SPRs布线会不会带来问题－过多io连接，替代方案：两次译码，例如在except模块再对spr_addr译码
 */
 
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_sprs(
		// Clk & Rst
		clk, rst,

		reg_uops, reg_addr, dat_i, spr_wb_dat,

		carry, so,
    	so_new, so_we, 
        ov_new, ov_we, 
        ca_new, ca_we, 

	    cr_addr, 
	    cr, 
	    cr_alu, cr_alu_we, 
        cr0_lsu, cr0_lsu_we, 
        
		msr,
		msr_except, msr_expwe,
		sig_svm_check, 
		
		dear, esr, evpr, mcsr, srr0, srr1, srr2, srr3, 
		dear_we, esr_we, evpr_we, mcsr_we, srr0_we, srr1_we, srr2_we, srr3_we,

        eir, eir_we,
        
        lr, ctr,
        lr_we, ctr_we,
        
        tbl, tbu, pit, tsr, tcr, 
        tbl_we, tbu_we, pit_we, tsr_we, tcr_we, 

        dsurx, dsutx, dsuctrl, dsusta, 
        dsurx_we, dsutx_we, dsuctrl_we, dsusta_we, 
                
        spr_dat_wr_o
);

parameter width = `OPERAND_WIDTH;

//
// I/O Ports
//

input				clk; 		
input 				rst;		

//
// Pipeline interface
//
input	[`REGOP_WIDTH-1:0]	reg_uops;	
input	[9:0]       reg_addr;	    
input	[31:0]		dat_i;	        
input   [14:0]      cr_addr;            // cr source selector, connect from ex_inst
output	[31:0]	    spr_wb_dat;         // write-back data for mfspr-class instructions

//
// CR output for bpu, alu
//
output  [31:0]  cr;

//
// ALU Interface
//
input   so_new;
input   so_we;
input   ov_new;
input   ov_we;
input   ca_new;
input   ca_we;

//
//  LSU Interface
//        
// cr0 update for atomic instructions
input   [3:0]   cr0_lsu;
input           cr0_lsu_we;

//
// ALU Interface
//
// CR
input   [31:0]      cr_alu;
input   [7:0]       cr_alu_we; 
// XER/CR update information for ALU 
output 				carry;		// XER[CY]
output              so; 

//
// Interface with LR/CTR at BPU
//
input   [31:0]      lr;
input   [31:0]      ctr;
output              lr_we;
output              ctr_we;

//
// Interface with Exception Unit: expception request, SPRs access
//
output              sig_svm_check;

input 				msr_expwe; 
input	[31:0]	    msr_except;
output	[31:0]	    msr;

output				dear_we;
output				esr_we;
output				srr0_we;
output				srr1_we;
output				srr2_we;
output				srr3_we;
output				evpr_we;
output				mcsr_we;

input	[width-1:0] 	dear;
input	[width-1:0] 	esr;		    
input	[width-1:0] 	srr0;	
input	[width-1:0] 	srr1;	
input	[width-1:0] 	srr2;	
input	[width-1:0] 	srr3;
input	[width-1:0] 	evpr;	
input	[width-1:0] 	mcsr;	

//
output          eir_we;
input   [width-1:0]     eir;

//
// timer
//
input   [31:0]          tbl, tbu, pit, tsr, tcr; 
output                  tbl_we, tbu_we, pit_we, tsr_we, tcr_we;

//
// interface with dsu
//
input   [7:0]           dsurx, dsutx, dsuctrl, dsusta;
output                  dsurx_we, dsutx_we, dsuctrl_we, dsusta_we;

//
// To/from SPRs at core-out building blocks
//
output	[31:0]			spr_dat_wr_o;	

//
// Internal regs & wires
//
reg	[width-1:0]	msr;
reg	[width-1:0]	xer;

wire [31:0] cr; 
reg [3:0]   cr0; 
reg [3:0]   cr1; 
reg [3:0]   cr2; 
reg [3:0]   cr3; 
reg [3:0]   cr4; 
reg [3:0]   cr5; 
reg [3:0]   cr6; 
reg [3:0]   cr7; 

wire    [31:0]  pvr;

reg     write_spr;
reg     read_spr; 
reg	[width-1:0]	spr_wb_dat;	

wire	[`REGOP_WIDTH-1:0]	reg_uops;
wire    [9:0]   spr_addr; 
wire            pvr_sel;
wire            cr_we;
wire            xer_we; 

wire            carry;
wire            so;

wire    [31:0]  xer_new;
wire    [3:0]   cr0_new; 
wire    [3:0]   cr1_new; 
wire    [3:0]   cr2_new; 
wire    [3:0]   cr3_new; 
wire    [3:0]   cr4_new; 
wire    [3:0]   cr5_new; 
wire    [3:0]   cr6_new; 
wire    [3:0]   cr7_new; 
wire    [3:0]   cr0_alu;
wire    [3:0]   cr1_alu;
wire    [3:0]   cr2_alu;
wire    [3:0]   cr3_alu;
wire    [3:0]   cr4_alu;
wire    [3:0]   cr5_alu;
wire    [3:0]   cr6_alu;
wire    [3:0]   cr7_alu;
wire            so_new; 
wire            ov_new; 

wire    lr_sel;
wire    lr_we;
wire    ctr_sel;
wire    ctr_we;

wire [31:0] spr_dat_i; 
wire [31:0] msr_new; 

//
wire [7:0]  dsurx, dsutx, dsuctrl, dsusta; 

//
wire [31:0] tbl, tbu, pit, tsr, tcr;

//
// Generate sprs opcode
//
assign spr_addr = reg_addr;     
assign spr_dat_i = dat_i;       // write data to SPRs, send out by DSS or mtspr-class instructions
assign spr_dat_wr_o = dat_i;    // bypass the write data to other SPRs, from GPRs via operandmux

//
// supervisor model check
//  svm instructions：mfmsr/mtmsr, rfi/rfci, wrtee/wrteei, mfdcr/mtdcr
//      msb of spr_addr must be 1'b0 under mfspr/mtspr case
//      rfi/rfci is check at except module
assign sig_svm_check = msr[`pippo_MSR_PR_BITS] & 
                    (((reg_uops == `REGOP_MTMSR) | (reg_uops == `REGOP_WRTEE) | (reg_uops == `REGOP_MFMSR)) |
                    (spr_addr[9] & (reg_uops == `REGOP_MFSPR) | (reg_uops == `REGOP_MFSPR)));

//
// MSR
//
// Note: 1, MSR of pippo only implemented: [PR][ME][CE][EE]
//       2, other unimplemented fields are keep to 1'b0;
//       3, reserved field operate as 405

assign msr_we = msr_expwe | (!sig_svm_check & (reg_uops == `REGOP_MTMSR) | (reg_uops == `REGOP_WRTEE));

// [TBV] msr_new logic's coding style: mux should be inferred, not priority decoder
assign msr_new[`pippo_MSR_PR_BITS] = 
		(reg_uops == `REGOP_MTMSR)? spr_dat_i[`pippo_MSR_PR_BITS]: 
		msr[`pippo_MSR_PR_BITS];

assign msr_new[`pippo_MSR_ME_BITS] = 
		(reg_uops == `REGOP_MTMSR)? spr_dat_i[`pippo_MSR_ME_BITS]: 
		msr[`pippo_MSR_ME_BITS];

assign msr_new[`pippo_MSR_CE_BITS] = 
		(reg_uops == `REGOP_MTMSR)? spr_dat_i[`pippo_MSR_CE_BITS]: 
		msr[`pippo_MSR_CE_BITS];

// wrtee/wrteei
assign msr_new[`pippo_MSR_EE_BITS] =
		(reg_uops == `REGOP_MTMSR)? spr_dat_i[`pippo_MSR_EE_BITS]: 
		(reg_uops == `REGOP_WRTEE)? spr_dat_i[`pippo_MSR_EE_BITS]:
		msr[`pippo_MSR_EE_BITS];

always @(posedge clk or posedge rst)
	if (rst)
		msr <= #1 `pippo_MSR_RESET;
	else if (msr_expwe)
        msr <= #1 msr_except;
	else if (msr_we)
		msr <= #1 msr_new;

//
// GP SPR
//
reg [31:0]  usprg0; 
reg [31:0]  sprg0, sprg1, sprg2, sprg3, sprg4, sprg5, sprg6, sprg7;

// USPRG0 - User
assign usprg0_sel = (spr_addr == `pippo_SPR_USPRG0);
assign usprg0_we = write_spr & usprg0_sel;

always @(posedge clk or posedge rst) begin
	if (rst)
		usprg0 <= #1 `pippo_SPR_USPRG0_RESET;
	else if (usprg0_we)
		usprg0 <= #1 spr_dat_i;
end

// SPRG0, SPRG1, SPRG2, SPRG3 - Privileged
assign sprg0_sel = (spr_addr == `pippo_SPR_SPRG0);
assign sprg1_sel = (spr_addr == `pippo_SPR_SPRG1);
assign sprg2_sel = (spr_addr == `pippo_SPR_SPRG2);
assign sprg3_sel = (spr_addr == `pippo_SPR_SPRG3);
assign sprg0_we = write_spr & sprg0_sel;
assign sprg1_we = write_spr & sprg1_sel;
assign sprg2_we = write_spr & sprg2_sel;
assign sprg3_we = write_spr & sprg3_sel;

always @(posedge clk or posedge rst) begin
	if (rst)
		sprg0 <= #1 `pippo_SPR_SPRG0_RESET;
	else if (sprg0_we)
		sprg0 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		sprg1 <= #1 `pippo_SPR_SPRG1_RESET;
	else if (sprg1_we)
		sprg1 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		sprg2 <= #1 `pippo_SPR_SPRG2_RESET;
	else if (sprg2_we)
		sprg2 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		sprg3 <= #1 `pippo_SPR_SPRG3_RESET;
	else if (sprg3_we)
		sprg3 <= #1 spr_dat_i;
end

// SPRG4, SPRG5, SPRG6, SPRG7 - User read, privileged write
assign sprg4_selu = (spr_addr == `pippo_SPR_SPRG4U);
assign sprg5_selu = (spr_addr == `pippo_SPR_SPRG5U);
assign sprg6_selu = (spr_addr == `pippo_SPR_SPRG6U);
assign sprg7_selu = (spr_addr == `pippo_SPR_SPRG7U);
assign sprg4_sel = (spr_addr == `pippo_SPR_SPRG4);
assign sprg5_sel = (spr_addr == `pippo_SPR_SPRG5);
assign sprg6_sel = (spr_addr == `pippo_SPR_SPRG6);
assign sprg7_sel = (spr_addr == `pippo_SPR_SPRG7);
assign sprg4_we = write_spr & sprg4_sel;
assign sprg5_we = write_spr & sprg5_sel;
assign sprg6_we = write_spr & sprg6_sel;
assign sprg7_we = write_spr & sprg7_sel;

always @(posedge clk or posedge rst) begin
	if (rst)
		sprg4 <= #1 `pippo_SPR_SPRG4_RESET;
	else if (sprg4_we)
		sprg4 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		sprg5 <= #1 `pippo_SPR_SPRG5_RESET;
	else if (sprg5_we)
		sprg5 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		sprg6 <= #1 `pippo_SPR_SPRG6_RESET;
	else if (sprg6_we)
		sprg6 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		sprg7 <= #1 `pippo_SPR_SPRG7_RESET;
	else if (sprg7_we)
		sprg7 <= #1 spr_dat_i;
end

//
// CR
//
//[TBD] we和wr逻辑简化

wire    [2:0]   crf_d;
wire    [2:0]   crf_s;
wire    [7:0]   fxm; 

assign crf_d = cr_addr[14:12]; 
assign crf_s = cr_addr[9:7]; 
assign fxm = cr_addr[8:1];
                
assign cr0_rd = ((reg_uops == `REGOP_MCRF) & crf_s == 3'b000);
assign cr1_rd = ((reg_uops == `REGOP_MCRF) & crf_s == 3'b001);
assign cr2_rd = ((reg_uops == `REGOP_MCRF) & crf_s == 3'b010);
assign cr3_rd = ((reg_uops == `REGOP_MCRF) & crf_s == 3'b011);
assign cr4_rd = ((reg_uops == `REGOP_MCRF) & crf_s == 3'b100);
assign cr5_rd = ((reg_uops == `REGOP_MCRF) & crf_s == 3'b101);
assign cr6_rd = ((reg_uops == `REGOP_MCRF) & crf_s == 3'b110);
assign cr7_rd = ((reg_uops == `REGOP_MCRF) & crf_s == 3'b111);

assign cr0_wr = ((reg_uops == `REGOP_MCRF) & crf_d == 3'b000) | 
                ((reg_uops == `REGOP_MTCRF) & fxm[7]) | 
                ((reg_uops == `REGOP_MCRXR) & crf_d == 3'b000);
assign cr1_wr = ((reg_uops == `REGOP_MCRF) & crf_d == 3'b001) | 
                ((reg_uops == `REGOP_MTCRF) & fxm[6]) | 
                ((reg_uops == `REGOP_MCRXR) & crf_d == 3'b001);
assign cr2_wr = ((reg_uops == `REGOP_MCRF) & crf_d == 3'b010) | 
                ((reg_uops == `REGOP_MTCRF) & fxm[5]) | 
                ((reg_uops == `REGOP_MCRXR) & crf_d == 3'b010);
assign cr3_wr = ((reg_uops == `REGOP_MCRF) & crf_d == 3'b011) | 
                ((reg_uops == `REGOP_MTCRF) & fxm[4]) | 
                ((reg_uops == `REGOP_MCRXR) & crf_d == 3'b011);
assign cr4_wr = ((reg_uops == `REGOP_MCRF) & crf_d == 3'b100) | 
                ((reg_uops == `REGOP_MTCRF) & fxm[3]) | 
                ((reg_uops == `REGOP_MCRXR) & crf_d == 3'b100);
assign cr5_wr = ((reg_uops == `REGOP_MCRF) & crf_d == 3'b101) | 
                ((reg_uops == `REGOP_MTCRF) & fxm[2]) | 
                ((reg_uops == `REGOP_MCRXR) & crf_d == 3'b101);
assign cr6_wr = ((reg_uops == `REGOP_MCRF) & crf_d == 3'b110) | 
                ((reg_uops == `REGOP_MTCRF) & fxm[1]) | 
                ((reg_uops == `REGOP_MCRXR) & crf_d == 3'b110);
assign cr7_wr = ((reg_uops == `REGOP_MCRF) & crf_d == 3'b111) | 
                ((reg_uops == `REGOP_MTCRF) & fxm[0]) | 
                ((reg_uops == `REGOP_MCRXR) & crf_d == 3'b111);

assign cr0_alu_we = cr_alu_we[7];
assign cr1_alu_we = cr_alu_we[6];
assign cr2_alu_we = cr_alu_we[5];
assign cr3_alu_we = cr_alu_we[4];
assign cr4_alu_we = cr_alu_we[3];
assign cr5_alu_we = cr_alu_we[2];
assign cr6_alu_we = cr_alu_we[1];
assign cr7_alu_we = cr_alu_we[0];

assign cr0_alu = cr_alu[31:28];
assign cr1_alu = cr_alu[27:24];
assign cr2_alu = cr_alu[23:20];
assign cr3_alu = cr_alu[19:16];
assign cr4_alu = cr_alu[15:12];
assign cr5_alu = cr_alu[11:8];
assign cr6_alu = cr_alu[7:4];
assign cr7_alu = cr_alu[3:0];

assign cr0_we = cr0_alu_we | 
                cr0_lsu_we | 
                ((reg_uops == `REGOP_MTCRF) & cr0_wr) | 
                ((reg_uops == `REGOP_MCRXR) & cr0_wr) | 
                ((reg_uops == `REGOP_MCRF) & cr0_wr);

assign cr0_new =
		cr0_alu_we ? cr0_alu :
		cr0_lsu_we ? cr0_lsu :
		((reg_uops == `REGOP_MCRXR) & cr0_wr) ? xer[31:28]:
		((reg_uops == `REGOP_MTCRF) & cr0_wr) ? spr_dat_i[31:28]:
		((reg_uops == `REGOP_MCRF) & cr0_wr) ? 
            (cr0_rd ? cr0 :
             cr1_rd ? cr1 :
             cr2_rd ? cr2 :
             cr3_rd ? cr3 :
             cr4_rd ? cr4 :
             cr5_rd ? cr5 :
             cr6_rd ? cr6 :
             cr7) : 
		cr0;

assign cr1_we = cr1_alu_we | 
                ((reg_uops == `REGOP_MTCRF) & cr1_wr) | 
                ((reg_uops == `REGOP_MCRXR) & cr1_wr) | 
                ((reg_uops == `REGOP_MCRF) & cr1_wr);

assign cr1_new =
		cr1_alu_we ? cr1_alu :
		((reg_uops == `REGOP_MCRXR) & cr1_wr) ? xer[31:28]:
		((reg_uops == `REGOP_MTCRF) & cr1_wr) ? spr_dat_i[27:24]:
		((reg_uops == `REGOP_MCRF) & cr1_wr) ?
            (cr0_rd ? cr0 :
             cr1_rd ? cr1 :
             cr2_rd ? cr2 :
             cr3_rd ? cr3 :
             cr4_rd ? cr4 :
             cr5_rd ? cr5 :
             cr6_rd ? cr6 :
             cr7) :
		cr1;

assign cr2_we = cr2_alu_we | 
                ((reg_uops == `REGOP_MTCRF) & cr2_wr) | 
                ((reg_uops == `REGOP_MCRXR) & cr2_wr) | 
                ((reg_uops == `REGOP_MCRF) & cr2_wr);

assign cr2_new =
		cr2_alu_we ? cr2_alu :
		((reg_uops == `REGOP_MCRXR) & cr2_wr) ? xer[31:28]:
		((reg_uops == `REGOP_MTCRF) & cr2_wr) ? spr_dat_i[23:20]:
		((reg_uops == `REGOP_MCRF) & cr2_wr) ? 
            (cr0_rd ? cr0 :
             cr1_rd ? cr1 :
             cr2_rd ? cr2 :
             cr3_rd ? cr3 :
             cr4_rd ? cr4 :
             cr5_rd ? cr5 :
             cr6_rd ? cr6 :
             cr7) :
		cr2;

assign cr3_we = cr3_alu_we | 
                ((reg_uops == `REGOP_MTCRF) & cr3_wr) | 
                ((reg_uops == `REGOP_MCRXR) & cr3_wr) | 
                ((reg_uops == `REGOP_MCRF) & cr3_wr);

assign cr3_new =
		cr3_alu_we ? cr3_alu :
		((reg_uops == `REGOP_MCRXR) & cr3_wr) ? xer[31:28]:
		((reg_uops == `REGOP_MTCRF) & cr3_wr) ? spr_dat_i[19:16]:
		((reg_uops == `REGOP_MCRF) & cr3_wr) ? 
            (cr0_rd ? cr0 :
             cr1_rd ? cr1 :
             cr2_rd ? cr2 :
             cr3_rd ? cr3 :
             cr4_rd ? cr4 :
             cr5_rd ? cr5 :
             cr6_rd ? cr6 :
             cr7) :
		cr3;

assign cr4_we = cr4_alu_we | 
                ((reg_uops == `REGOP_MTCRF) & cr4_wr) | 
                ((reg_uops == `REGOP_MCRXR) & cr4_wr) | 
                ((reg_uops == `REGOP_MCRF) & cr4_wr);

assign cr4_new =
		cr4_alu_we ? cr4_alu :
		((reg_uops == `REGOP_MCRXR) & cr4_wr) ? xer[31:28]:
		((reg_uops == `REGOP_MTCRF) & cr4_wr) ? spr_dat_i[15:12]:
		((reg_uops == `REGOP_MCRF) & cr4_wr) ? 
            (cr0_rd ? cr0 :
             cr1_rd ? cr1 :
             cr2_rd ? cr2 :
             cr3_rd ? cr3 :
             cr4_rd ? cr4 :
             cr5_rd ? cr5 :
             cr6_rd ? cr6 :
             cr7) :
		cr4;

assign cr5_we = cr5_alu_we | 
                ((reg_uops == `REGOP_MTCRF) & cr5_wr) | 
                ((reg_uops == `REGOP_MCRXR) & cr5_wr) | 
                ((reg_uops == `REGOP_MCRF) & cr5_wr);

assign cr5_new =
		cr5_alu_we ? cr5_alu :
		((reg_uops == `REGOP_MCRXR) & cr5_wr) ? xer[31:28]:
		((reg_uops == `REGOP_MTCRF) & cr5_wr) ? spr_dat_i[11:8]:
		((reg_uops == `REGOP_MCRF) & cr5_wr) ? 
            (cr0_rd ? cr0 :
             cr1_rd ? cr1 :
             cr2_rd ? cr2 :
             cr3_rd ? cr3 :
             cr4_rd ? cr4 :
             cr5_rd ? cr5 :
             cr6_rd ? cr6 :
             cr7) :            
		cr5;

assign cr6_we = cr6_alu_we | 
                ((reg_uops == `REGOP_MTCRF) & cr6_wr) | 
                ((reg_uops == `REGOP_MCRXR) & cr6_wr) | 
                ((reg_uops == `REGOP_MCRF) & cr6_wr);

assign cr6_new =
		cr6_alu_we ? cr6_alu :
		((reg_uops == `REGOP_MCRXR) & cr6_wr) ? xer[31:28]:
		((reg_uops == `REGOP_MTCRF) & cr6_wr) ? spr_dat_i[7:4]:
		((reg_uops == `REGOP_MCRF) & cr6_wr) ? 
            (cr0_rd ? cr0 :
             cr1_rd ? cr1 :
             cr2_rd ? cr2 :
             cr3_rd ? cr3 :
             cr4_rd ? cr4 :
             cr5_rd ? cr5 :
             cr6_rd ? cr6 :
             cr7) :
		cr6;

assign cr7_we = cr7_alu_we | 
                ((reg_uops == `REGOP_MTCRF) & cr7_wr) | 
                ((reg_uops == `REGOP_MCRXR) & cr7_wr) | 
                ((reg_uops == `REGOP_MCRF) & cr7_wr);

assign cr7_new =
		cr7_alu_we ? cr7_alu :
		((reg_uops == `REGOP_MCRXR) & cr7_wr) ? xer[31:28]:
		((reg_uops == `REGOP_MTCRF) & cr7_wr) ? spr_dat_i[3:0]:
		((reg_uops == `REGOP_MCRF) & cr7_wr) ? 
            (cr0_rd ? cr0 :
             cr1_rd ? cr1 :
             cr2_rd ? cr2 :
             cr3_rd ? cr3 :
             cr4_rd ? cr4 :
             cr5_rd ? cr5 :
             cr6_rd ? cr6 :
             cr7) :
		cr7;

assign cr_we = cr0_we | cr1_we | cr2_we | cr3_we | cr4_we | cr5_we | cr6_we | cr7_we;

always @(posedge clk or posedge rst) begin
	if (rst) begin
		cr0 <= #1 4'b0000;
		cr1 <= #1 4'b0000;
		cr2 <= #1 4'b0000;
		cr3 <= #1 4'b0000;
		cr4 <= #1 4'b0000;
		cr5 <= #1 4'b0000;
		cr6 <= #1 4'b0000;
		cr7 <= #1 4'b0000;
    end
	else if (cr_we) begin
		cr0 <= #1 cr0_new;
		cr1 <= #1 cr1_new;
		cr2 <= #1 cr2_new;
		cr3 <= #1 cr3_new;
		cr4 <= #1 cr4_new;
		cr5 <= #1 cr5_new;
		cr6 <= #1 cr6_new;
		cr7 <= #1 cr7_new;
	end
end

// CR output
assign cr = {cr0, cr1, cr2, cr3, cr4, cr5, cr6, cr7};

//
// XER
//
//  xer_we为重复逻辑，并无用
assign xer_sel = (spr_addr == `pippo_SPR_XER);
assign xer_we = (write_spr && xer_sel) | ca_we | ov_we | so_we | (reg_uops == `REGOP_MCRXR);

assign xer_new[`pippo_SPR_XER_SO_BITS] =
		(reg_uops == `REGOP_MCRXR)? 1'b0 :                     
		so_we ? so_new :
		(write_spr && xer_sel) ? spr_dat_i[`pippo_SPR_XER_SO_BITS] :
		xer[`pippo_SPR_XER_CA_BITS];

assign xer_new[`pippo_SPR_XER_OV_BITS] =
		(reg_uops == `REGOP_MCRXR)? 1'b0 :                     
		ov_we ? ov_new :
		(write_spr && xer_sel) ? spr_dat_i[`pippo_SPR_XER_OV_BITS] :
		xer[`pippo_SPR_XER_OV_BITS];

assign xer_new[`pippo_SPR_XER_CA_BITS] =
		(reg_uops == `REGOP_MCRXR)? 1'b0 :                     
		ca_we ? ca_new : 
		(write_spr && xer_sel) ? spr_dat_i[`pippo_SPR_XER_CA_BITS] :
		xer[`pippo_SPR_XER_CA_BITS];

always @(posedge clk or posedge rst)
	if (rst)
		xer <= #1 `pippo_SPR_XER_RESET;
	else if (xer_we)
		xer <= #1 xer_new;

// output to ALU
assign carry = xer[`pippo_SPR_XER_CA_BITS];  
assign so = xer[`pippo_SPR_XER_SO_BITS];

//
// PVR: Read Only
//  Implemented as combinational logic only

//  [TBD] coding style, to check
assign pvr_sel = (spr_addr == `pippo_SPR_PVR);

assign pvr[`pippo_SPR_PVR_OWN_BITS] = `pippo_SPR_PVR_OWN;
assign pvr[`pippo_SPR_PVR_PCF_BITS] = `pippo_SPR_PVR_PCF;
assign pvr[`pippo_SPR_PVR_CAS_BITS] = `pippo_SPR_PVR_CAS;
assign pvr[`pippo_SPR_PVR_PCV_BITS] = `pippo_SPR_PVR_PCV;
assign pvr[`pippo_SPR_PVR_AID_BITS] = `pippo_SPR_PVR_AID;

//
// Exception SPRs Interface
//
assign dear_sel = (spr_addr == `pippo_SPR_DEAR);
assign esr_sel = (spr_addr == `pippo_SPR_ESR);
assign evpr_sel = (spr_addr == `pippo_SPR_EVPR);
assign mcsr_sel = (spr_addr == `pippo_SPR_MCSR);
assign srr0_sel = (spr_addr == `pippo_SPR_SRR0);
assign srr1_sel = (spr_addr == `pippo_SPR_SRR1);
assign srr2_sel = (spr_addr == `pippo_SPR_SRR2);
assign srr3_sel = (spr_addr == `pippo_SPR_SRR3);

assign dear_we = (write_spr && dear_sel);
assign esr_we = (write_spr && esr_sel);
assign evpr_we = (write_spr && evpr_sel);
assign mcsr_we = (write_spr && mcsr_sel);
assign srr0_we = (write_spr && srr0_sel);
assign srr1_we = (write_spr && srr1_sel);
assign srr2_we = (write_spr && srr2_sel);
assign srr3_we = (write_spr && srr3_sel);

// Emulation SPRs
assign eir_sel = (spr_addr == `pippo_SPR_EIR);
assign eir_we = (write_spr && eir_sel);

//
// LR/CTR Interface
//
assign lr_sel =  (spr_addr == `pippo_SPR_LR);
assign lr_we = (write_spr && lr_sel);
assign ctr_sel =  (spr_addr == `pippo_SPR_CTR);
assign ctr_we = (write_spr && ctr_sel);

//
// timer interface
//
assign tbl_selu = (spr_addr == `pippo_SPR_TBLU);
assign tbu_selu = (spr_addr == `pippo_SPR_TBUU);
assign tbl_sel = (spr_addr == `pippo_SPR_TBL);
assign tbl_we = (write_spr && tbl_sel);
assign tbu_sel = (spr_addr == `pippo_SPR_TBU);
assign tbu_we = (write_spr && tbu_sel);
assign pit_sel = (spr_addr == `pippo_SPR_PIT);
assign pit_we = (write_spr && pit_sel);
assign tsr_sel = (spr_addr == `pippo_SPR_TSR);
assign tsr_we = (write_spr && tsr_sel);
assign tcr_sel = (spr_addr == `pippo_SPR_TCR);
assign tcr_we = (write_spr && tcr_sel);

//
// DSU interface
//
assign  dsurx_sel = (spr_addr == `pippo_SPR_DSURX);
assign  dsurx_we = (write_spr && dsurx_sel);
assign  dsutx_sel = (spr_addr == `pippo_SPR_DSUTX);
assign  dsutx_we = (write_spr && dsutx_sel);
assign  dsuctrl_sel = (spr_addr == `pippo_SPR_DSUCTRL);
assign  dsuctrl_we = (write_spr && dsuctrl_sel);
assign  dsusta_sel = (spr_addr == `pippo_SPR_DSUSTA);
assign  dsusta_we = (write_spr && dsusta_sel);

//
// MTSPR/MFSPR interface
//

always @(reg_uops or spr_addr or msr or cr or pvr or xer or ctr or lr or
         dear or esr or evpr or mcsr or srr0 or srr1 or srr2 or srr3 or
         usprg0 or sprg0 or sprg1 or sprg2 or sprg3 or sprg4 or sprg5 or 
         sprg6 or sprg7 or dsurx or dsutx or dsuctrl or dsusta or
         tbl or tbu or  pit or tsr or tcr or
         dear_sel or esr_sel or evpr_sel or mcsr_sel or srr0_sel or srr1_sel or 
         srr2_sel or srr3_sel or ctr_sel or pvr_sel or xer_sel or lr_sel or
         usprg0_sel or sprg0_sel or sprg1_sel or sprg2_sel or sprg3_sel or
         sprg4_sel or sprg5_sel or sprg6_sel or sprg7_sel or 
         sprg4_selu or sprg5_selu or sprg6_selu or sprg7_selu or
         tbl_sel or tbl_selu or tbu_sel or tbu_selu or pit_sel or tsr_sel or tcr_sel or
         eir or eir_sel or 
         dsurx_sel or dsutx_sel or dsuctrl_sel or dsusta_sel) begin
         
    write_spr = 1'b0;
	read_spr = 1'b0;
	spr_wb_dat = 32'b0;
	case (reg_uops)	// synopsys parallel_case	

        `REGOP_MFMSR : begin
			write_spr = 1'b0;
			read_spr = 1'b1;
			spr_wb_dat = msr;
		end		
		
		`REGOP_MFCR : begin
			write_spr = 1'b0;
			read_spr = 1'b0;
			spr_wb_dat = cr;
		end		
		
		`REGOP_MFSPR : begin                  
			write_spr = 1'b0;
			read_spr = 1'b1;
			casex (1) // synopsys parallel_case
				pvr_sel: 
				    spr_wb_dat = pvr;
                xer_sel: 
                    spr_wb_dat = xer; 
                ctr_sel:
                    spr_wb_dat = ctr; 
                lr_sel:
                    spr_wb_dat = lr; 

                usprg0_sel:
                    spr_wb_dat = usprg0; 
                sprg0_sel:
                    spr_wb_dat = sprg0; 
                sprg1_sel:
                    spr_wb_dat = sprg1; 
                sprg2_sel:
                    spr_wb_dat = sprg2; 
                sprg3_sel:
                    spr_wb_dat = sprg3; 
                sprg4_sel, sprg4_selu:
                    spr_wb_dat = sprg4; 
                sprg5_sel, sprg5_selu:
                    spr_wb_dat = sprg5; 
                sprg6_sel, sprg6_selu:
                    spr_wb_dat = sprg6; 
                sprg7_sel, sprg7_sel:
                    spr_wb_dat = sprg7; 
                    
                dear_sel:
                    spr_wb_dat = dear; 
                esr_sel:
                    spr_wb_dat = esr; 
                evpr_sel:
                    spr_wb_dat = evpr; 
                mcsr_sel:
                    spr_wb_dat = mcsr; 
                srr0_sel:
                    spr_wb_dat = srr0; 
                srr1_sel:
                    spr_wb_dat = srr1; 
                srr2_sel:
                    spr_wb_dat = srr2; 
                srr3_sel:
                    spr_wb_dat = srr3; 

                eir_sel:
                    spr_wb_dat = eir;
                    
                tbl_sel, tbl_selu:
                    spr_wb_dat = tbl; 
                tbu_sel, tbu_selu:
                    spr_wb_dat = tbu; 
                pit_sel:
                    spr_wb_dat = pit; 
                tsr_sel:
                    spr_wb_dat = tsr; 
                tcr_sel:
                    spr_wb_dat = tcr;  

                dsurx_sel:
                    spr_wb_dat = {24'd0, dsurx}; 
                dsutx_sel:
                    spr_wb_dat = {24'd0, dsutx}; 
                dsuctrl_sel:
                    spr_wb_dat = {24'd0, dsuctrl}; 
                dsusta_sel:
                    spr_wb_dat = {24'd0, dsusta}; 
                    
				default:
					spr_wb_dat = 32'd0;     // how to deal with CCR0/CCR1 access, to support legacy binary
			endcase
		end
                    
		`REGOP_MTSPR : begin
			write_spr = 1'b1;
			read_spr = 1'b0;
			spr_wb_dat = 32'b0;
		end		

		default : begin
			write_spr = 1'b0;
			read_spr = 1'b0;
			spr_wb_dat = 32'b0;
		end		
	endcase
end

endmodule
