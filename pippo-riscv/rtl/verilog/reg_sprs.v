/*
 * File:        reg_sprs.v
 * Project:     pippo
 * Designer:    fang@ali
 * Mainteiner:  fang@ali
 * Checker:
 * Description:
    一，功能描述
        a）为显性和隐性系统寄存器访问指令（CSRRW/CSRRS/CSRRC/CSRRWI/CSRRSI/CSRRCI）
           和其他需要访问的情况（中断处理），提供访问统一接口－地址和数据。
                执行mtspr时，其他模块的SPRs写数据来自spr_dat_wr_o（经Operandmux的GPR数据）
        b）实现SPRs：
            Status Register(MSR)：
                写操作－CSR访问指令，SCALL/SBREAK指令和外部中断发生，SRET指令
                读操作－mfmsr（读至gpr），中断发生（读至srr寄存器）
            其他SPRs实现分布如下：
	            中断处理相关SPRs在except模块实现，包括EPC, BADADDR, EVEC, CAUSE, K0和K1等
	            各系统模块相关的SPRs，如Cache，MMU和Timer等，分别在相应模块实现
            实现DSU的SPRs访问接口，链接uartlite模块；
                DSUTX, DSURX, DSUCTRL, DSUSTA
        c）检查SPRs访问相关的处理器状态，发出响应中断请求sig_svm_check
    二，RISCV架构SPRs有
        Original ppc405 register sets excluding GPRs including:
            a. MSR
            b. CSRs:
            b.1) Interrupts and Exceptions (implemented at except module)
                    EPC, BADADDR, EVEC, CAUSE - Privileged
            b.2) Timer Facilities at timer units
                    CYCLES, TIME
            b.3) Performance Counters
                    INSTRET
 * Task
 *      [TBD]SPRs地址译码统一在本模块实现？然后送出we和data_wr，或rde和接收data_rd
 *          地址来自ex_inst，注意连接关系在顶层模块连接时实现，例如SPRs地址的置换
 *      [TBD]写回SPRs时，是否也要增加流水控制逻辑？
 *          1，SPRs访问指令都是单周期执行完成，进入EXE阶段，就会正常wb，不会产生wb_freeze的情况
 *          2，[TBD]SPRs访问指令可能产生中断－非supervisor态执行previlleged操作（访问supervisor的SPRs）等，需要根据flushpipe取消写回？
 *      [TBD]msr更新值是否需要送至except模块
 *      [TBD]SPRs布线会不会带来问题－过多io连接，替代方案：两次译码，例如在except模块再对spr_addr译码
 */
 
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_sprs(
		// Clk & Rst
		clk, rst,

		reg_uops, reg_addr, dat_i, spr_wb_dat,
        
		msr,
		msr_except, msr_expwe,
		sig_svm_check, 
		
		epc, badaddr, evec, cause, 
		epc_we, badaddr_we, evec_we, cause_we, 

        
        csr_time, csr_cycle, csr_count, csr_compare, 
        csr_time_we, csr_cycle_we, csr_count_we, csr_compare_we, 

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
input	[`SPR_ADDR_WIDTH-1:0]       reg_addr;	    
input	[width-1:0]		dat_i;	        
output	[width-1:0]	    spr_wb_dat;         // write-back data for mfspr-class instructions

//
// Interface with Exception Unit: expception request, SPRs access
//
output              sig_svm_check;

input 				msr_expwe; 
input	[width-1:0]	msr_except;
output	[width-1:0]	msr;

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
// timer
//
input   [width-1:0]     tbl, tbu, pit, tsr, tcr; 
output                  tbl_we, tbu_we, pit_we, tsr_we, tcr_we;

//
// interface with dsu
//
input   [7:0]           dsurx, dsutx, dsuctrl, dsusta;
output                  dsurx_we, dsutx_we, dsuctrl_we, dsusta_we;

//
// To/from SPRs at core-out building blocks
//
output	[width-1:0]		spr_dat_wr_o;	

//
// Internal regs & wires
//
reg	[width-1:0]	msr;

wire    [width-1:0]  pvr;

reg     write_spr;
reg     read_spr; 
reg	[width-1:0]	spr_wb_dat;	

wire	[`REGOP_WIDTH-1:0]	    reg_uops;
wire    [`SPR_ADDR_WIDTH-1:0]   spr_addr; 
wire            pvr_sel;

wire [width-1:0] spr_dat_i; 
wire [width-1:0] msr_new; 

//
wire [7:0]  dsurx, dsutx, dsuctrl, dsusta; 

//
wire [width-1:0] tbl, tbu, pit, tsr, tcr;

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
                    (((reg_uops == `REGOP_MTMSR) | (reg_uops == `REGOP_MFMSR)) |
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
// GP CSR
//
reg [width-1:0]  uarch0, uarch1, uarch2, uarch3, uarch4, uarch5, uarch6, uarch7;
reg [width-1:0]  uarch8, uarch9, uarch10, uarch11, uarch12, uarch13, uarch14, uarch15;

// SPRG0, SPRG1, SPRG2, SPRG3 - Privileged
assign uarch0_sel = (spr_addr == `pippo_CSR_UARCH0);
assign uarch1_sel = (spr_addr == `pippo_CSR_UARCH1);
assign uarch2_sel = (spr_addr == `pippo_CSR_UARCH2);
assign uarch3_sel = (spr_addr == `pippo_CSR_UARCH3);
assign uarch4_sel = (spr_addr == `pippo_CSR_UARCH4);
assign uarch5_sel = (spr_addr == `pippo_CSR_UARCH5);
assign uarch6_sel = (spr_addr == `pippo_CSR_UARCH6);
assign uarch7_sel = (spr_addr == `pippo_CSR_UARCH7);
assign uarch8_sel = (spr_addr == `pippo_CSR_UARCH8);
assign uarch9_sel = (spr_addr == `pippo_CSR_UARCH9);
assign uarch10_sel = (spr_addr == `pippo_CSR_UARCH10);
assign uarch11_sel = (spr_addr == `pippo_CSR_UARCH11);
assign uarch12_sel = (spr_addr == `pippo_CSR_UARCH12);
assign uarch13_sel = (spr_addr == `pippo_CSR_UARCH13);
assign uarch14_sel = (spr_addr == `pippo_CSR_UARCH14);
assign uarch15_sel = (spr_addr == `pippo_CSR_UARCH15);

assign uarch0_we = write_spr & uarch0_sel;
assign uarch1_we = write_spr & uarch1_sel;
assign uarch2_we = write_spr & uarch2_sel;
assign uarch3_we = write_spr & uarch3_sel;
assign uarch4_we = write_spr & uarch4_sel;
assign uarch5_we = write_spr & uarch5_sel;
assign uarch6_we = write_spr & uarch6_sel;
assign uarch7_we = write_spr & uarch7_sel;
assign uarch8_we = write_spr & uarch8_sel;
assign uarch9_we = write_spr & uarch9_sel;
assign uarch10_we = write_spr & uarch10_sel;
assign uarch11_we = write_spr & uarch11_sel;
assign uarch12_we = write_spr & uarch12_sel;
assign uarch13_we = write_spr & uarch13_sel;
assign uarch14_we = write_spr & uarch14_sel;
assign uarch15_we = write_spr & uarch15_sel;

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch0 <= #1 `pippo_CSR_UARCH0_RESET;
	else if (uarch0_we)
		uarch0 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch1 <= #1 `pippo_CSR_UARCH1_RESET;
	else if (uarch1_we)
		uarch1 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch2 <= #1 `pippo_CSR_UARCH2_RESET;
	else if (uarch2_we)
		uarch2 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch3 <= #1 `pippo_CSR_UARCH3_RESET;
	else if (uarch3_we)
		uarch3 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch4 <= #1 `pippo_CSR_UARCH4_RESET;
	else if (uarch4_we)
		uarch4 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch5 <= #1 `pippo_CSR_UARCH5_RESET;
	else if (uarch5_we)
		uarch5 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch6 <= #1 `pippo_CSR_UARCH6_RESET;
	else if (uarch6_we)
		uarch6 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch7 <= #1 `pippo_CSR_UARCH7_RESET;
	else if (uarch7_we)
		uarch7 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch8 <= #1 `pippo_CSR_UARCH8_RESET;
	else if (uarch8_we)
		uarch8 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch9 <= #1 `pippo_CSR_UARCH9_RESET;
	else if (uarch9_we)
		uarch9 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch10 <= #1 `pippo_CSR_UARCH10_RESET;
	else if (uarch10_we)
		uarch10 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch11 <= #1 `pippo_CSR_UARCH11_RESET;
	else if (uarch11_we)
		uarch11 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch12 <= #1 `pippo_CSR_UARCH12_RESET;
	else if (uarch12_we)
		uarch12 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch13 <= #1 `pippo_CSR_UARCH13_RESET;
	else if (uarch13_we)
		uarch13 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch14 <= #1 `pippo_CSR_UARCH14_RESET;
	else if (uarch14_we)
		uarch14 <= #1 spr_dat_i;
end

always @(posedge clk or posedge rst) begin
	if (rst)
		uarch15 <= #1 `pippo_CSR_UARCH15_RESET;
	else if (uarch15_we)
		uarch15 <= #1 spr_dat_i;
end

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

always @(reg_uops or spr_addr or msr or pvr or
         dear or esr or evpr or mcsr or srr0 or srr1 or srr2 or srr3 or
         usprg0 or sprg0 or sprg1 or sprg2 or sprg3 or sprg4 or sprg5 or 
         sprg6 or sprg7 or dsurx or dsutx or dsuctrl or dsusta or
         tbl or tbu or  pit or tsr or tcr or
         dear_sel or esr_sel or evpr_sel or mcsr_sel or srr0_sel or srr1_sel or 
         srr2_sel or srr3_sel or pvr_sel or
         usprg0_sel or sprg0_sel or sprg1_sel or sprg2_sel or sprg3_sel or
         sprg4_sel or sprg5_sel or sprg6_sel or sprg7_sel or 
         sprg4_selu or sprg5_selu or sprg6_selu or sprg7_selu or
         tbl_sel or tbl_selu or tbu_sel or tbu_selu or pit_sel or tsr_sel or tcr_sel or
         dsurx_sel or dsutx_sel or dsuctrl_sel or dsusta_sel) begin
         
    write_spr = 1'b0;
	read_spr = 1'b0;
	spr_wb_dat = 64'b0;
	case (reg_uops)	// synopsys parallel_case	

        `REGOP_MFMSR : begin
			write_spr = 1'b0;
			read_spr = 1'b1;
			spr_wb_dat = msr;
		end		
				
		`REGOP_MFSPR : begin                  
			write_spr = 1'b0;
			read_spr = 1'b1;
			casex (1) // synopsys parallel_case
				pvr_sel: 
				    spr_wb_dat = pvr;
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
                    spr_wb_dat = {56'd0, dsurx}; 
                dsutx_sel:
                    spr_wb_dat = {56'd0, dsutx}; 
                dsuctrl_sel:
                    spr_wb_dat = {56'd0, dsuctrl}; 
                dsusta_sel:
                    spr_wb_dat = {56'd0, dsusta}; 
                    
				default:
					spr_wb_dat = 64'd0;     // how to deal with CCR0/CCR1 access, to support legacy binary
			endcase
		end
                    
		`REGOP_MTSPR : begin
			write_spr = 1'b1;
			read_spr = 1'b0;
			spr_wb_dat = 64'b0;
		end		

		default : begin
			write_spr = 1'b0;
			read_spr = 1'b0;
			spr_wb_dat = 64'b0;
		end		
	endcase
end

endmodule
