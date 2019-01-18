/*
 * File:        pippo_core.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner: 
 * Description:
 *      top module for pippo core
 */
  
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_core(

	clk, rst,

	// i-side IMX
	iimx_adr_o, iimx_rqt_o, iimx_rty_i, 
	iimx_dat_i, iimx_ack_i, iimx_err_i, iimx_adr_i,
	
	// d-side IMX
	dimx_adr_o, dimx_rqt_o, dimx_we_o, dimx_sel_o, dimx_dat_o,
	dimx_dat_i, dimx_ack_i, dimx_err_i, 

	// external interrupt
    sig_ext_ci,
    sig_ext_i,
    
    // pmu interface from timer
    rqt_core_rst, 
	rqt_sys_rst,
	rqt_chip_rst,

    // dsu interface
    dsu_rst, 
    dsu_sram_ce, dsu_sram_we, dsu_sram_addr, dsu_sram_data,   
    dsu_burn_enable, 
    txd, rxd

)/* synthesis syn_preserve=1 */;

parameter dw = `OPERAND_WIDTH;
parameter aw = `GPR_ADDR_WIDTH;

//
// I/O ports
//

input 				clk;
input 				rst;

//
// i-IMX
//
output	[31:0]		iimx_adr_o;
output				iimx_rqt_o;
input	[31:0]		iimx_dat_i;
input				iimx_ack_i;
input				iimx_rty_i;
input				iimx_err_i;
input	[31:0]		iimx_adr_i;

//
// d-IMX
//
output	[31:0]		dimx_adr_o;
output				dimx_rqt_o;
output				dimx_we_o;
output	[3:0]		dimx_sel_o;
output	[31:0]		dimx_dat_o;
input	[31:0]		dimx_dat_i;
input				dimx_ack_i;
input				dimx_err_i;

//
// interrupt exceptions
//
input				sig_ext_ci;
input				sig_ext_i;

//
// dsu
//
input   dsu_rst;

output  dsu_sram_ce;
output  dsu_sram_we;
output  [`IOCM_Word_BW-1:0]   dsu_sram_addr;
output  [31:0]  dsu_sram_data;
input   dsu_burn_enable;

input   rxd;
output  txd;

//
// pmu
//
output  rqt_core_rst;
output  rqt_sys_rst;
output  rqt_chip_rst;

//
// interconnections
//

// core I/O
wire                clk;
wire                rst;

wire	[31:0]		iimx_adr_o;
wire				iimx_rqt_o;
wire	[31:0]		iimx_dat_i;
wire				iimx_ack_i;
wire				iimx_rty_i;
wire				iimx_err_i;
wire	[31:0]		iimx_adr_i;

wire	[31:0]		dimx_adr_o;
wire				dimx_rqt_o;
wire				dimx_we_o;
wire	[3:0]		dimx_sel_o;
wire	[31:0]		dimx_dat_o;
wire	[31:0]		dimx_dat_i;
wire				dimx_ack_i;
wire				dimx_err_i;

// instruction fetch
wire   [29:0]       npc_branch;         
wire                npc_branch_valid;   
wire   [31:0]       npc_except;         
wire                npc_except_valid;   
wire                id_valid;
wire	[31:0]		id_inst;
wire	[29:0]		id_cia;
wire    [29:0]		id_snia;
wire				id_sig_ibuserr;     

// instruction decode 
wire                ex_valid;
wire	[31:0]		ex_inst;
wire	[29:0]		ex_cia;
wire    [29:0]		ex_snia;

wire	[31:0]			    ex_imm; 
wire	[`OPSEL_WIDTH-1:0]	ex_sel_a;
wire	[`OPSEL_WIDTH-1:0]	ex_sel_b;
wire	[29:0]			    branch_addrofs;
wire                        reg_zero;     
        
wire	[`SPR_ADDR_WIDTH-1:0]	reg_addr;  
wire	[`ALUUOPS_WIDTH-1:0]	alu_uops;
wire	[`BPUUOPS_WIDTH-1:0]	bpu_uops;
wire	[`LSUUOPS_WIDTH-1:0]	lsu_uops;
wire	[`REGUOPS_WIDTH-1:0]	reg_uops;
wire	[`CRUOPS_WIDTH-1:0]	    cr_uops;
wire	[`RFWBUOPS_WIDTH-1:0]	rfwb_uops;

wire	[`MULTICYCLE_WIDTH-1:0]	multicycle_cnt;

// GPR access
wire				rf_rda;
wire				rf_rdb;
wire				rf_rdc;
wire	[aw-1:0] 	rf_addra;
wire	[aw-1:0] 	rf_addrb;
wire	[aw-1:0] 	rf_addrc;
wire	[dw-1:0]	rf_dataa;
wire	[dw-1:0]	rf_datab;
wire	[dw-1:0]	rf_datac;

wire	[aw-1:0]	rf_addrwa;
wire	[aw-1:0]	rf_addrwb;
wire	[dw-1:0]	rf_datawa;
wire	[dw-1:0]	rf_datawb;

// alu
wire    [31:0]  cr_alu;
wire    [7:0]   cr_alu_we;

// lsu
wire [31:0] lsu_addr;
wire [3:0]  cr0_lsu;

// operandmux
wire    [dw-1:0]	wb_fwd;
wire    [dw-1:0]	bus_a;
wire    [dw-1:0]	bus_b;

// write-back
wire	[dw-1:0]	alu_dataout;
wire	[dw-1:0]	lsu_dataout;
wire	[dw-1:0]	sprs_dataout;

// MSR/CR/SPRs
wire    [31:0]      msr; 
wire    [31:0]      cr; 
wire    [31:0]      lr; 
wire    [31:0]      ctr; 

wire    [31:0]      spr_dat_o;

// except
wire			    sig_rfi;
wire                sig_rfci;
wire				sig_syscall;
wire				sig_eieio;
wire                sig_isync;
wire                sig_sync;
wire				sig_illegal;
wire                sig_emulate;
wire                sig_ibuserr;
wire    [31:0]      msr_except;
wire	[31:0]		dear;
wire	[31:0]		esr;
wire	[31:0]		evpr;
wire	[31:0]		mcsr;
wire	[31:0]		srr0;
wire	[31:0]		srr1;
wire	[31:0]		srr2;
wire	[31:0]		srr3;
wire    [31:0]      eir;

// timer
wire    [31:0]      tbl, tbu, pit, tsr, tcr;

// dsu
wire    [7:0]       dsurx, dsutx, dsuctrl, dsusta;
wire    [`IOCM_Word_BW-1:0]         dsu_sram_addr; 

// pipeline control
wire				if_stall;
wire				lsu_stall;
wire				lsu_done;
wire                flush_branch;
wire                flush_except;

wire				pc_freeze;
wire				if_freeze;
wire				id_freeze;
wire				ex_freeze;
wire				wb_freeze;
wire				flushpipe;

//
// instruction fetch
//
pippo_if pippo_if(
	.clk(clk),
	.rst(rst),

    .iimx_adr_o(iimx_adr_o),
	.iimx_rqt_o(iimx_rqt_o),

	.iimx_rty_i(iimx_rty_i),
	.iimx_ack_i(iimx_ack_i),
	.iimx_err_i(iimx_err_i),
	.iimx_dat_i(iimx_dat_i),
	.iimx_adr_i(iimx_adr_i),
	
	.npc_branch(npc_branch), 
	.npc_except(npc_except), 
	.npc_branch_valid(npc_branch_valid), 
	.npc_except_valid(npc_except_valid), 

	.if_stall(if_stall), 
	.pc_freeze(pc_freeze), 
	.if_freeze(if_freeze), 
	.id_freeze(id_freeze), 
	.flushpipe(flushpipe), 
	
	.id_valid(id_valid),
	.id_inst(id_inst), 
	.id_cia(id_cia), 
	.id_snia(id_snia),
	
	.id_sig_ibuserr(id_sig_ibuserr)
);

//
// inst decoder
//
pippo_id pippo_id(

	.clk(clk),
	.rst(rst),

	.id_valid(id_valid),
	.id_inst(id_inst), 
	.id_cia(id_cia), 
	.id_snia(id_snia),
	.ex_valid(ex_valid),
	.ex_inst(ex_inst), 
	.ex_cia(ex_cia), 
	.ex_snia(ex_snia),
	
	.gpr_addr_rda(rf_addra), 
	.gpr_addr_rdb(rf_addrb), 
	.gpr_addr_rdc(rf_addrc), 
	.gpr_rda_en(rf_rda), 
	.gpr_rdb_en(rf_rdb), 
	.gpr_rdc_en(rf_rdc), 
	.ex_imm(ex_imm), 
	.ex_sel_a(ex_sel_a), 
	.ex_sel_b(ex_sel_b), 
	
	.ex_branch_addrofs(branch_addrofs), 	
	.reg_zero(reg_zero),
	.ex_spr_addr(reg_addr), 
	
    .set_atomic(set_atomic), 
    .clear_atomic(clear_atomic),   

	.ex_bpu_uops(bpu_uops), 
	.ex_alu_uops(alu_uops), 
	.ex_cr_uops(cr_uops), 
	.ex_lsu_uops(lsu_uops), 
	.ex_reg_uops(reg_uops), 
	
	.ex_rfwb_uops(rfwb_uops), 
	.ex_gpr_addr_wra(rf_addrwa), 
	.ex_gpr_addr_wrb(rf_addrwb),     //[TBD] for extending wb
	
	.multicycle_cnt(multicycle_cnt),
	.id_freeze(id_freeze), 
	.ex_freeze(ex_freeze), 
	.wb_freeze(wb_freeze), 
	.flushpipe(flushpipe), 
		
	
	.sig_syscall(sig_syscall),
	.sig_rfi(sig_rfi), 
	.sig_rfci(sig_rfci), 
	.sig_eieio(sig_eieio), 
	.sig_isync(sig_isync), 
	.sig_sync(sig_sync), 
	.sig_illegal(sig_illegal), 
	.sig_emulate(sig_emulate), 
	.id_sig_ibuserr(id_sig_ibuserr), 
	.sig_ibuserr(sig_ibuserr)
);

//
// register file
//
reg_gprs reg_gprs(
	.clk(clk),
	.rst(rst),
    
    .id_freeze(id_freeze), 
    .ex_freeze(ex_freeze), 
    .wb_freeze(wb_freeze),
	.flushpipe(flushpipe),
	
	.addra(rf_addra),
	.rda(rf_rda),
	.dataa(rf_dataa),

	.addrb(rf_addrb),
	.rdb(rf_rdb),
	.datab(rf_datab),

	.addrc(rf_addrc),
	.rdc(rf_rdc),
	.datac(rf_datac),
	
	.addrwa(rf_addrwa),
	.datawa(rf_datawa),	
	.wea(rfwb_uops[0]),         // [TBD] to add extend wb logic

	.addrwb(rf_addrwb),
	.datawb(lsu_addr),	        // for update style memory access inst.
	.web(rfwb_uops[1])          // [TBD] to add extend wb logic
	
);

//
// operand muxes
//
pippo_operandmuxes pippo_operandmuxes(
	.rf_dataa(rf_dataa),
	.rf_datab(rf_datab),
	.wb_fwd(wb_fwd),
	.imm(ex_imm),
	.sel_a(ex_sel_a),
	.sel_b(ex_sel_b),
	.bus_a(bus_a),
	.bus_b(bus_b)
);

//
// alu
//
pippo_alu pippo_alu(
    .clk(clk),
    .rst(rst),

	.alu_uops(alu_uops), 
	.cr_uops(cr_uops),
	
	.bus_a(bus_a),
	.bus_b(bus_b),
	.reg_zero(reg_zero),
	.sh_mb_me(ex_inst[15:1]), 
	.trap_to(ex_inst[25:21]),
	
	.carry(carry),
	.so(so),		
	.result(alu_dataout),
	.sig_trap(sig_trap),
	
	.so_new(so_new), 
	.so_we(so_we),
    .ov_new(ov_new), 
    .ov_we(ov_we),
    .ca_new(ca_new), 
    .ca_we(ca_we),
    
	.cr_addr(ex_inst[25:11]), 
	.cr(cr), 
	.cr_alu(cr_alu), 
	.cr_alu_we(cr_alu_we)
);

//
// lsu
//
pippo_lsu pippo_lsu(
    .clk(clk),
    .rst(rst),
	.lsu_uops(lsu_uops),
	.addrbase(bus_a),
	.addrofs(bus_b),      
	.lsu_addr(lsu_addr), 
	.reg_zero(reg_zero),
	.lsu_datain(rf_datac),
	.lsu_dataout(lsu_dataout),
	.lsu_stall(lsu_stall),
	.lsu_done(lsu_done),

    .so(so),
    .cr0_lsu(cr0_lsu),     
    .cr0_lsu_we(cr0_lsu_we), 
    
    .set_atomic(set_atomic), 
    .clear_atomic(clear_atomic),   
    
	.dimx_adr_o(dimx_adr_o),
	.dimx_rqt_o(dimx_rqt_o),
	.dimx_we_o(dimx_we_o),
	.dimx_sel_o(dimx_sel_o),
	.dimx_dat_o(dimx_dat_o),
	.dimx_dat_i(dimx_dat_i),
	.dimx_ack_i(dimx_ack_i),
	.dimx_err_i(dimx_err_i),
	
    .sig_align(sig_align),
	.sig_dbuserr(sig_dbuserr)
);

//
// bpu: branch processing unit
//
pippo_bpu pippo_bpu(
    .clk(clk),
    .rst(rst),    
    .bpu_uops(bpu_uops),
    .bo_field(ex_inst[25:21]),
    .bi_field(ex_inst[20:16]),
    .branch_addrofs(branch_addrofs), 
    .cia(ex_cia), 
    .snia(ex_snia),     
    .cr(cr), 
    .lr(lr), 
    .ctr(ctr), 
    .lr_we(lr_we), 
    .ctr_we(ctr_we), 
    .spr_dat_i(spr_dat_o),     
    .npc_branch_valid(npc_branch_valid), 
    .npc_branch(npc_branch), 
    .flush_branch(flush_branch)
);

//
// SPRs
//
pippo_sprs pippo_sprs(
	.clk(clk),
	.rst(rst),
    .reg_uops(reg_uops),
    .reg_addr(reg_addr),
    .dat_i(bus_a),
    .spr_wb_dat(sprs_dataout),
    
    .carry(carry),
    .so(so),
    .so_new(so_new), 
    .so_we(so_we),
    .ov_new(ov_new), 
    .ov_we(ov_we), 
    .ca_new(ca_new), 
    .ca_we(ca_we), 
    .cr_addr(ex_inst[25:11]),
    .cr(cr),
    .cr_alu(cr_alu),
    .cr_alu_we(cr_alu_we),
    .cr0_lsu(cr0_lsu),     
    .cr0_lsu_we(cr0_lsu_we), 
    
    .msr(msr),
    .msr_except(msr_except), 
    .msr_expwe(msr_expwe),
    .sig_svm_check(sig_svm_check),
    .dear(dear), 
    .esr(esr), 
    .evpr(evpr), 
    .mcsr(mcsr), 
    .srr0(srr0), 
    .srr1(srr1), 
    .srr2(srr2), 
    .srr3(srr3), 
    .dear_we(dear_we), 
    .esr_we(esr_we), 
    .evpr_we(evpr_we), 
    .mcsr_we(mcsr_we), 
    .srr0_we(srr0_we), 
    .srr1_we(srr1_we), 
    .srr2_we(srr2_we), 
    .srr3_we(srr3_we),

    .eir_we(wir_we),
    .eir(eir),
    
    .lr(lr), 
    .ctr(ctr),
    .lr_we(lr_we), 
    .ctr_we(ctr_we),
    
    .tbl(tbl), 
    .tbu(tbu), 
    .pit(pit), 
    .tsr(tsr), 
    .tcr(tcr), 
    .tbl_we(tbl_we), 
    .tbu_we(tbu_we), 
    .pit_we(pit_we), 
    .tsr_we(tsr_we), 
    .tcr_we(tcr_we), 

    .dsurx(dsurx), 
    .dsutx(dsutx), 
    .dsuctrl(dsuctrl), 
    .dsusta(dsusta), 
    .dsurx_we(dsurx_we), 
    .dsutx_we(dsutx_we), 
    .dsuctrl_we(dsuctrl_we), 
    .dsusta_we(dsusta_we), 

    .spr_dat_wr_o(spr_dat_o)
);

//
// write-back and operand forwarding muxes
//
pippo_wbmux pippo_wbmux(
	.clk(clk),
	.rst(rst),
	.wb_freeze(wb_freeze),
	.rfwb_op(rfwb_uops),
	.muxin_a(alu_dataout),
	.muxin_b(lsu_dataout),
	.muxin_c(sprs_dataout),
	.muxin_d(lsu_addr),
	.muxout(rf_datawa),
	.muxreg(wb_fwd)
);

//
// pipeline control
//
pippo_pipectrl pippo_pipectrl(
    .flush_except(flush_except),
    .flush_branch(flush_branch),
    .flushpipe(flushpipe), 
	.if_stall(if_stall),
	.lsu_stall(lsu_stall),
	.multicycle_cnt(multicycle_cnt),
	.asyn_stall(asyn_stall),
	.pc_freeze(pc_freeze),
	.if_freeze(if_freeze),
	.id_freeze(id_freeze),
	.ex_freeze(ex_freeze),
	.wb_freeze(wb_freeze)
);

//
// exception processing block
//
pippo_except pippo_except(
	.clk(clk),
	.rst(rst),

	.sig_ibuserr(sig_ibuserr),
	.sig_dbuserr(sig_dbuserr),
	.sig_illegal(sig_illegal),
	.sig_emulate(sig_emulate),
	.sig_align(sig_align),
	.sig_syscall(sig_syscall),
    .sig_ext_ci(sig_ext_ci),
    .sig_ext_i(sig_ext_i),
	.sig_rfi(sig_rfi),
	.sig_rfci(sig_rfci),
    .sig_trap(sig_trap),
	.sig_eieio(sig_eieio), 
	.sig_isync(sig_isync), 
	.sig_sync(sig_sync), 
	.sig_pit(sig_pit), 
	.sig_fit(sig_fit), 
	.sig_watchdog(sig_watchdog),
    
	.npc_except(npc_except), 
	.npc_exp_valid(npc_except_valid), 

    .asyn_stall(asyn_stall),
    .wb_freeze(wb_freeze),
	.flush_except(flush_except),
	
    .ex_inst(ex_inst),
    .eir_we(wir_we),
    .eir(eir),
    
    .ex_cia(ex_cia),
    .ex_snia(ex_snia), 
    .msr(msr),
	.msr_except(msr_except), 
	.msr_expwe(msr_expwe),
    .sig_svm_check(sig_svm_check),	
	.dear(dear), 
	.esr(esr), 
	.evpr(evpr), 
	.mcsr(mcsr), 
	.srr0(srr0), 
	.srr1(srr1), 
	.srr2(srr2), 
	.srr3(srr3), 
	.dear_we(dear_we), 
	.esr_we(esr_we), 
	.evpr_we(evpr_we), 
	.mcsr_we(mcsr_we), 
	.srr0_we(srr0_we), 
	.srr1_we(srr1_we), 
	.srr2_we(srr2_we), 
	.srr3_we(srr3_we),
    .spr_dat_i(spr_dat_o), 
	.lsu_addr(lsu_addr)
);

//
// timer
//
pippo_timer pippo_timer(
	.clk(clk), 
	.rst(rst), 

    .tbl(tbl), 
    .tbu(tbu), 
    .pit(pit), 
    .tsr(tsr), 
    .tcr(tcr), 
    .tbl_we(tbl_we), 
    .tbu_we(tbu_we), 
    .pit_we(pit_we), 
    .tsr_we(tsr_we), 
    .tcr_we(tcr_we), 
    .spr_dat_i(spr_dat_o),     
    
	.sig_pit(sig_pit), 
	.sig_fit(sig_fit), 
	.sig_watchdog(sig_watchdog),
	.rqt_core_rst(rqt_core_rst), 
	.rqt_sys_rst(rqt_sys_rst), 
	.rqt_chip_rst(rqt_chip_rst)
);

//
// dsu - uartlite
//
dsu_uartlite dsu_uartlite(
    .clk(clk),
    .rst(dsu_rst),
    
    .rxd(rxd),
    .txd(txd),
      
    .reg_rxdata(dsurx), 
    .reg_txdata(dsutx), 
    .reg_ctrl(dsuctrl), 
    .reg_sta(dsusta), 
    .reg_rxdata_we(dsurx_we), 
    .reg_txdata_we(dsutx_we), 
    .reg_ctrl_we(dsuctrl_we), 
    .reg_sta_we(dsusta_we), 
    .spr_dat_i(spr_dat_o[7:0]), 
    
    .sram_ce(dsu_sram_ce),
    .sram_we(dsu_sram_we),
    .sram_addr(dsu_sram_addr),
    .sram_wdata(dsu_sram_data),
    
    .download_enable(dsu_burn_enable)
    
);

endmodule
