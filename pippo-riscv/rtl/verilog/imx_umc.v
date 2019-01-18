/*
 * File:        imx_umc.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description:
 *      unified (inst/data) on-chip memory controller hooked to IMX
 */

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module imx_umc(

	clk, rst,

	iimx_adr_i, iimx_rqt_i, 	
	iimx_rty_o, iimx_ack_o, iimx_err_o, iimx_dat_o, iimx_adr_o, 

//	icbu_adr_o, icbu_cycstb_o, icbu_ci_o, icbu_sel_o, icbu_tag_o,
//	icbu_dat_i, icbu_ack_i, icbu_rty_i, icbu_err_i, icbu_tag_i,

	dimx_adr_i, dimx_rqt_i, dimx_we_i, dimx_dat_i, dimx_sel_i,	
	dimx_dat_o, dimx_ack_o, dimx_err_o,

//	dcbu_adr_o, dcbu_cycstb_o, dcbu_ci_o, dcbu_we_o, dcbu_sel_o, dcbu_tag_o, dcbu_dat_o,
//	dcbu_dat_i, dcbu_ack_i, dcbu_rty_i, dcbu_err_i, dcbu_tag_i, 

    // uartlite backdoor
    dsu_sram_ce, dsu_sram_we, dsu_sram_addr, dsu_sram_data,   
    dsu_burn_enable
);

parameter dw = `OPERAND_WIDTH;

input				clk;
input				rst;

//
// fetcher-umc interface
//
input	[31:0]		iimx_adr_i;
input				iimx_rqt_i;

output	[dw-1:0]    iimx_dat_o;
output				iimx_ack_o;
output				iimx_err_o;
output	[31:0]	    iimx_adr_o;
output				iimx_rty_o;

//
// umc-icbu interface
//
//output	[31:0]		icbu_adr_o;
//output				icbu_cycstb_o;
//output				icbu_ci_o;
//output	[3:0]		icbu_sel_o;
//output	[3:0]		icbu_tag_o;
//input	    [31:0]		icbu_dat_i;
//input				    icbu_ack_i;
//input				    icbu_rty_i;
//input				    icbu_err_i;
//input	    [3:0]		icbu_tag_i;

//
// lsu-umc interface
//
input	[31:0]		dimx_adr_i;
input				dimx_rqt_i;
input	[3:0]		dimx_sel_i;
input               dimx_we_i;
input   [dw-1:0]    dimx_dat_i;

output	[dw-1:0]	dimx_dat_o;
output				dimx_ack_o;
output				dimx_err_o;

//
// umc-dcbu interface
//
//output    [31:0]      dcbu_adr_o;
//output                dcbu_cycstb_o;
//output                dcbu_ci_o;
//output                dcbu_we_o;
//output    [3:0]       dcbu_sel_o;
//output    [3:0]       dcbu_tag_o;
//output    [dw-1:0]    dcbu_dat_o;
//input     [dw-1:0]    dcbu_dat_i;
//input                 dcbu_ack_i;
//input                 dcbu_rty_i;
//input                 dcbu_err_i;
//input     [3:0]       dcbu_tag_i;

//
// dsu interface
//
input   dsu_sram_ce;
input   dsu_sram_we;
input   [`UOCM_Word_BW-1:0]   dsu_sram_addr;
input   [31:0]  dsu_sram_data;
input   dsu_burn_enable;

//
// regs and wires
//
wire    [3:0]   dimx_sel_i;
wire            imc_hit;
wire            dmc_hit;
wire            uocm_en;
wire            uocm_we;
wire    [31:0]  uocm_addr;
wire    [dw-1:0]  uocm_di, ram_di;
wire    [dw-1:0]  ram_do;
wire    [`UOCM_Word_BW-1:0]ram_addr;

reg [2:0]   state;
reg         dmc_ack;
reg         imc_ack;

//
// address decoder
//
assign imc_hit = (iimx_adr_i & `pippo_IMC_MASK) == `pippo_IMC_ADDR;
assign dmc_hit = (dimx_adr_i & `pippo_DMC_MASK) == `pippo_DMC_ADDR;
assign imc_rqt_valid = imc_hit & iimx_rqt_i;
assign dmc_rqt_valid = dmc_hit & dimx_rqt_i;

//
// uocm access signals
//
assign uocm_addr = (dmc_rqt_valid) ? dimx_adr_i : iimx_adr_i;
assign uocm_en = imc_rqt_valid | dmc_rqt_valid;

// special case for byte/halfword/word store access
//  cycle 1: store_nonword assert for two cycles
//           reading memory to fetch the whole word
//  cycle 2: store_nonword_reg assert for one cycle
//           writing inserted word back to memory
//  cycle 3: dmc_ack assert, the transaction complete
//  
reg store_nonword_reg;

// "!dmc_ack" is to disassert store_nonword at cycle 3, 
// if not, store_nonword_reg will assert at cycle 4.
assign store_nonword = !(&dimx_sel_i) & dmc_rqt_valid & dimx_we_i & !dmc_ack; 

always @(posedge clk or posedge rst) begin
    if (rst)
        store_nonword_reg <= #1 1'b0; 
    else if (store_nonword_reg)
        store_nonword_reg <= #1 1'b0;
    else          
        store_nonword_reg <= #1 store_nonword; 
end

reg [31:0]  dat_nonword;
always @(*) begin
    case(dimx_sel_i)
        8'b00000001:    dat_nonword = {ram_do[63:8], dimx_dat_i[7:0] };        
        8'b00000010:    dat_nonword = {ram_do[63:16], dimx_dat_i[15:8], ram_do[7:0]};
        8'b00000100:    dat_nonword = {ram_do[63:24], dimx_dat_i[23:16], ram_do[15:0]};
        8'b00001000:    dat_nonword = {ram_do[63:32], dimx_dat_i[31:24], ram_do[23:0]};
        8'b00010000:    dat_nonword = {ram_do[63:40], dimx_dat_i[39:32], ram_do[31:0]};
        8'b00100000:    dat_nonword = {ram_do[63:48], dimx_dat_i[47:40], ram_do[39:0]};
        8'b01000000:    dat_nonword = {ram_do[63:56], dimx_dat_i[55:48], ram_do[47:0]};
        8'b10000000:    dat_nonword = {dimx_dat_i[63:56], ram_do[55:0]};
        8'b00000011:    dat_nonword = {ram_do[63:16], dimx_dat_i[15:0] };        
        8'b00001100:    dat_nonword = {ram_do[63:32], dimx_dat_i[31:16], ram_do[15:0]};
        8'b00110000:    dat_nonword = {ram_do[63:48], dimx_dat_i[47:32], ram_do[31:0]};
        8'b11000000:    dat_nonword = {dimx_dat_i[63:48], ram_do[47:0]};
        8'b00001111:    dat_nonword = {ram_do[63:32], dimx_dat_i[31:0] };        
        8'b11110000:    dat_nonword = {dimx_dat_i[63:32], ram_do[31:0]};
        8'b11111111:    dat_nonword = dimx_dat_i[63:0];        
        default: dat_nonword = dimx_dat_i[63:0];;
    endcase
end

// write happens:
//  1, normal store: only at the 1st cycle, dmc_ack assert at following cycle
//  2, byte/hw store: only at the 2nd cycle, dmc_ack assert at following cycle
assign uocm_we = dmc_rqt_valid & dimx_we_i & !dmc_ack & (!store_nonword | store_nonword_reg);
assign uocm_di = store_nonword ? dat_nonword : dimx_dat_i;

//
// backdoor for dsu 
//
assign ram_addr = dsu_burn_enable ? dsu_sram_addr : uocm_addr[`UOCM_Word_BW+1:2];
assign ram_ce = dsu_burn_enable ? dsu_sram_ce : uocm_en;
assign ram_we = dsu_burn_enable ? dsu_sram_we : uocm_we;
assign ram_di = dsu_burn_enable ? dsu_sram_data : uocm_di;

// unified on-chip memory
imx_uocm imx_uocm(
	.clk(clk),
    .ce(ram_ce),
	.we(ram_we),
	.oe(1'b1),
	.addr(ram_addr),
	.di(ram_di),
	.doq(ram_do)
);

//
// FSM for core response
//  assumption: the uocm must be synchronous sram
//
`define pippo_UMCFSM_IDLE	3'd0
`define pippo_UMCFSM_LOAD	3'd1
`define pippo_UMCFSM_STORE	3'd2
`define pippo_UMCFSM_FETCH	3'd4

always @(posedge clk or `pippo_RST_EVENT rst) begin
    if (rst == `pippo_RST_VALUE) begin
        state <=  `pippo_UMCFSM_IDLE;
        dmc_ack <=  1'b0;
        imc_ack <=  1'b0;
    end
    else case (state)	// synopsys parallel_case
        `pippo_UMCFSM_IDLE: begin
			if (dmc_rqt_valid & dimx_we_i & !dsu_burn_enable) begin
				state <=  `pippo_UMCFSM_STORE;
				dmc_ack <=  !dmc_ack & (!store_nonword | store_nonword_reg);
				imc_ack <=  1'b0;
			end
			else if (dmc_rqt_valid & !dsu_burn_enable) begin
				state <=  `pippo_UMCFSM_LOAD;
				dmc_ack <=  1'b1;
				imc_ack <=  1'b0;
			end
			else if (imc_rqt_valid & !dsu_burn_enable) begin
				state <=  `pippo_UMCFSM_FETCH;
				imc_ack <=  1'b1;
				dmc_ack <=  1'b0;
			end
		end
		`pippo_UMCFSM_STORE: begin
			if (dmc_rqt_valid & dimx_we_i & !dsu_burn_enable) begin
				state <=  `pippo_UMCFSM_STORE;
				dmc_ack <=  !dmc_ack & (!store_nonword | store_nonword_reg);
				imc_ack <=  1'b0;
			end
			else if (dmc_rqt_valid & !dsu_burn_enable) begin
				state <=  `pippo_UMCFSM_LOAD;
				dmc_ack <=  1'b1;
				imc_ack <=  1'b0;
			end
			else if (imc_rqt_valid & !dsu_burn_enable) begin
				state <=  `pippo_UMCFSM_FETCH;
				imc_ack <=  1'b1;
				dmc_ack <=  1'b0;
			end
			else begin
				state <=  `pippo_UMCFSM_IDLE;
				dmc_ack <=  1'b0;
				imc_ack <=  1'b0;
			end
		end
		`pippo_UMCFSM_LOAD: begin
			if (dmc_rqt_valid & dimx_we_i & !dsu_burn_enable) begin
				state <=  `pippo_UMCFSM_STORE;
				dmc_ack <=  !dmc_ack & (!store_nonword | store_nonword_reg);
				imc_ack <=  1'b0;
			end
			else if (dmc_rqt_valid & !dsu_burn_enable) begin
				state <=  `pippo_UMCFSM_LOAD;
				dmc_ack <=  1'b1;
				imc_ack <=  1'b0;
			end
			else if (imc_rqt_valid & !dsu_burn_enable) begin
				state <=  `pippo_UMCFSM_FETCH;
				imc_ack <=  1'b1;
				dmc_ack <=  1'b0;
			end
			else begin
				state <=  `pippo_UMCFSM_IDLE;
				dmc_ack <=  1'b0;
				imc_ack <=  1'b0;
			end
		end
		`pippo_UMCFSM_FETCH: begin
			if (dmc_rqt_valid & dimx_we_i & !dsu_burn_enable) begin
				state <=  `pippo_UMCFSM_STORE;
				dmc_ack <=  !dmc_ack & (!store_nonword | store_nonword_reg);
				imc_ack <=  1'b0;
			end
			else if (dmc_rqt_valid & !dsu_burn_enable) begin
				state <=  `pippo_UMCFSM_LOAD;
				dmc_ack <=  1'b1;
				imc_ack <=  1'b0;
			end
			else if (imc_rqt_valid & !dsu_burn_enable) begin
				state <=  `pippo_UMCFSM_FETCH;
				imc_ack <=  1'b1;
				dmc_ack <=  1'b0;
			end
			else begin
				state <=  `pippo_UMCFSM_IDLE;
				dmc_ack <=  1'b0;
				imc_ack <=  1'b0;
			end
		end
		default: begin
			state <=  `pippo_UMCFSM_IDLE;
			dmc_ack <=  1'b0;
			imc_ack <=  1'b0;
		end
	endcase
end

// returned instruction address for fetcher
reg [31:0]  rsp_addr;
always @(posedge clk or posedge rst) begin
    if(rst) begin
        rsp_addr <= #1 32'd0;
    end
    else if(imc_hit & iimx_rqt_i) begin           
        rsp_addr <= #1 uocm_addr;
    end
    else begin
        rsp_addr <= #1 32'd0;
    end
end

//
// umc response to core.fetcher
//

// address phase: for fetch pipeling, to improve performance
assign iimx_rty_o = imc_hit ? !(imc_rqt_valid & !dmc_rqt_valid) : 1'b1;
// data phase
assign iimx_ack_o = imc_hit ? imc_ack : 1'b0;
assign iimx_err_o = imc_hit ? !imc_ack : 1'b0;
assign iimx_adr_o = imc_hit ? rsp_addr : 32'd0;
// still use 32 bit interface for fetcher
assign iimx_dat_o = imc_hit ? (iimx_adr_o[2] ? {ram_do[63:32],32'd0} : {32'd0, ram_do[63:32]} ): 64'd0;

//
// umc response to core.lsu
//
// data phase only, there is no address phase
assign dimx_ack_o = dmc_hit ? dmc_ack : 1'b0;
assign dimx_dat_o = dmc_hit ? ram_do : 64'd0;
assign dimx_err_o = dmc_hit ? !dmc_ack : 1'd0;

//
// umc response to core.fetcher
//
//assign iimx_rty_o = imc_ack ? !imc_ack : icbu_rty_i;
//assign iimx_ack_o = imc_ack ? imc_ack : icbu_ack_i;
//assign iimx_dat_o = imc_ack ? ram_do : icbu_dat_i;
//assign iimx_err_o = imc_ack ? !imc_ack : icbu_err_i;
//assign iimx_adr_o = imc_ack ? rsp_addr : icbu_adr_i;

//
// umc response to core.lsu
//
//assign dimx_ack_o = dmc_ack ? dmc_ack : dcbu_ack_i;
//assign dimx_dat_o = dmc_ack ? ram_do : dcbu_dat_i;
//assign dimx_err_o = dmc_ack ? !dmc_ack : dcbu_err_i;

//
// icbu request from umc
//
//assign icbu_adr_o = imc_hit ? 32'h0000_0000 : iimx_adr_i;
//assign icbu_cycstb_o = imc_hit ? 1'b0 : iimx_rqt_i;
//assign icbu_ci_o = imc_hit ? 1'b0 : iimx_ci_i;
//assign icbu_sel_o = imc_hit ? 4'h0 : iimx_sel_i;
//assign icbu_tag_o = imc_hit ? 4'h0 : iimx_tag_i;

//
// dcbu request from umc
//
//assign dcbu_adr_o = dmc_hit ? 32'h0000_0000 : dimx_adr_i;
//assign dcbu_cycstb_o = dmc_hit ? 1'b0 : dimx_rqt_i;
//assign dcbu_ci_o = dmc_hit ? 1'b0 : dimx_ci_i;
//assign dcbu_we_o = dmc_hit ? 1'b0 : dimx_we_i;
//assign dcbu_sel_o = dmc_hit ? 4'h0 : dimx_sel_i;
//assign dcbu_tag_o = dmc_hit ? 4'h0 : dimx_tag_i;
//assign dcbu_dat_o = dmc_hit ? 32'h0000_0000 : dimx_dat_i;

endmodule
