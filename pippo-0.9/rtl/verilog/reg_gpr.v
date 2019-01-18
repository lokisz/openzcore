/*
 * File:        reg_gprs.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker: 
 * Description:
 *      Register file: 32 GPRs for Fixed-point function
 *      Currently there are 3-Read/1-Write port
 *      Pipelining behavior:
            当id段冻结时，rf流水逻辑将保留上一拍访问的寄存器值，以供exe段使用
            当前id段的访问挂起－即新指令的寄存器访问请求
 * Task.I:
        [TBV] RAW指令流－如果存在读写冲突，通过wbmux解决
                id段送出读地址，exe段数据可用；此时上一条指令的exe段正写回
                如果指令需要stall exe段，是否能够hold住wbmux的值？
 *      [TBD]lut-based or bram-based, and timing of read access
 *          data is available at current cycle(ID) or next cycle(EXE)?
 *          where to integrate pipeling logic: 
 *              1. address/read_enable signal
 *              2. temp read register
 *      [TBV]timing of pipelining: 
 *          temp read register to keep current state
 *          correct behavior after ex_freeze recover(assertion -> disassertion)
 */
 
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module reg_gprs(
	// Clock and reset
	clk, rst,

    // Pipeline control
    id_freeze, ex_freeze, wb_freeze, flushpipe,
    
	// W/R i/f
	addrwa, datawa, wea, addrwb, datawb, web, 
	addra, addrb, addrc, dataa, datab, datac, rda, rdb, rdc

);

parameter dw = `OPERAND_WIDTH;
parameter aw = `GPR_ADDR_WIDTH;

//
// I/O
//
input				clk;
input				rst;

//
// gpr read access
//
input               id_freeze; 
input               ex_freeze; 
// address & data
input	[aw-1:0]	addra;
input	[aw-1:0]	addrb;
input	[aw-1:0]	addrc;
input				rda;
input				rdb;
input				rdc;
output	[dw-1:0]	dataa;
output	[dw-1:0]	datab;
output	[dw-1:0]	datac;

//
// gpr write access
//
input				wb_freeze;
input				flushpipe;
// address & data
input	[aw-1:0]	addrwa;
input	[dw-1:0]	datawa;
input				wea;

input	[aw-1:0]	addrwb;
input	[dw-1:0]	datawb;
input				web;

//
// Internal wires and regs
//
reg	    [dw:0]			dataa_reg;
reg	    [dw:0]          datab_reg;
reg	    [dw:0]          datac_reg;
wire	[dw-1:0]		from_rfa;
wire	[dw-1:0]		from_rfb;
wire	[dw-1:0]		from_rfc;
wire	[aw-1:0]		rf_addra;
wire	[aw-1:0]		rf_addrb;
wire	[aw-1:0]		rf_addrc;
wire				    rf_ena;
wire				    rf_enb;
wire				    rf_enc;

wire	[aw-1:0]	addrwa;
wire	[dw-1:0]	datawa;
wire				wea;

wire	[aw-1:0]	addrwb;
wire	[dw-1:0]	datawb;
wire				web;

wire	[aw-1:0]		rf_addrw;
wire	[dw-1:0]		rf_dataw;
wire				    rf_we;

//
// read access
//

// address
assign rf_addra = addra;
assign rf_addrb = addrb;
assign rf_addrc = addrc;

// CS RF asserted when instruction reads operand A and ID stage is not stalled
assign rf_ena = rda & ~id_freeze;
assign rf_enb = rdb & ~id_freeze;
assign rf_enc = rdc & ~id_freeze;

//
// write access
//

// write enable
//      1, when wb_freeze asserted, pipeline can't write register file
//      2, when wb_freeze disasserted, pipeline must check if flushpipe happened for except processing
// [TBD]to add extend_wb logic
assign rf_we = ((wea & ~wb_freeze) | (web & ~wb_freeze)) & (~flushpipe);

assign rf_addrw = wea ? addrwa : web ? addrwb : 5'd0;
assign rf_dataw = wea ? datawa : web ? datawb : 32'd0;

//
// Stores operand from RF_A into temp reg when pipeline is frozen
//
always @(posedge clk or posedge rst) begin
	if (rst) begin
		dataa_reg <= #1 33'b0;
	end
	else if (id_freeze & !dataa_reg[32]) begin
		dataa_reg <= #1 {1'b1, from_rfa};
	end
	else if (!id_freeze)
		dataa_reg <= #1 33'b0;
end

//
// Stores operand from RF_B into temp reg when pipeline is frozen
//
always @(posedge clk or posedge rst) begin
	if (rst) begin
		datab_reg <= #1 33'b0;
	end
	else if (ex_freeze & !datab_reg[32]) begin
		datab_reg <= #1 {1'b1, from_rfb};
	end
	else if (!ex_freeze)
		datab_reg <= #1 33'b0;
end

//
// Stores operand from RF_C into temp reg when pipeline is frozen
//
always @(posedge clk or posedge rst) begin
	if (rst) begin
		datac_reg <= #1 33'b0;
	end
	else if (ex_freeze & !datac_reg[32]) begin
		datac_reg <= #1 {1'b1, from_rfc};
	end
	else if (!ex_freeze)
		datac_reg <= #1 33'b0;
end
//
// output to EXE stage:
//  from RF or temp registers
//
assign dataa = (dataa_reg[32]) ? dataa_reg[31:0] : from_rfa;
assign datab = (datab_reg[32]) ? datab_reg[31:0] : from_rfb;
assign datac = (datac_reg[32]) ? datac_reg[31:0] : from_rfc;

//
// register file implementation
//

//
// Instantiation of generic register file
//      flip-flop based, at default case, synthesis tool infers dual-port RAM
//
reg_3r1w_generic reg_rf(

	.clk(clk),
	.rst(rst),
			
	.rda_en(rf_ena),
	.rda_addr(rf_addra),
	.rda_do(from_rfa),

	.rdb_en(rf_enb),
	.rdb_addr(addrb),
	.rdb_do(from_rfb),

	.rdc_en(rf_enc),
	.rdc_addr(addrc),
	.rdc_do(from_rfc),

	.wr_en(rf_we),
	.wr_addr(rf_addrw),
	.wr_data(rf_dataw)
);


endmodule
