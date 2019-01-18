/*
 * File:        lsu_reg2mem.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Description:
 *  
 */
 
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module lsu_reg2mem(addr, lsu_op, regdata, memdata);

parameter width = `OPERAND_WIDTH;

//
// I/O
//
input	[1:0]			addr;
input	[`LSUOP_WIDTH-1:0]	lsu_op;
input	[width-1:0]		regdata;
output	[width-1:0]		memdata;

//
// big-endian memory layout
//
reg	[7:0]			memdata_hh;     // byte address 00
reg	[7:0]			memdata_hl;     // byte address 01
reg	[7:0]			memdata_lh;     // byte address 10
reg	[7:0]			memdata_ll;     // byte address 11

//
assign memdata = {memdata_hh, memdata_hl, memdata_lh, memdata_ll};

//
// Mux to memdata[31:24]
//
always @(lsu_op or addr or regdata) begin
	casex({lsu_op, addr[1:0]})	// synopsys parallel_case
		{`LSUOP_STB, 2'b00} : memdata_hh = regdata[7:0];
		{`LSUOP_STH, 2'b00} : memdata_hh = regdata[15:8];
		{`LSUOP_STHB, 2'b00} : memdata_hh = regdata[7:0];
		{`LSUOP_STW, 2'b00} : memdata_hh = regdata[31:24];
		{`LSUOP_STWB, 2'b00} : memdata_hh = regdata[7:0];
		default : memdata_hh = regdata[31:24];
	endcase
end

//
// Mux to memdata[23:16]
//      [TBD] comment out unneccessary access pattern(same with default), to evaluate syn result
always @(lsu_op or addr or regdata) begin
	casex({lsu_op, addr[1:0]})	// synopsys parallel_case
		{`LSUOP_STB, 2'b01} : memdata_hl = regdata[7:0];
		{`LSUOP_STH, 2'b00} : memdata_hl = regdata[7:0];
		{`LSUOP_STHB, 2'b00} : memdata_hl = regdata[15:8];
		{`LSUOP_STW, 2'b00} : memdata_hl = regdata[23:16];
		{`LSUOP_STWB, 2'b00} : memdata_hl = regdata[15:8];
		default : memdata_hl = regdata[7:0];
	endcase
end

//
// Mux to memdata[15:8]
//
always @(lsu_op or addr or regdata) begin
	casex({lsu_op, addr[1:0]})	// synopsys parallel_case
		{`LSUOP_STB, 2'b10} : memdata_lh = regdata[7:0];
		{`LSUOP_STH, 2'b10} : memdata_lh = regdata[15:8];
		{`LSUOP_STHB, 2'b10} : memdata_lh = regdata[7:0];
		{`LSUOP_STW, 2'b00} : memdata_lh = regdata[15:8];
		{`LSUOP_STWB, 2'b00} : memdata_lh = regdata[23:16];
		default : memdata_lh = regdata[15:8];
	endcase
end

//
// Mux to memdata[7:0]
//
always @(lsu_op or addr or regdata) begin
	casex({lsu_op, addr[1:0]})	// synopsys parallel_case
		{`LSUOP_STB, 2'b11} : memdata_ll = regdata[7:0];
		{`LSUOP_STH, 2'b10} : memdata_ll = regdata[7:0];
		{`LSUOP_STHB, 2'b10} : memdata_ll = regdata[15:8];
		{`LSUOP_STW, 2'b00} : memdata_ll = regdata[7:0];
		{`LSUOP_STWB, 2'b00} : memdata_ll = regdata[31:25];
		default : memdata_ll = regdata[7:0];
	endcase
end


endmodule


