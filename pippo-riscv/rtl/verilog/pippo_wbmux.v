/*
 * File:        pippo_wbmux.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description:
 *      mux input:
 *          alu - normal execution result data and bypass data 
 *          lsu - memory access data 
 *          spr - special register access data
 *          bpu - updatea link register when execution jump with link instruction
 *      mux output: 
 *          gpr - for write back
 *          operandmuxes - forward registered data(wb_fwd)
 */

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_wbmux(
	// Clock and reset
	clk, rst,

	// Internal i/f
	wb_freeze, rfwb_op,
	muxin_a, muxin_b, muxin_c, muxin_d, 
	muxout,  muxreg
);

parameter width = `OPERAND_WIDTH;

//
// I/O
//

input				clk;
input				rst;

input				    wb_freeze;
input	[width-1:0]		muxin_a;        // from alu
input	[width-1:0]		muxin_b;        // from lsu
input	[width-1:0]		muxin_c;        // from spr module
input	[width-1:0]		muxin_d;        // from lsu.ea

input	[`RFWBOP_WIDTH-1:0]	rfwb_op;    // send by id module

output	[width-1:0]		muxout;         // to gpr.wr
output	[width-1:0]		muxreg;         // to operandmux(wb_fwd)

//
// Internal wires and regs
//
reg	[width-1:0]		muxout;
reg	[width-1:0]		muxreg;

//
// Write-back multiplexer
//
always @(muxin_a or muxin_b or muxin_c or muxin_d or rfwb_op) begin
`ifdef pippo_ADDITIONAL_SYNOPSYS_DIRECTIVES
	case(rfwb_op[`RFWBOP_WIDTH-1:0]) // synopsys parallel_case infer_mux
`else
	case(rfwb_op[`RFWBOP_WIDTH-1:0]) // synopsys parallel_case
`endif
		4'b00: begin
		    muxout = muxin_a;
		end
		
		2'b01: begin
			muxout = muxin_b;
		end
		
		2'b10: begin
			muxout = muxin_c;
		end

		2'b11: begin
			muxout = muxin_d;
		end
	endcase
end

//
// Registered output for forwarding
//
always @(posedge clk or posedge rst) begin
	if (rst) begin
		muxreg <= #1 32'd0;
	end
	else if (!wb_freeze) begin
		muxreg <= #1 muxout;
	end
end

endmodule


