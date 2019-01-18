/*
 * File:        lsu_mem2reg.v
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

module lsu_mem2reg(addr, lsu_op, memdata, regdata);

parameter width = `OPERAND_WIDTH;

//
// I/O
//
input	[1:0]			    addr;
input	[`LSUOP_WIDTH-1:0]	lsu_op;
input	[width-1:0]		    memdata;

output	[width-1:0]		    regdata;

//
//
//
reg	[width-1:0]		regdata;
reg	[width-1:0]		aligned;

//
// Alignment: get data from memory bus
//
always @(addr or memdata) begin
`ifdef pippo_ADDITIONAL_SYNOPSYS_DIRECTIVES
	case(addr) // synopsys parallel_case infer_mux
`else
	case(addr) // synopsys parallel_case
`endif
		2'b00:
			aligned = memdata;
		2'b01:
			aligned = {memdata[23:0], 8'b0};
		2'b10:
			aligned = {memdata[15:0], 16'b0};
		2'b11:
			aligned = {memdata[7:0], 24'b0};
	endcase
end

//
// Bytes
//
always @(lsu_op or aligned) begin
`ifdef pippo_ADDITIONAL_SYNOPSYS_DIRECTIVES
	case(lsu_op) // synopsys parallel_case infer_mux
`else
	case(lsu_op) // synopsys parallel_case
`endif
		`LSUOP_LBZ: begin
				regdata[7:0] = aligned[31:24];
				regdata[31:8] = 24'b0;
			end
		`LSUOP_LHA: begin
				regdata[15:0] = aligned[31:16];
				regdata[31:16] = {16{aligned[31]}};
			end
		`LSUOP_LHZ: begin
				regdata[15:0] = aligned[31:16];
				regdata[31:16] = 16'b0;
			end
		`LSUOP_LHZB: begin
				regdata[15:0] = {aligned[23:16], aligned[31:24]};
				regdata[31:16] = 16'b0;
			end
		`LSUOP_LWZ: begin
				regdata[31:0] = aligned[31:0];
			end			
		`LSUOP_LWZB: begin
				regdata[31:0] = {aligned[7:0], aligned[15:8], aligned[23:16], aligned[31:24]};
			end
		default:
				regdata[31:0] = aligned[31:0];
	endcase
end



endmodule
