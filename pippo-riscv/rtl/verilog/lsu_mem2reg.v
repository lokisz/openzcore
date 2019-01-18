/*
 * File:        lsu_mem2reg.v
 * Project:     pippo
 * Designer:    fang@ali
 * Mainteiner:  fang@ali
 * Checker:
 * Description:
 *      MUX for load data
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
//  align data to lsb according to access address
always @(addr or memdata) begin
`ifdef pippo_ADDITIONAL_SYNOPSYS_DIRECTIVES
	case(addr) // synopsys parallel_case infer_mux
`else
	case(addr) // synopsys parallel_case
`endif
		3'b000:
			aligned = memdata;
		3'b001:
			aligned = {8'b0, memdata[55:0]};
		3'b010:
			aligned = {16'b0, memdata[47:0]};
		3'b011:
			aligned = {24'b0, memdata[39:0]};
		3'b100:
			aligned = {32'b0, memdata[31:0] };
		3'b101:
			aligned = {40'b0, memdata[23:0]};
		3'b110:
			aligned = {48'b0, memdata[15:0]};
		3'b111:
			aligned = {56'b0, memdata[7:0]};
        default:
            aligned = memdata;	    
	endcase
end

//
// Bytes
//  mux data and extend according to instruction type
always @(lsu_op or aligned) begin
`ifdef pippo_ADDITIONAL_SYNOPSYS_DIRECTIVES
	case(lsu_op) // synopsys parallel_case infer_mux
`else
	case(lsu_op) // synopsys parallel_case
`endif
		`LSUOP_LB: begin
				regdata[63:16] = {56{aligned[7]}};
				regdata[7:0] = aligned[7:0];
			end

		`LSUOP_LBU: begin
				regdata[63:8] = 56'b0;
				regdata[7:0] = aligned[7:0];
			end

		`LSUOP_LH: begin
				regdata[63:16] = {48{aligned[31]}};
				regdata[31:16] = aligned[15:0];
			end

		`LSUOP_LHU: begin
				regdata[63:16] = 48'b0;
				regdata[15:0] = aligned[15:0];;
			end

		`LSUOP_LW: begin
				regdata[63:32] = {32{aligned[31]}};
				regdata[31:0]  = aligned[31:0]};
			end			

		`LSUOP_LWU: begin
				regdata[63:0] = {32{1'b0}};
				regdata[31:0] = aligned[31:0]};
			end

		`LSUOP_LD: begin
				regdata[63:0] = aligned[63:0];
			end
		default:
				regdata[63:0] = aligned[63:0];
	endcase
end



endmodule
