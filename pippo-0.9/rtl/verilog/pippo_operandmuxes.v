/*
 * File:        pippo_operandmuxes.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description:
 *      Mux.A: 
 *          input: rf_a, wb_fwd
 *          output: bus a (alu, mac, lsu)
 *      Mux.B: 
 *          input: rf_b, imm, wb_fwd
 *          output: bus b (alu, mac, lsu, sprs(dat_i), except(datain))
 *
 */

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_operandmuxes(

	rf_dataa, rf_datab, wb_fwd,	imm,
	 
	sel_a, sel_b, 
	
	bus_a, bus_b
);

parameter width = `OPERAND_WIDTH;

//
// I/O
//
input	[width-1:0]		rf_dataa;
input	[width-1:0]		rf_datab;
input	[width-1:0]		wb_fwd;
input	[width-1:0]		imm;

input	[`OPSEL_WIDTH-1:0]	sel_a;
input	[`OPSEL_WIDTH-1:0]	sel_b;

output	[width-1:0]		bus_a;
output	[width-1:0]		bus_b;

//
// Internal wires and regs
//
reg	[width-1:0]		bus_a;
reg	[width-1:0]		bus_b;

//
// Multiplexer for operand bus A
//      source: rf_dataa, wb_fwd
//
always @(wb_fwd or rf_dataa or sel_a) begin
`ifdef pippo_ADDITIONAL_SYNOPSYS_DIRECTIVES
	casex (sel_a)	// synopsys parallel_case infer_mux
`else
	casex (sel_a)	// synopsys parallel_case
`endif
		`OPSEL_WBFWD:
			bus_a = wb_fwd;
		`OPSEL_RF:
			bus_a = rf_dataa;			
		default:
			bus_a = rf_dataa;
            `ifdef pippo_VERBOSE
            // synopsys translate_off
            		$display("%t: WARNING: OperandMux enter default case %h", $time);
            // synopsys translate_on
            `endif            
	endcase
end

//
// Multiplexer for operand bus B
//      source: imm, rf_datab, wb_fwd
//
always @(imm or wb_fwd or rf_datab or sel_b) begin
`ifdef pippo_ADDITIONAL_SYNOPSYS_DIRECTIVES
	casex (sel_b)	// synopsys parallel_case infer_mux
`else
	casex (sel_b)	// synopsys parallel_case
`endif
		`OPSEL_IMM:
			bus_b = imm;
		`OPSEL_WBFWD:
			bus_b = wb_fwd;
		`OPSEL_RF:
			bus_b = rf_datab;			
		default:
			bus_b = rf_datab;
            `ifdef pippo_VERBOSE
            // synopsys translate_off
            		$display("%t: WARNING: OperandMux enter default case %h", $time);
            // synopsys translate_on
            `endif            
	endcase
end

endmodule


