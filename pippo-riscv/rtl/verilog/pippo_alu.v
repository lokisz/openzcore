/*
 * File:        pippo_alu.v
 * Project:     pippo
 * Designer:    fang@ali
 * Mainteiner:  fang@ali
 * Checker:
 * Assigner:    
 * Description: 
     一，功能描述
        a）执行PWR架构中的算术和逻辑运算指令、比较指令、循环移位指令和CR逻辑运算指令
        b）ALU部件除得到基本结果更新目标寄存器外，根据指令类型，还将更新CR和XER寄存器。分为以下四种情况：
            1，[.]格式的算术运算指令，表示将运算结果和零进行比较，将结果记入CR0[LT, GT, EQ, SO]
            2，[o]格式的算术和逻辑运算指令、循环移位指令，表示将根据运算结果更新XER[SO, OV]
            3，带Carrying的指令，表示将根据运算结果更新XER[CA]            
 * Task:
 *      [TBO]使用寄存器的ena，以降低功耗
 */

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_alu(

    clk, rst, 
    
    alu_uops, bus_a, bus_b, 
    
    sh_mb_me,
            
	result
);

parameter width = `OPERAND_WIDTH;

// 
input   clk;
input   rst;

input	[`ALUUOPS_WIDTH-1:0]	alu_uops;

input	[width-1:0]		bus_a;
input	[width-1:0]		bus_b;
input                   reg_zero;
input   [14:0]          sh_mb_me; 

output	[width-1:0]		result;

// global wires in module
wire    [width-1:0]  bus_a;
wire    [width-1:0]  bus_b;

//
// Logic
//
wire [`ALUOP_WIDTH-1:0] alu_op; 
assign alu_op = alu_uops[`ALUOP_WIDTH-1:0];

//
// barrel shifter
//
wire [5:0] shrot_cnt;
wire    shift_arith;
wire    shift_left;
wire    shift_mode_32b
// shrot operands
assign shrot_cnt = alu_op[`ALUOP_SHTEN_BIT] ?  bus_b[5:0] : 6'b0; 

// control signals
assign shift_mode_32b = alu_op[`ALUOP_M32B_BIT];
assign shift_left = alu_op[`ALUOP_LFT_BIT];
assign shift_arith = alu_op[`ALUOP_AGM_BIT];

// shifted result
wire    [width-1:0]  shrot_result;

pippo_barrel pippo_barrel (
	.shift_in(bus_a),
	.shift_cnt(shrot_cnt),
	.shift_left(shift_left),
	.shift_arith(shift_arith),
	.shift_mode_32b(shift_mode_32b), 
    .shrot_out(shrot_result)
);

//
// multiplier for 32x32
//
//wire    [63:0]  mul_out, mul_out_a;
//reg     [31:0]  mul_result;
//wire    [31:0]  bus_a_u;
//wire    [31:0]  bus_b_u;
//wire    [31:0]  opa;
//wire    [31:0]  opb;

//assign tag_unsigned = (alu_op == `ALUOP_MULHWU); 
//assign mul_sign = bus_a[31] ^ bus_b[31];
//assign bus_a_u = bus_a[31] ? (~bus_a + 32'd1) : bus_a;
//assign bus_b_u = bus_b[31] ? (~bus_b + 32'd1) : bus_b;
//assign opa = tag_unsigned ? bus_a : {1'b0, bus_a_u[30:0]}; 
//assign opb = tag_unsigned ? bus_b : {1'b0, bus_b_u[30:0]}; 

// unsigned multiplier    
//pippo_mul32x32 pippo_mul32x32 (
//    .clk(clk), 
//    .rst(rst), 
//    .opa(opa), 
//    .opb(opb), 
//    .result(mul_out_a)
//);

// adjust the sign bit of result
//  1, if unsigned instruction, nothing to do
//  2, if positive mulitply negative, transfer the absolute result to negative value
// Notes: it doesn't matter for the last case: actually it's a 31*31 multiplication
//assign mul_out = tag_unsigned ? mul_out_a[63:0] : (mul_sign ? (~mul_out_a + 1'd1): {1'b0, mul_out_a[62:0]}); 
//assign mul_out = tag_unsigned ? mul_out_a[63:0] : (mul_sign ? (~mul_out_a + 1'd1): mul_out_a[63:0]); 

//always @(alu_op or mul_out or mul_sign) begin
//    mul_result=32'd0;
//	casex (alu_op)		// synopsys parallel_case
//        `ALUOP_MULHWU: begin
//                            mul_result = mul_out[63:32];
//                       end
//        `ALUOP_MULHW: begin
//                            mul_result = mul_out[31:0];
//                       end
//        `ALUOP_MULLI: begin
//                            mul_result = mul_out[31:0];
//                      end
//        `ALUOP_MULLW: begin
//                            mul_result = mul_out[31:0];
//                      end
//        default: begin
//                            mul_result = mul_out[31:0];
//                 end
//   endcase
//end                           

//
// hardware divider
//
//`ifdef pippo_DIV_IMPLEMENTED
//module pippo_div64x32 (   
//    clk(), 
//    ena(),     
//    z(), 
//    d(), 
//    q(), 
//    s(),     
//    ovf(), 
//    div0()
//);
//`endif

//
// ALU
//
reg     [width-1:0]     result;
wire    [width-1:0]     cmp_a, cmp_b;

assign cmp_a = {bus_a[width-1] ^ alu_uops[`ALUOP_SCMP_BIT], bus_a[width-2:0]};
assign cmp_b = {bus_b[width-1] ^ alu_uops[`ALUOP_SCMP_BIT], bus_b[width-2:0]};

always @(alu_op or bus_a or bus_b or cmp_a or cmp_b or shrot_result ) begin    
    result = 64'd0;	
	casex (alu_op)		// synopsys parallel_case            
// arithmetic		
		`ALUOP_ADD : begin
            result = bus_a + bus_b;
		end
		
		`ALUOP_SUB : begin
			result = bus_b - bus_a;
		end

// logic 
        `ALUOP_AND : begin
            result = bus_a & bus_b;
        end

		`ALUOP_OR : begin
			result = bus_a | bus_b;
		end

		`ALUOP_XOR : begin
			result = bus_a ^ bus_b;
		end

		`ALUOP_SLT, `ALUOP_SLTU: begin
			result = (cmp_a < cmp_b); 
		end

// barrel shifter
        `ALUOP_SLL,  `ALUOP_SRL,  `ALUOP_SRA, 
        `ALUOP_SLLW, `ALUOP_SRLW, `ALUOP_SRAW: begin
            result = shrot_result; 
        end            
        
// multiplier
//      `ifdef pippo_MULT_IMPLEMENTED
//      `ALUOP_MULHWU, `ALUOP_MULHW, `ALUOP_MULLI, `ALUOP_MULLW: begin
//          result = mul_result;
//      end
//      `endif

	endcase
end

//
// Simulation check for bad ALU behavior
//
`ifdef pippo_WARNINGS
// synopsys translate_off
always @(result) begin
	if (result === 32'bx)
		$display("%t: WARNING: 32'bx detected on ALU result bus. Please check !", $time);
end
// synopsys translate_on
`endif

endmodule
