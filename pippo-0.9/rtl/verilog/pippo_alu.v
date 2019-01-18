/*
 * File:        pippo_alu.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description:
     一，功能描述
        a）执行PWR架构中的算术和逻辑运算指令、比较指令、循环移位指令和CR逻辑运算指令
        b）ALU部件除得到基本结果更新目标寄存器外，根据指令类型，还将更新CR和XER寄存器。分为以下四种情况：
            1，[.]格式的算术运算指令，表示将运算结果和零进行比较，将结果记入CR0[LT, GT, EQ, SO]
            2，[o]格式的算术和逻辑运算指令、循环移位指令，表示将根据运算结果更新XER[SO, OV]
            3，带Carrying的指令，表示将根据运算结果更新XER[CA]            
        c）CR访问
            读操作：CR逻辑运算指令读取CR中的两个操作数位；
            写操作：
                1，[.]格式的算术运算指令更新域CR0；
                2，CR逻辑运算指令更新CR中的目标位；
                3，比较指令更新指定域
        d）XER访问                
            读操作：带Extended的指令，将XER[CA]作为源操作数；所有涉及到XER[SO]的指令都读取XER[SO]
            写操作：
                1，[o]格式的算术和逻辑运算指令、循环移位指令，表示将根据运算结果更新XER[SO, OV]
                2，带Carrying的指令，表示将根据运算结果更新XER[CA]                        
 * Task.I
        [TBD]carry和overflow逻辑
        [TBV]合并crf的四个写使能会有问题否
        使用寄存器的ena，以降低功耗
 * Task.II
 *      CR逻辑运算实现移至sprs模块？
 *      提高编码效率和性能，例如统一所有操作的ALUOP编码或设置子类别等
 */

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_alu(

    clk, rst, 
    
    alu_uops, cr_uops, bus_a, bus_b, carry, so, reg_zero, 
    
    sh_mb_me,
    trap_to, 
        
	result, 
    sig_trap, 
    	
	so_new, so_we,
    ov_new, ov_we,
    ca_new, ca_we,
    
	cr_addr, cr, cr_alu, cr_alu_we
);

parameter width = `OPERAND_WIDTH;

// 
input   clk;
input   rst;

input	[`ALUUOPS_WIDTH-1:0]	alu_uops;
input   [`CROP_WIDTH-1:0]       cr_uops; 

input	[width-1:0]		bus_a;
input	[width-1:0]		bus_b;
input                   reg_zero;
input   [14:0]          sh_mb_me; 
input   [4:0]           trap_to;

output	[width-1:0]		result;

// XER update
input				    carry;  // XER[CA] field
input                   so;     // XER[SO] field
output				    so_new;
output				    so_we;
output	                ca_new;
output				    ca_we;
output	                ov_new;
output				    ov_we;

// CR update
input   [14:0]          cr_addr;            // cr source selector, connect from ex_inst
input   [31:0]          cr;    
output  [31:0]          cr_alu;
output  [7:0]           cr_alu_we; 

output                  sig_trap;

//
wire    [31:0]  bus_a;
wire    [31:0]  bus_b;

//
// Logic
//
wire [`ALUOP_WIDTH-1:0] alu_op; 
assign alu_op = alu_uops[`ALUOP_WIDTH-1:0];

//
// barrel shifter
//
wire [4:0] shrot_cnt, shrot_mb, shrot_me;
wire    mask_op;
wire    rotate_en;
wire    shift_arith;
wire    shift_left;

// shrot operands
assign shrot_cnt = ((alu_op == `ALUOP_SRAWI) | (alu_op == `ALUOP_RLWIMI) | (alu_op == `ALUOP_RLWINM))
                    ? sh_mb_me[14:10] : bus_b[4:0]; 
assign shrot_mb = sh_mb_me[9:5]; 
assign shrot_me = sh_mb_me[4:0]; 

// control signals
assign rotate_en = (alu_op == `ALUOP_RLWIMI) | (alu_op == `ALUOP_RLWINM) | (alu_op == `ALUOP_RLWNM); 
assign mask_op = (alu_op == `ALUOP_RLWINM) | (alu_op == `ALUOP_RLWNM); 
assign shift_arith = (alu_op == `ALUOP_SRAW) | (alu_op == `ALUOP_SRAWI);
assign shift_left = rotate_en | (alu_op == `ALUOP_SLW);

//
//[TBD] logic for special register update
//
wire    [31:0]  shrot_result;

pippo_barrel pippo_barrel (
	.shift_in(bus_a),
	.shift_cnt(shrot_cnt),
	.shift_left(shift_left),
	.shift_arith(shift_arith),
	.rotate_en(rotate_en), 
	.insert_in(bus_b),
	.mask_begin(shrot_mb),
	.mask_end(shrot_me), 
	.mask_op(mask_op), 
    .shrot_out(shrot_result),
    .shrot_ca_new(shrot_ca_new)
);

//
// multiplier for 32x32
//
wire    [63:0]  mul_out, mul_out_a;
reg     [31:0]  mul_result;
wire    [31:0]  bus_a_u;
wire    [31:0]  bus_b_u;
wire    [31:0]  opa;
wire    [31:0]  opb;

assign tag_unsigned = (alu_op == `ALUOP_MULHWU); 
assign mul_sign = bus_a[31] ^ bus_b[31];
assign bus_a_u = bus_a[31] ? (~bus_a + 32'd1) : bus_a;
assign bus_b_u = bus_b[31] ? (~bus_b + 32'd1) : bus_b;
assign opa = tag_unsigned ? bus_a : {1'b0, bus_a_u[30:0]}; 
assign opb = tag_unsigned ? bus_b : {1'b0, bus_b_u[30:0]}; 

// unsigned multiplier    
pippo_mul32x32 pippo_mul32x32 (
    .clk(clk), 
    .rst(rst), 
    .opa(opa), 
    .opb(opb), 
    .result(mul_out_a)
);

// adjust the sign bit of result
//  1, if unsigned instruction, nothing to do
//  2, if positive mulitply negative, transfer the absolute result to negative value
// Notes: it doesn't matter for the last case: actually it's a 31*31 multiplication
//assign mul_out = tag_unsigned ? mul_out_a[63:0] : (mul_sign ? (~mul_out_a + 1'd1): {1'b0, mul_out_a[62:0]}); 
assign mul_out = tag_unsigned ? mul_out_a[63:0] : (mul_sign ? (~mul_out_a + 1'd1): mul_out_a[63:0]); 

always @(alu_op or mul_out or mul_sign) begin
    mul_result=32'd0;
	casex (alu_op)		// synopsys parallel_case
        `ALUOP_MULHWU: begin
                            mul_result = mul_out[63:32];
                       end
        `ALUOP_MULHW: begin
                            mul_result = mul_out[31:0];
                       end
        `ALUOP_MULLI: begin
                            mul_result = mul_out[31:0];
                      end
        `ALUOP_MULLW: begin
                            mul_result = mul_out[31:0];
                      end
        default: begin
                            mul_result = mul_out[31:0];
                 end
    endcase
end                           

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
reg [31:0]  result;
reg         ca_new;

always @(alu_op or bus_a or bus_b or reg_zero or carry or
         shrot_result or mul_result or shrot_ca_new) begin    
    ca_new = carry;     //[TBV] plan b: ca_new = 1'b0;
    result = 32'd0;	
	casex (alu_op)		// synopsys parallel_case
            
// arithmetic		
		`ALUOP_ADD : begin
            result = reg_zero ? bus_b : (bus_a + bus_b);
		end
		
		`ALUOP_ADDC : begin
            {ca_new, result} = bus_a + bus_b;       
		end

		`ALUOP_ADDE : begin
            {ca_new, result} = bus_a + bus_b + {31'd0, carry};
		end

		`ALUOP_SUBF : begin
			result = bus_b - bus_a;
		end

		`ALUOP_SUBFC : begin
			{ca_new, result} = bus_b - bus_a;
		end

		`ALUOP_SUBFE : begin
			{ca_new, result} = ~bus_a + bus_b + {31'd0, carry};
		end

// logic 
        `ALUOP_AND : begin
            result = bus_a & bus_b;
        end

        `ALUOP_ANDC : begin
            result = bus_a & (~bus_b);
        end

        `ALUOP_NAND	: begin
            result = ~(bus_a & bus_b);
        end

        `ALUOP_NOR : begin
            result = ~(bus_a | bus_b);
        end	    

		`ALUOP_OR : begin
			result = bus_a | bus_b;
		end

		`ALUOP_ORC : begin
			result = bus_a | (~bus_b);
		end

		`ALUOP_XOR : begin
			result = bus_a ^ bus_b;
		end

        `ALUOP_EQV : begin
            result = ~(bus_a ^ bus_b);
        end       

        `ALUOP_NEG : begin
            result = (~bus_a) + 32'd1;
        end

// multiplier
        `ifdef pippo_MULT_IMPLEMENTED
        `ALUOP_MULHWU, `ALUOP_MULHW, `ALUOP_MULLI, `ALUOP_MULLW: begin
            result = mul_result;
        end
        `endif

// barrel shifter
        `ALUOP_SLW, `ALUOP_SRW: begin
            result = bus_b[5] ? 32'd0 : shrot_result; 
        end            
        
        `ALUOP_SRAW: begin
            result = bus_b[5] ? {32{bus_a[31]}} : shrot_result; 
            ca_new = bus_b[5] ? bus_a[31] : shrot_ca_new; 
        end            

        `ALUOP_SRAWI: begin
            result = shrot_result; 
            ca_new = shrot_ca_new; 
        end            

        `ALUOP_RLWIMI, `ALUOP_RLWINM, `ALUOP_RLWNM : begin
            result = shrot_result; 
        end            

// misc        
        `ALUOP_EXTSB : begin
            result = {{24{bus_a[8]}}, bus_a[7:0]};
        end            
        `ALUOP_EXTSH : begin
            result = {{16{bus_a[16]}}, bus_a[15:0]};
        end            
        
        `ALUOP_CNTLZW: begin
            result = bus_a[31] ? 32'd0 : bus_a[30] ? 32'd1 : bus_a[29] ? 32'd2 : bus_a[28] ? 32'd3 : 
                     bus_a[27] ? 32'd4 : bus_a[26] ? 32'd5 : bus_a[25] ? 32'd6 : bus_a[24] ? 32'd7 : 
                     bus_a[23] ? 32'd8 : bus_a[22] ? 32'd9 : bus_a[21] ? 32'd10 : bus_a[20] ? 32'd11 : 
                     bus_a[19] ? 32'd12 : bus_a[18] ? 32'd13 : bus_a[17] ? 32'd14 : bus_a[16] ? 32'd15 : 
                     bus_a[15] ? 32'd16 : bus_a[14] ? 32'd17 : bus_a[13] ? 32'd18 : bus_a[12] ? 32'd19 : 
                     bus_a[11] ? 32'd20 : bus_a[10] ? 32'd21 : bus_a[9] ? 32'd22 : bus_a[8] ? 32'd23 : 
                     bus_a[7] ? 32'd24 : bus_a[6] ? 32'd25 : bus_a[5] ? 32'd26 : bus_a[4] ? 32'd27 : 
                     bus_a[3] ? 32'd28 : bus_a[2] ? 32'd29 : bus_a[1] ? 32'd30 : bus_a[0] ? 32'd31 : 
                     32'd32;
        end
	endcase
end

//
// compare inst.
//
wire    [width-1:0] cmp_a;
wire    [width-1:0] cmp_b;

// (cr_uops[3] & cr_uops[2]) assertion(1'b1) means cmp, (1'b0 means cmpl)
assign cmp_a = {bus_a[width-1] ^ (cr_uops[3] & cr_uops[2]) , bus_a[width-2:0]};
assign cmp_b = {bus_b[width-1] ^ (cr_uops[3] & cr_uops[2]) , bus_b[width-2:0]};

reg cmp_lt; 
reg cmp_gt; 
reg cmp_eq; 
reg cmp_so; 
reg cmp_crwe; 

// compare implementation
always @(cr_uops or cmp_a or cmp_b or so_new) begin
	case(cr_uops)	// synopsys parallel_case
		`CROP_TRAP, `CROP_CMP, `CROP_CMPL: begin
			cmp_lt = (cmp_a < cmp_b);
			cmp_gt = (cmp_a > cmp_b);
			cmp_eq = (cmp_a == cmp_b);
			cmp_so = so_new;            // [TBD]Summary overflow; a copy of XER[SO] at instruction completion.
			cmp_crwe = 1'b1; 
        end
		default: begin
			cmp_lt = 1'b0;
			cmp_gt = 1'b0;
			cmp_eq = 1'b0;
			cmp_so = so_new;            // [TBD]Summary overflow; a copy of XER[SO] at instruction completion.
			cmp_crwe = 1'b0; 
	    end
	endcase
end

//
// tw/twi instruction processing
//
assign sig_trap = (cr_uops == `CROP_TRAP) & (trap_to[4] & cmp_lt) | (trap_to[3] & cmp_gt) |
                                            (trap_to[2] & cmp_eq) | (trap_to[1] & cmp_lt) |
                                            (trap_to[0] & cmp_gt);
                                            
//
// XER update
//

// XER[SO, OV] update
// [TBD] to add logic here
assign bit_oe = alu_uops[`ALUUOPS_WIDTH-1];
assign so_we = bit_oe;
assign ov_we = bit_oe;

assign ov_new = 1'b0;           // [TBD]
assign so_new = so | ov_new;    // [TBD]

// XER[CA] update see logic above

reg ca_we; 
always @(alu_op) begin
	casex (alu_op)		// synopsys parallel_case
		`ALUOP_ADDC : begin
			ca_we = 1'b1;
		end
		default: begin
			ca_we = 1'b0;
		end
	endcase
end


//
// CR-related instructions processing
//

// CR0 update from [.] inst.
assign bit_rc = alu_uops[`ALUUOPS_WIDTH-2];

assign cr0_lt = result[31];
assign cr0_gt = (~result[31]) & |result[30:0];
assign cr0_eq = (result == 32'h0000_0000);
assign cr0_so = so | ov_new;            // [TBD]Summary overflow; a copy of XER[SO] at instruction completion.


// cr update from cr inst.
wire [4:0]  crbD;
wire [4:0]  crbA;
wire [4:0]  crbB;

assign crbD = cr_addr[14:10];
assign crbA = cr_addr[9:5];
assign crbB = cr_addr[4:0];

reg crb_opa; 
always @(crbA or cr) begin
    case (crbA)
        5'b00000: crb_opa = cr[31];
        5'b00001: crb_opa = cr[30];
        5'b00010: crb_opa = cr[29];
        5'b00011: crb_opa = cr[28];
        5'b00100: crb_opa = cr[27];
        5'b00101: crb_opa = cr[26];
        5'b00110: crb_opa = cr[25];
        5'b00111: crb_opa = cr[24];
        5'b01000: crb_opa = cr[23];
        5'b01001: crb_opa = cr[22];
        5'b01010: crb_opa = cr[21];
        5'b01011: crb_opa = cr[20];
        5'b01100: crb_opa = cr[19];
        5'b01101: crb_opa = cr[18];
        5'b01110: crb_opa = cr[17];
        5'b01111: crb_opa = cr[16];
        5'b10000: crb_opa = cr[15];
        5'b10001: crb_opa = cr[14];
        5'b10010: crb_opa = cr[13];
        5'b10011: crb_opa = cr[12];
        5'b10100: crb_opa = cr[11];
        5'b10101: crb_opa = cr[10];
        5'b10110: crb_opa = cr[9];
        5'b10111: crb_opa = cr[8];
        5'b11000: crb_opa = cr[7];
        5'b11001: crb_opa = cr[6];
        5'b11010: crb_opa = cr[5];
        5'b11011: crb_opa = cr[4];
        5'b11100: crb_opa = cr[3];
        5'b11101: crb_opa = cr[2];
        5'b11110: crb_opa = cr[1];
        5'b11111: crb_opa = cr[0];               
    endcase        
end

reg crb_opb;
always @(crbA or cr) begin
    case (crbA)
        5'b00000: crb_opb = cr[31];
        5'b00001: crb_opb = cr[30];
        5'b00010: crb_opb = cr[29];
        5'b00011: crb_opb = cr[28];
        5'b00100: crb_opb = cr[27];
        5'b00101: crb_opb = cr[26];
        5'b00110: crb_opb = cr[25];
        5'b00111: crb_opb = cr[24];
        5'b01000: crb_opb = cr[23];
        5'b01001: crb_opb = cr[22];
        5'b01010: crb_opb = cr[21];
        5'b01011: crb_opb = cr[20];
        5'b01100: crb_opb = cr[19];
        5'b01101: crb_opb = cr[18];
        5'b01110: crb_opb = cr[17];
        5'b01111: crb_opb = cr[16];
        5'b10000: crb_opb = cr[15];
        5'b10001: crb_opb = cr[14];
        5'b10010: crb_opb = cr[13];
        5'b10011: crb_opb = cr[12];
        5'b10100: crb_opb = cr[11];
        5'b10101: crb_opb = cr[10];
        5'b10110: crb_opb = cr[9];
        5'b10111: crb_opb = cr[8];
        5'b11000: crb_opb = cr[7];
        5'b11001: crb_opb = cr[6];
        5'b11010: crb_opb = cr[5];
        5'b11011: crb_opb = cr[4];
        5'b11100: crb_opb = cr[3];
        5'b11101: crb_opb = cr[2];
        5'b11110: crb_opb = cr[1];
        5'b11111: crb_opb = cr[0];               
    endcase        
end

assign crb31_opwe = (crbD == 5'b00000); 
assign crb30_opwe = (crbD == 5'b00001); 
assign crb29_opwe = (crbD == 5'b00010); 
assign crb28_opwe = (crbD == 5'b00011); 
assign crb27_opwe = (crbD == 5'b00100); 
assign crb26_opwe = (crbD == 5'b00101); 
assign crb25_opwe = (crbD == 5'b00110); 
assign crb24_opwe = (crbD == 5'b00111); 
assign crb23_opwe = (crbD == 5'b01000); 
assign crb22_opwe = (crbD == 5'b01001); 
assign crb21_opwe = (crbD == 5'b01010); 
assign crb20_opwe = (crbD == 5'b01011); 
assign crb19_opwe = (crbD == 5'b01100); 
assign crb18_opwe = (crbD == 5'b01101); 
assign crb17_opwe = (crbD == 5'b01110); 
assign crb16_opwe = (crbD == 5'b01111); 
assign crb15_opwe = (crbD == 5'b10000); 
assign crb14_opwe = (crbD == 5'b10001); 
assign crb13_opwe = (crbD == 5'b10010); 
assign crb12_opwe = (crbD == 5'b10011); 
assign crb11_opwe = (crbD == 5'b10100); 
assign crb10_opwe = (crbD == 5'b10101); 
assign crb9_opwe = (crbD == 5'b10110); 
assign crb8_opwe = (crbD == 5'b10111); 
assign crb7_opwe = (crbD == 5'b11000); 
assign crb6_opwe = (crbD == 5'b11001); 
assign crb5_opwe = (crbD == 5'b11010); 
assign crb4_opwe = (crbD == 5'b11011); 
assign crb3_opwe = (crbD == 5'b11100); 
assign crb2_opwe = (crbD == 5'b11101); 
assign crb1_opwe = (crbD == 5'b11110); 
assign crb0_opwe = (crbD == 5'b11111); 

reg crb_opd;                 
always @(cr_uops or crb_opa or crb_opb) begin
    crb_opd = 1'bx;
	casex (cr_uops)		// synopsys parallel_case
		`CROP_AND : begin
			crb_opd = crb_opa & crb_opb;
		end

		`CROP_ANDC : begin
			crb_opd = crb_opa & crb_opb;
		end

		`CROP_EQV : begin
			crb_opd = (crb_opa == crb_opb);
		end

		`CROP_NAND : begin
			crb_opd = crb_opa & crb_opb;
		end

		`CROP_NOR : begin
			crb_opd = crb_opa & crb_opb;
		end

		`CROP_OR : begin
			crb_opd = crb_opa | crb_opb;
		end

		`CROP_XOR : begin
			crb_opd = crb_opa & crb_opb;
		end

		`CROP_ORC : begin
			crb_opd = crb_opa & crb_opb;
		end
		
	endcase
end

// cr update from compare inst.
wire [2:0]  cmp_crf; 

assign cmp_crf = cr_addr[14:12];

assign cr0_cmpwe = (cmp_crf == 3'b000) & cmp_crwe; 
assign cr1_cmpwe = (cmp_crf == 3'b001) & cmp_crwe; 
assign cr2_cmpwe = (cmp_crf == 3'b010) & cmp_crwe; 
assign cr3_cmpwe = (cmp_crf == 3'b011) & cmp_crwe; 
assign cr4_cmpwe = (cmp_crf == 3'b100) & cmp_crwe; 
assign cr5_cmpwe = (cmp_crf == 3'b101) & cmp_crwe; 
assign cr6_cmpwe = (cmp_crf == 3'b110) & cmp_crwe; 
assign cr7_cmpwe = (cmp_crf == 3'b111) & cmp_crwe; 

// bit_rc only update CR0(cr[31:28])
assign crb31_new =
		crb31_opwe ? crb_opd :
		bit_rc ? cr0_lt :
		cr0_cmpwe ? cmp_lt : 
		cr[31];

assign crb30_new =
		crb30_opwe ? crb_opd :
		bit_rc ? cr0_gt :
		cr0_cmpwe ? cmp_gt : 
		cr[30];

assign crb29_new =
		crb29_opwe ? crb_opd :
		bit_rc ? cr0_eq:
		cr0_cmpwe ? cmp_eq : 
		cr[29];

assign crb28_new =
		crb28_opwe ? crb_opd :
		bit_rc ? cr0_so :
		cr0_cmpwe ? cmp_so : 
		cr[28];

assign crb27_new =
		crb27_opwe ? crb_opd :
		cr1_cmpwe ? cmp_lt: 
		cr[27];

assign crb26_new =
		crb26_opwe ? crb_opd :
		cr1_cmpwe ? cmp_gt: 
		cr[26];

assign crb25_new =
		crb25_opwe ? crb_opd :
		cr1_cmpwe ? cmp_eq : 
		cr[25];

assign crb24_new =
		crb24_opwe ? crb_opd :
		cr1_cmpwe ? cmp_so: 
		cr[24];

assign crb23_new =
		crb23_opwe ? crb_opd :
		cr2_cmpwe ? cmp_lt: 
		cr[23];

assign crb22_new =
		crb22_opwe ? crb_opd :
		cr2_cmpwe ? cmp_gt: 
		cr[22];

assign crb21_new =
		crb21_opwe ? crb_opd :
		cr2_cmpwe ? cmp_eq : 
		cr[21];

assign crb20_new =
		crb20_opwe ? crb_opd :
		cr2_cmpwe ? cmp_so: 
		cr[20];

assign crb19_new =
		crb19_opwe ? crb_opd :
		cr3_cmpwe ? cmp_lt: 
		cr[19];

assign crb18_new =
		crb18_opwe ? crb_opd :
		cr3_cmpwe ? cmp_gt: 
		cr[18];

assign crb17_new =
		crb17_opwe ? crb_opd :
		cr3_cmpwe ? cmp_eq : 
		cr[17];

assign crb16_new =
		crb16_opwe ? crb_opd :
		cr3_cmpwe ? cmp_so: 
		cr[16];

assign crb15_new =
		crb15_opwe ? crb_opd :
		cr4_cmpwe ? cmp_lt: 
		cr[15];

assign crb14_new =
		crb14_opwe ? crb_opd :
		cr4_cmpwe ? cmp_gt: 
		cr[14];

assign crb13_new =
		crb13_opwe ? crb_opd :
		cr4_cmpwe ? cmp_eq: 
		cr[13];

assign crb12_new =
		crb12_opwe ? crb_opd :
		cr4_cmpwe ? cmp_so: 
		cr[12];

assign crb11_new =
		crb11_opwe ? crb_opd :
		cr5_cmpwe ? cmp_lt: 
		cr[11];

assign crb10_new =
		crb10_opwe ? crb_opd :
		cr5_cmpwe ? cmp_gt: 
		cr[10];

assign crb9_new =
		crb9_opwe ? crb_opd :
		cr5_cmpwe ? cmp_eq: 
		cr[9];

assign crb8_new =
		crb8_opwe ? crb_opd :
		cr5_cmpwe ? cmp_so: 
		cr[8];

assign crb7_new =
		crb7_opwe ? crb_opd :
		cr6_cmpwe ? cmp_lt: 
		cr[7];

assign crb6_new =
		crb6_opwe ? crb_opd :
		cr6_cmpwe ? cmp_gt: 
		cr[6];

assign crb5_new =
		crb5_opwe ? crb_opd :
		cr6_cmpwe ? cmp_eq: 
		cr[5];

assign crb4_new =
		crb4_opwe ? crb_opd :
		cr6_cmpwe ? cmp_so: 
		cr[4];

assign crb3_new =
		crb3_opwe ? crb_opd :
		cr7_cmpwe ? cmp_lt: 
		cr[3];

assign crb2_new =
		crb2_opwe ? crb_opd :
		cr7_cmpwe ? cmp_gt: 
		cr[2];

assign crb1_new =
		crb1_opwe ? crb_opd :
		cr7_cmpwe ? cmp_eq: 
		cr[1];

assign crb0_new =
		crb0_opwe ? crb_opd :
		cr7_cmpwe ? cmp_so: 
		cr[0];

assign crb31_we = crb31_opwe | bit_rc | cr0_cmpwe; 
assign crb30_we = crb30_opwe | bit_rc | cr0_cmpwe; 
assign crb29_we = crb29_opwe | bit_rc | cr0_cmpwe; 
assign crb28_we = crb28_opwe | bit_rc | cr0_cmpwe; 
assign crb27_we = crb27_opwe | cr1_cmpwe; 
assign crb26_we = crb26_opwe | cr1_cmpwe; 
assign crb25_we = crb25_opwe | cr1_cmpwe; 
assign crb24_we = crb24_opwe | cr1_cmpwe; 
assign crb23_we = crb23_opwe | cr2_cmpwe; 
assign crb22_we = crb22_opwe | cr2_cmpwe; 
assign crb21_we = crb21_opwe | cr2_cmpwe; 
assign crb20_we = crb20_opwe | cr2_cmpwe; 
assign crb19_we = crb19_opwe | cr3_cmpwe; 
assign crb18_we = crb18_opwe | cr3_cmpwe; 
assign crb17_we = crb17_opwe | cr3_cmpwe; 
assign crb16_we = crb16_opwe | cr3_cmpwe; 
assign crb15_we = crb15_opwe | cr4_cmpwe; 
assign crb14_we = crb14_opwe | cr4_cmpwe; 
assign crb13_we = crb13_opwe | cr4_cmpwe; 
assign crb12_we = crb12_opwe | cr4_cmpwe; 
assign crb11_we = crb11_opwe | cr5_cmpwe; 
assign crb10_we = crb10_opwe | cr5_cmpwe; 
assign crb9_we = crb9_opwe | cr5_cmpwe; 
assign crb8_we = crb8_opwe | cr5_cmpwe; 
assign crb7_we = crb7_opwe | cr6_cmpwe; 
assign crb6_we = crb6_opwe | cr6_cmpwe; 
assign crb5_we = crb5_opwe | cr6_cmpwe; 
assign crb4_we = crb4_opwe | cr6_cmpwe; 
assign crb3_we = crb3_opwe | cr7_cmpwe; 
assign crb2_we = crb2_opwe | cr7_cmpwe; 
assign crb1_we = crb1_opwe | cr7_cmpwe; 
assign crb0_we = crb0_opwe | cr7_cmpwe; 

// output
wire [31:0]     cr_alu;
wire [7:0]      cr_alu_we;

assign cr_alu = {crb31_new, crb30_new, crb29_new, crb28_new,
                 crb27_new, crb26_new, crb25_new, crb24_new,
                 crb23_new, crb22_new, crb21_new, crb20_new,
                 crb19_new, crb18_new, crb17_new, crb16_new,
                 crb15_new, crb14_new, crb13_new, crb12_new,
                 crb11_new, crb10_new, crb9_new, crb8_new,
                 crb7_new, crb6_new, crb5_new, crb4_new, 
                 crb3_new, crb2_new, crb1_new, crb0_new
                };
                
assign cr_alu_we[7] = crb31_we | crb30_we | crb29_we | crb28_we;
assign cr_alu_we[6] = crb27_we | crb26_we | crb25_we | crb24_we;
assign cr_alu_we[5] = crb23_we | crb22_we | crb21_we | crb20_we;
assign cr_alu_we[4] = crb19_we | crb18_we | crb17_we | crb16_we;
assign cr_alu_we[3] = crb15_we | crb14_we | crb13_we | crb12_we;
assign cr_alu_we[2] = crb11_we | crb10_we | crb9_we | crb8_we;
assign cr_alu_we[1] = crb7_we | crb6_we | crb5_we | crb4_we;
assign cr_alu_we[0] = crb3_we | crb2_we | crb1_we | crb0_we;

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
