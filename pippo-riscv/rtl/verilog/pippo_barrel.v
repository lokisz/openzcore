/*
 * File:        pippo_barrel.v
 * Project:     pippo
 * Designer:    fang@ali
 * Mainteiner:  fang@ali
 * Checker:
 * Assigner:    
 * Description: barrel shifter
 * Task
 *      [TBD]合并32位和64位移位逻辑；
 *      [TBO]多周期参数化     
 */
 
module pippo_barrel (
   
	shift_in,
	shift_cnt,

	shift_left,
	shift_arith,
	shift_mode_32b, 

    shrot_out
	);
parameter width = `OPERAND_WIDTH;

input   [width-1:0]     shift_in;   // data to be shifted
input   [5:0]           shift_cnt;	// fully encoded shift amount
input   shift_left;		    // =1 for left shift, =0 for right
input   shift_arith;        // =1 for arith shift, =0 for logical
input   shift_mode_32b;     // =0 for 64bit shift, =1 for 32bit shift

output [width-1:0] shrot_out;

//
// 32b mode shift
//
wire                fill_32b;
reg     [95:0]      shifted_32b;
wire    [95:0]      sht_operand_32b;
wire    [64:0]      shift_out_32b;

assign fill_32b = shift_in[31] & shift_arith & shift_mode_32b;
assign sht_operand_32b = {({32{fill}}), shift_in[31:0], 32'b0};

always @ (sht_operand or shift_left or shift_cnt) begin
	if (shift_left)
		shifted_32b = sht_operand_32b << shift_cnt[4:0];
	else
		shifted_32b = sht_operand_32b >> shift_cnt[4:0];
end

assign shift_out_32b = {32{shifted_32b[63]}, shifted_32b[63:32]};

//
// 64b mode shift
//
wire                fill_64b;
reg     [191:0]     shifted_64b;
wire    [191:0]     sht_operand_64b;
wire    [63:0]      shift_out_64b;

assign fill_64b = shift_in[63] & shift_arith & (!shift_mode_32b);
assign sht_operand_64b = {({64{fill}}), shift_in[63:0], 64'b0};

always @ (sht_operand_64b or shift_left_64b or shift_cnt) begin
	if (shift_left)
		shifted_64b = sht_operand_64b << shift_cnt[5:0];
	else
		shifted_64b = sht_operand_64b >> shift_cnt[5:0];
end

assign shift_out_64b = shifted_64b[127:64];

//
// shrot result
//
assign shrot_out = shift_mode_32b ? shift_out_32b : shift_out_64b;

endmodule

