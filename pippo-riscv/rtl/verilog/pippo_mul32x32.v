/*
 * File:        pippo_mul32x32.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Description: implmentation of 32*32 multiplier
 *      currently, it use a generic rtl for fpga implementation
 *      and, there is two stages to enable retiming optimization of synthesis tool
 */

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

module pippo_mul32x32 (opa, opb, clk, rst, result);

input   [31:0]  opa;
input   [31:0]  opb;
input           clk;
input           rst;
output  [63:0]  result;

//
//
//
reg     [63:0]  s1;
reg     [63:0]  s2;
wire    [63:0]  result;

//
// 1st stage
//
always @(posedge clk or posedge rst) begin
    if (rst)
        s1 <= 64'd0;
    else
        s1 <= opa * opb;
end

//
// 2nd stage
//
always @(posedge clk or posedge rst) begin
    if (rst)
        s2 <= 64'd0;
    else
        s2 <= s1;
end

assign result = s2;

endmodule

