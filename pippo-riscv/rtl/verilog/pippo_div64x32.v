/*
 * File:        pippo_div64x32.v
 * Project:     pippo
 * Designer:    
 * Mainteiner:  
 * Checker:
 * Description: implmentation of 32/32 divider 
        Currently, the implementation signed-extend the 32-bit divident to 64 bit, then divide
        using non-restoring algorithm: Detailed algorithm see [Computer Arithmetic - Algorithms and Hardware designs] chapter 13
 * Task.I
        [TBD]the overflow definition of powerpc divide instructions?
            currently, the overflow defined by textbooks can't happen - just signed-extend of the 32-bit divident
 */

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

module pippo_div64x32 (
    
    clk, ena, 
    
    z, d, q, s, 
    
    ovf, div0
);

//
// parameters
//
parameter z_width = 64;
parameter d_width = z_width /2;
  
//
// i/o
//
input clk;              
input ena;              

input  [z_width-1:0] z; // divident
input  [d_width-1:0] d; // divisor
output [d_width  :0] q; // quotient
output [d_width  :0] s; // remainder

output ovf;
output div0;

//
// regs & wires
//
reg [d_width:0] q, s;
reg ovf, div0;

reg [z_width -1:0] iz;
reg [d_width -1:0] id;
reg [d_width +1:0] szpipe, sdpipe;

wire [d_width -1:0] iq, is;
wire                idiv0, iovf;

//
// rtl
//

// check sign bit, take abs value
always @(posedge clk)
if (ena)
  if (z[z_width-1])
     iz <= ~z +1'h1;
  else
     iz <= z;

always @(posedge clk)
if (ena)
  if (d[d_width-1])
     id <= ~d +1'h1;
  else
     id <= d;

pippo_div_uu #(z_width, d_width) divider (
    .clk(clk),
    .ena(ena),
    .z(iz),
    .d(id),
    .q(iq),
    .s(is),
    .div0(idiv0),
    .ovf(iovf)
);

//
// adjust sign bit of results
// [TBD] to eliminate the redundant pipelined register for sign bit

// generate szpipe (z sign bit pipe)
integer n;
always @(posedge clk)
if(ena)
begin
    szpipe[0] <= z[z_width-1];
    for(n=1; n <= d_width+1; n=n+1)
       szpipe[n] <= szpipe[n-1];
end

// generate sdpipe (d sign bit pipe)
integer m;
always @(posedge clk)
if(ena)
begin
    sdpipe[0] <= d[d_width-1];
    for(m=1; m <= d_width+1; m=m+1)
       sdpipe[m] <= sdpipe[m-1];
end

always @(posedge clk) begin
if(ena)
    begin
        q <= (szpipe[d_width+1]^sdpipe[d_width+1]) ?
             ((~iq) + 1'h1) : ({1'b0, iq});
        s <= (szpipe[d_width+1]) ?
             ((~is) + 1'h1) : ({1'b0, is});
    end
end

// delay flags same as results
always @(posedge clk) begin
if(ena)
    begin
        ovf  <= iovf;
        div0 <= idiv0;
    end
end

endmodule
