/*
 * File:        pippo_div_uu.v
 * Project:     pippo
 * Designer:    
 * Mainteiner:  
 * Checker:
 * Description: implmentation of 64/32 divider using non-restoring algorithm
        Detailed algorithm see [Computer Arithmetic - Algorithms and Hardware designs] chapter 13
 * Task.II
 *      optimize the implementation: reusing the pipelined register and logic to reduce size
 */

module pippo_div_uu(
    
    clk, ena, 
    
    z, d, q, s, 
    
    div0, ovf
);

//
// parameters
//
parameter z_width = 64;
parameter d_width = z_width /2;

//
// inputs & outputs
//
input clk;              
input ena;              

input  [z_width -1:0] z; // divident
input  [d_width -1:0] d; // divisor
output [d_width -1:0] q; // quotient
output [d_width -1:0] s; // remainder
output div0;
output ovf;
	
//
//
//
reg [d_width-1:0] q;
reg [d_width-1:0] s;
reg div0;
reg ovf;

//	
// functions
//

// temporal remainder
function [z_width:0] gen_s;
	input [z_width:0] si;
	input [z_width:0] di;
begin
  if(si[z_width])
    gen_s = {si[z_width-1:0], 1'b0} + di;
  else
    gen_s = {si[z_width-1:0], 1'b0} - di;
end
endfunction

// generate 1-bit quotient per stage
function [d_width-1:0] gen_q;
	input [d_width-1:0] qi;
	input [z_width:0] si;
begin
  gen_q = {qi[d_width-2:0], ~si[z_width]};
end
endfunction

// last quotient
function [d_width-1:0] assign_s;
	input [z_width:0] si;
	input [z_width:0] di;
	reg [z_width:0] tmp;
begin
  if(si[z_width])
    tmp = si + di;
  else
    tmp = si;

  assign_s = tmp[z_width-1:z_width-d_width];
end
endfunction

//
// divide
//
reg [d_width-1:0] q_pipe  [d_width-1:0];
reg [z_width:0] s_pipe  [d_width:0];
reg [z_width:0] d_pipe  [d_width:0];

reg [d_width:0] div0_pipe, ovf_pipe;

integer n0, n1, n2, n3;

// generate divisor (d) pipe
always @(d)
  d_pipe[0] <= {1'b0, d, {(z_width-d_width){1'b0}} };

always @(posedge clk)
  if(ena)
    for(n0=1; n0 <= d_width; n0=n0+1)
       d_pipe[n0] <= d_pipe[n0-1];

// generate internal remainder pipe
always @(z)
  s_pipe[0] <= z;

always @(posedge clk)
  if(ena)
    for(n1=1; n1 <= d_width; n1=n1+1)
       s_pipe[n1] <= gen_s(s_pipe[n1-1], d_pipe[n1-1]);

// generate quotient pipe
always @(posedge clk)
  q_pipe[0] <= 0;

always @(posedge clk)
  if(ena)
    for(n2=1; n2 < d_width; n2=n2+1)
       q_pipe[n2] <= gen_q(q_pipe[n2-1], s_pipe[n2]);

//
// divider results
//
always @(posedge clk)
  if(ena)
    q <= gen_q(q_pipe[d_width-1], s_pipe[d_width]);

always @(posedge clk)
  if(ena)
    s <= assign_s(s_pipe[d_width], d_pipe[d_width]);

//
// flags (divide_by_zero, overflow)
//
always @(z or d)
begin
//  ovf_pipe[0]  <= !(z[z_width-1:d_width] < d);
    ovf_pipe[0] <= ~|d;     // [TBD]there is no need to check overflow, coz the high part bits is just signed-extend
    div0_pipe[0] <= ~|d;
end

always @(posedge clk)
  if(ena)
    for(n3=1; n3 <= d_width; n3=n3+1)
    begin
        ovf_pipe[n3] <= ovf_pipe[n3-1];
        div0_pipe[n3] <= div0_pipe[n3-1];
    end

always @(posedge clk)
  if(ena)
    ovf <= ovf_pipe[d_width];

always @(posedge clk)
  if(ena)
    div0 <= div0_pipe[d_width];
	    
endmodule


