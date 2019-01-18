/*
 * File:        imx_uocm.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Description:
 *      general purpose synchronous single-port memory
 * Notes:
 *      1, currently signal "oe" is NOT driven by this module, it should be used by 
 *         memory controller to tag validation of read data.
 */
 
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module imx_uocm(
   clk,    
   ce, oe, we, 
   addr, di, doq
   );    
   
parameter aw = `UOCM_Word_BW;   // 2**9 word = 2K Bytes (one xilinx block ram)
parameter dw = 32;

input   clk;
input   ce;
input   we;
input   oe;	
input [aw-1:0]  addr;
input [dw-1:0]  di;

output [dw-1:0] doq;

//
// Internal wires and registers  
//
reg [dw-1:0]    mem [(1<<aw)-1:0] /*synthesis syn_ramstyle = "no_rw_check"*/;
reg [aw-1:0]    addr_reg;		

//
// boot flash
//
initial begin
	$readmemh("boot_binary.dat", mem);
end

//
// read access
//

// memory read address register for synchronous access
always @(posedge clk) begin
    if (ce)
        addr_reg <=  addr;
end
        
// Data output 
assign doq = mem[addr_reg];

//   
// write access
//
always @(posedge clk) begin
    if (we && ce)
        mem[addr] <=  di; 
end
   
endmodule
