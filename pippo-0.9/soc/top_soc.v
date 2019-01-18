/*
 * File:        top_soc.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi 
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description:
 *      top module for soc, includes
 *          pippo sub-system, interconnection fabric, peripheral IP
 *      
 */
 
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module top_soc(
    
    clk, rst,
    	
	txd, rxd,
    
    dsu_rst, 
    dsu_burn_enable,
   	dsu_burn_status,
   	dsu_sram_we,
   	
    iimx_adr_o,
	iimx_cycstb_o,
	iimx_sel_o,
	iimx_tag_o,

	iimx_rty_i,
	iimx_ack_i,
	iimx_err_i,
	iimx_dat_i,
	iimx_adr_i,
	iimx_tag_i,

    dimx_adr_o,
    dimx_cycstb_o,
    dimx_we_o,
    dimx_sel_o,
    dimx_tag_o,
    dimx_dat_o,
    dimx_dat_i,
    dimx_ack_i,
    dimx_rty_i,
    dimx_err_i,
    dimx_tag_i
); 

//
// I/O
//
input   clk;
input   rst;

input   dsu_rst;

input   dsu_burn_enable;
output  dsu_burn_status;
output  dsu_sram_we;

output  txd;
input   rxd;

output	[31:0]		iimx_adr_o;
output				iimx_rqt_o;
output	[31:0]		iimx_dat_i;
output				iimx_ack_i;
output				iimx_rty_i;
output				iimx_err_i;
output	[31:0]		iimx_adr_i;

output	[31:0]		dimx_adr_o;
output				dimx_rqt_o;
output				dimx_we_o;
output	[3:0]		dimx_sel_o;
output	[3:0]		dimx_tag_o;
output	[31:0]		dimx_dat_o;
output	[31:0]		dimx_dat_i;
output				dimx_ack_i;
output				dimx_rty_i;
output				dimx_err_i;
output	[3:0]		dimx_tag_i;

//
// interconnections
//

wire	[31:0]		iimx_adr_o;
wire				iimx_rqt_o;
wire	[31:0]		iimx_dat_i;
wire				iimx_ack_i;
wire				iimx_rty_i;
wire				iimx_err_i;
wire	[31:0]		iimx_adr_i;

wire	[31:0]		dimx_adr_o;
wire				dimx_rqt_o;
wire				dimx_we_o;
wire	[3:0]		dimx_sel_o;
wire	[3:0]		dimx_tag_o;
wire	[31:0]		dimx_dat_o;
wire	[31:0]		dimx_dat_i;
wire				dimx_ack_i;
wire				dimx_rty_i;
wire				dimx_err_i;
wire	[3:0]		dimx_tag_i;

wire    [31:0]      dsu_sram_data;
wire    [`IOCM_Word_BW-1:0]       dsu_sram_addr;

//
// pippo_core
//
top_pss top_pss(
		
	.clk(clk),
	.rst(rst),

    .iimx_adr_o(iimx_adr_o),
	.iimx_rqt_o(iimx_rqt_o),

	.iimx_rty_i(iimx_rty_i),
	.iimx_ack_i(iimx_ack_i),
	.iimx_err_i(iimx_err_i),
	.iimx_dat_i(iimx_dat_i),
	.iimx_adr_i(iimx_adr_i),
   
	.dimx_adr_o(dimx_adr_o),
	.dimx_rqt_o(dimx_rqt_o),
	.dimx_we_o(dimx_we_o),
	.dimx_sel_o(dimx_sel_o),
	.dimx_tag_o(dimx_tag_o),
	.dimx_dat_o(dimx_dat_o),

	.dimx_dat_i(dimx_dat_i),
	.dimx_ack_i(dimx_ack_i),
	.dimx_rty_i(dimx_rty_i),
	.dimx_err_i(dimx_err_i),  
	.dimx_tag_i(dimx_tag_i),
	
    .sig_ext_ci(1'b0),
    .sig_ext_i(1'b0),

    .rqt_core_rst(rqt_core_rst), 
	.rqt_sys_rst(rqt_sys_rst),
	.rqt_chip_rst(rqt_chip_rst),
    
    .txd(txd),
    .rxd(rxd),
    
    .dsu_rst(dsu_rst),
    .dsu_burn_enable(dsu_burn_enable),
   	.dsu_burn_status(dsu_burn_status),

    .dsu_sram_ce(dsu_sram_ce),
    .dsu_sram_we(dsu_sram_we),
    .dsu_sram_addr(dsu_sram_addr),
    .dsu_sram_data(dsu_sram_data)

);

//
// interconnection fabirc
//

//
// peripheral IPs
//



endmodule
