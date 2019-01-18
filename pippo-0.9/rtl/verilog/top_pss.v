/*
 * File:        top_pss.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description:
 *      top module for pippo sub-system, includes
 *          pippo core, imc, dmc, icbu, dcbu
 *      
 */
 
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module top_pss(
    
    clk, rst,
    	
	txd, rxd,
    
    dsu_rst, 
    dsu_burn_enable,
   	dsu_sram_we,
   	
    iimx_adr_o,
	iimx_rqt_o,
	iimx_rty_i,
	iimx_ack_i,
	iimx_err_i,
	iimx_dat_i,
	iimx_adr_i,

    dimx_adr_o,
    dimx_rqt_o,
    dimx_we_o,
    dimx_sel_o,
    dimx_dat_o,
    dimx_dat_i,
    dimx_ack_i,
    dimx_err_i
); 

//
// I/O
//
input   clk;
input   rst;

input   dsu_rst;

input   dsu_burn_enable;
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
output	[31:0]		dimx_dat_o;
output	[31:0]		dimx_dat_i;
output				dimx_ack_i;
output				dimx_err_i;

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
wire	[31:0]		dimx_dat_o;
wire	[31:0]		dimx_dat_i;
wire				dimx_ack_i;
wire				dimx_err_i;

wire    [31:0]      dsu_sram_data;
wire    [`IOCM_Word_BW-1:0]       dsu_sram_addr;

//
// pippo_core
//
pippo_core pippo_core(
		
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
	.dimx_dat_o(dimx_dat_o),

	.dimx_dat_i(dimx_dat_i),
	.dimx_ack_i(dimx_ack_i),
	.dimx_err_i(dimx_err_i),  
	
    .sig_ext_ci(1'b0),
    .sig_ext_i(1'b0),

    .rqt_core_rst(rqt_core_rst), 
	.rqt_sys_rst(rqt_sys_rst),
	.rqt_chip_rst(rqt_chip_rst),
    
    .txd(txd),
    .rxd(rxd),
    
    .dsu_rst(dsu_rst),
    .dsu_burn_enable(dsu_burn_enable),

    .dsu_sram_ce(dsu_sram_ce),
    .dsu_sram_we(dsu_sram_we),
    .dsu_sram_addr(dsu_sram_addr),
    .dsu_sram_data(dsu_sram_data)

);

//
// IMX unified memory controller
//
imx_umc imx_umc(                

	.clk(clk),
	.rst(rst),

	.iimx_adr_i(iimx_adr_o), 
	.iimx_rqt_i(iimx_rqt_o), 
	
	.iimx_rty_o(iimx_rty_i), 
	.iimx_ack_o(iimx_ack_i), 
	.iimx_err_o(iimx_err_i), 
	.iimx_dat_o(iimx_dat_i), 
	.iimx_adr_o(iimx_adr_i), 
	
//	.lsuimc_adr_i(dimx_adr_o), 
//	.lsuimc_rqt_i(dimx_rqt_o), 
//	.lsuimc_we_i(dimx_we_o), 
//	.lsuimc_dat_i(dimx_dat_o), 
//	.lsuimc_sel_i(dimx_sel_o), 
	
//	.imclsu_dat_o(imclsu_dat_o), 
//	.imclsu_ack_o(imclsu_ack_o), 
//	.imclsu_err_o(imclsu_err_o),	


	.dimx_adr_i(dimx_adr_o), 
	.dimx_rqt_i(dimx_rqt_o), 
	.dimx_we_i(dimx_we_o), 
	.dimx_dat_i(dimx_dat_o), 
	.dimx_sel_i(dimx_sel_o), 
	
	.dimx_dat_o(dimx_dat_i), 
	.dimx_ack_o(dimx_ack_i), 
	.dimx_err_o(dimx_err_i),

    .dsu_burn_enable(dsu_burn_enable),

    .dsu_sram_ce(dsu_sram_ce),
    .dsu_sram_we(dsu_sram_we),
    .dsu_sram_addr(dsu_sram_addr),
    .dsu_sram_data(dsu_sram_data)

);

//
// i-side cbu
//

//
// d-side cbu
//

endmodule
