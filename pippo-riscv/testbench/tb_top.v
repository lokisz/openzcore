/*
 * File:        tb_top.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Description:
 *      testbench for pippo-based system
 */

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module tb_top();

reg  		clk;
reg  		rst;
reg         dsu_rst;
reg         dsu_burn_enable;

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

wire    RS232_TX_DATA;
wire    RS232_RX_DATA;

wire    [7:0]   tb_rx_data;
wire    tb_rx_done;

wire    tb_tx_start;
wire    tb_tx_busy;
wire    [7:0]   tb_tx_data;

//
// clk & rst
//
parameter HalfClkCycle  = 7.8;  // FPGA_64MHZ
parameter RstTime = 3;

initial begin
    rst = 0; 
    dsu_rst = 0; 
    dsu_burn_enable = 0; 
    #RstTime rst = 1;
    dsu_rst = 1; 
    #RstTime rst = 0;
    dsu_rst = 0; 
end

initial
    clk = 1;  
always #HalfClkCycle clk = ~clk;

//
// virtual console
//
always @ (tb_rx_done or tb_rx_data) begin
    if (tb_rx_done)
    $write("%s",tb_rx_data);
end

//
// DUT
//
top_pss top_pss(    
    .clk(clk), 
    .rst(rst),

    .dsu_rst(dsu_rst),
    .dsu_burn_enable(dsu_burn_enable),
   	.dsu_sram_we(dsu_sram_we),

	.txd(RS232_TX_DATA),
	.rxd(RS232_RX_DATA),

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
    .dimx_err_i(dimx_err_i)
); 

//
// peripheral uartlite model
//
tb_dsu_rx tb_dsu_rx(
    .clk(clk), 
    .rst(rst), 
    .RxD(RS232_TX_DATA), 
    .RxD_data_ready(tb_rx_done), 
    .RxD_data(tb_rx_data), 
    .RxD_endofpacket(), 
    .RxD_idle()
);

tb_dsu_tx tb_dsu_tx(
    .clk(clk), 
    .rst(rst), 
    .TxD_start(tb_tx_start), 
    .TxD_data(tb_tx_data), 
    .TxD(RS232_RX_DATA), 
    .TxD_busy(tb_tx_busy)
);

endmodule


