/*
 * File:        demo_top.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi 
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description:
 *      top module for FPGA demo on XUP board
 *
 */
 
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module demo_top(
    
FPGA_SYSTEMACE_CLOCK, 

SW_0, SW_1, SW_2, SW_3,

PB_ENTER, PB_UP, PB_DOWN, PB_LEFT, PB_RIGHT,

LED_0, LED_1, LED_2, LED_3,

RS232_TX_DATA, RS232_RX_DATA
); 

//
// I/O
//

// clock to DCM
input   FPGA_SYSTEMACE_CLOCK;       // 32MHz 

// swith on board
input   SW_0;
input   SW_1;
input   SW_2;
input   SW_3;

// pushbotton on board
input   PB_ENTER;
input   PB_UP;
input   PB_DOWN;
input   PB_LEFT;
input   PB_RIGHT;

// led on board
output  LED_0;
output  LED_1;
output  LED_2;
output  LED_3;

// uart on board
input   RS232_RX_DATA;
output  RS232_TX_DATA;

//
// interconnections
//
wire                dsu_sram_we;

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

//
// clock and reset 
//
demo_clk demo_clk (
    .CLK_IN(FPGA_SYSTEMACE_CLOCK),
    .RST(1'b0),
    .CLK1X(clk32M),
    .CLK2X(clk),
    .LOCK(dcm_lock)
);

// [TBD] BUFG
assign rst = !PB_ENTER; 

//
// heartbreak logic: clock is running
//
//  Note: frequency of clk is 64MHz. 1s = 15.5ns * 64 * 10e6 (0x3D0_9000, 0011_1101_0000_1001_0000_0000_0000)
reg [26:0]   clk_counter;

always @(posedge clk or posedge rst) begin
    if(rst)
        clk_counter <= 27'd0;
    else
        clk_counter <= clk_counter + 27'd1;
end

//
// reserved logic: just to keep core un-optimized by synthesis tool
//
wire    iimx_rqt_status;
wire    iimx_rsp_status;
wire    dimx_rqt_status;
wire    dimx_rsp_status;

assign iimx_rqt_status = (|iimx_adr_o) & iimx_rqt_o;
assign dimx_rqt_status = (|dimx_adr_o) & dimx_rqt_o;
assign iimx_rsp_status = (|iimx_dat_i) & iimx_ack_i;
assign dimx_rsp_status = ((|dimx_dat_i) | dimx_we_o) & dimx_ack_i;

//
// xup-v2p board source
//  Note: When the FPGA drives a logic 0, the corresponding LED turns on. A single four-position DIP 
//  switch and five push buttons are provided for user input. If the DIP switch is up, closed, or on, 
//  or the push button is pressed, a logic 0 is seen by the FPGA, otherwise a logic 1 is indicated.

// signal pushed assert when push push-buttons
assign pushed = !(PB_UP & PB_DOWN & PB_LEFT & PB_RIGHT);

//
//
//
reg [9:0]   num_burn_word;
always @(posedge clk or posedge rst) begin
    if(rst)
        num_burn_word <= 10'd0;
    else if (dsu_sram_we)
        num_burn_word <= num_burn_word + 10'd1;
end

//
// LED & SW
//
//  SW_0: enable heartbreak flashing
//  SW_1: enable imx rqt status flashing
//  SW_2: enable imx rsp status flashing
//  SW_3: enable for dsu burning mode
//
//  LED0: status of clk
//  LED1: status of imx request
//  LED2: status of imx response
//  LED3: status of on-chip ram burning process
assign dsu_rst = pushed;
assign dsu_burn_enable = SW_3;

assign LED0_light = clk_counter[26] & !rst & SW_0;
assign LED1_light = iimx_rqt_status | dimx_rqt_status & SW_1;
assign LED2_light = iimx_rsp_status | dimx_rsp_status & SW_2;
assign LED3_light = |num_burn_word;

assign LED_0 = ! LED0_light;
assign LED_1 = ! LED1_light;
assign LED_2 = ! LED2_light;
assign LED_3 = ! LED3_light;

//
// sys_top
//
top_pss sys_top_pss(
    
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



endmodule


