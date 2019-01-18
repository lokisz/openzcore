/*
 * Programing Notes:
   1, there are four SPRs in uartlite, access using mtspr/mfspr instructions
   2, the four register are:
        control register
            ctrl[0]: tx_start
            ctrl[1]: rx_disenable
            ctrl[2]: tx_interrupt_ebable - currently not in use
            ctrl[3]: rx_interrupt_enable - currently not in use
            ctrl[4]: interrupt status clear	
            Reset Status: 8'h00
            bit[0]/[4] are auto-cleared when writing true-"1" into it.
        status register - READ ONLY
            sta[0]: transmitter status which 1 means transmitter is busy, 0 means transmitter is idle
            sta[1]: receiver status which 1 means receiver is busy, 0 means receiver is idle
            sta[2]: interrupt status which 1 mean interrupt status, 0 means normal status
            sta[3]: transmitter interrupt status which means transmitter interrupt
            sta[4]: receiver interrupt status which means receiver interrupt
        receive data register
        transmit data register
 * Note: 
        1, there is no flow control, after received a byte, if second byte is coming, first byte is covered
 */

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module dsu_uartlite(
    
    clk, rst, 
    
    txd, rxd,
    
    spr_dat_i, 
    reg_txdata,  reg_txdata_we, 
    reg_ctrl, reg_ctrl_we,
    reg_sta, reg_sta_we,
    reg_rxdata, reg_rxdata_we,
    
    sram_ce, 
    sram_we,
    sram_addr,
    sram_wdata,
    
    download_enable   	
);
	
//
input   clk;
input   rst;

// uart inout	
input  rxd;
output txd;

// spr access interface
input   [7:0]   spr_dat_i;
input           reg_txdata_we;
input           reg_ctrl_we;
input           reg_rxdata_we;
input           reg_sta_we;

output  [7:0]   reg_txdata;
output  [7:0]   reg_ctrl;
output  [7:0]   reg_sta;
output  [7:0]   reg_rxdata;

// sram interface
output sram_ce;
output sram_we; 
output [63:0] sram_wdata;
output [`IOCM_Word_BW-1:0] sram_addr; 

// backdoor control
input	download_enable;

//
// four internal SPRs for uartlite
// 
reg     [7:0]	reg_ctrl;	    
reg     [7:0]	reg_sta;	    
reg     [7:0]	reg_txdata;
reg     [7:0]  	reg_rxdata;

//
wire		tx_start;
wire		rx_enable;
reg			tx_busy_d;													
reg 		interrupt_tx;
reg			interrupt_rx;			

//
//
//
wire txd;
wire rxd;
wire [7:0] rx_data;
wire [7:0] rx_data_to_sram;
wire rx_data_rdy;
wire tx_busy;
wire rx_idle;
wire interrupt;

//
// SPR: control register
//
// [TBV]: coding style of auto-cleared
always@(posedge clk or `dsu_RST_EVENT rst) begin
	if(rst==`dsu_RST_VALUE)
		reg_ctrl  <= 8'h00;
	else if(reg_ctrl[0])        //  the tx_enable bit (ctrl[0]) will auto cleared after write 1
		reg_ctrl[0] <= 1'b0;
	else if(reg_ctrl_we)
		reg_ctrl <= spr_dat_i;
	else if(reg_ctrl[4])        //  the int_clear bit (ctrl[4]) will auto cleared after write 1
		reg_ctrl[4] <= 1'b0;
end

assign tx_start = reg_ctrl[0];
assign rx_enable = !reg_ctrl[1];															
assign int_clear = reg_ctrl[4];															

//
// SPR: status register
//
// tx interrupt detect 
always@(posedge clk or `dsu_RST_EVENT rst)
	if(rst==`dsu_RST_VALUE)
		tx_busy_d <= 1'b0;
	else 
		tx_busy_d <= tx_busy;

//when detect the negedge of tx_busy that means transmitter is finished
//if the tx_interrupt enable then generate interrupt of transmitter.
always@(posedge clk or `dsu_RST_EVENT rst)
	if(rst==`dsu_RST_VALUE)
  	    interrupt_tx <= 1'b0;
    else if(!tx_busy && tx_busy_d)  
  	    interrupt_tx <= 1'b1;
    else if(int_clear)
  	    interrupt_tx <= 1'b0;
 		 
always@(posedge clk or `dsu_RST_EVENT rst)
	if(rst==`dsu_RST_VALUE)
		interrupt_rx <= 1'b0;
	else if(rx_data_rdy && rx_enable)
		interrupt_rx <= 1'b1;
	else if(int_clear)
		interrupt_rx <= 1'b0;
    
assign interrupt = interrupt_rx || interrupt_tx;												

always@(posedge clk or `dsu_RST_EVENT rst)
	if(rst==`dsu_RST_VALUE)
  	    reg_sta <= 8'h00;
    else if(reg_sta_we)
  	    reg_sta <= spr_dat_i;
    else
  	    reg_sta <= {3'b000, interrupt_rx, interrupt_tx, interrupt, !rx_idle, tx_busy};

//
// SPR: receive data register
//
always@(posedge clk or `dsu_RST_EVENT rst) begin
	if(rst==`dsu_RST_VALUE)
        reg_rxdata <= 8'h00;
    else if(rx_data_rdy && rx_enable ) 
  	    reg_rxdata <= rx_data;
end

//
// SPR: transmit data register
//
always@(posedge clk or `dsu_RST_EVENT rst) begin	
    if(rst==`dsu_RST_VALUE)
		reg_txdata  <= 8'h00;
	else if(reg_txdata_we)
		reg_txdata <= spr_dat_i;
end 
    	
//
// transmitter and receiver 
//
dsu_Tx tx(
    .clk(clk), 
    .rst(rst), 
    .TxD_start(tx_start), 
    .TxD_data(reg_txdata), 
    .TxD(txd), 
    .TxD_busy(tx_busy)
);

dsu_Rx rx(
    .clk(clk), 
    .rst(rst), 
    .RxD(rxd), 
    .RxD_data_ready(rx_data_rdy), 
    .RxD_data(rx_data), 
    .RxD_endofpacket(), 
    .RxD_idle(rx_idle)
);

//
// back door mode: burn sram using received data
//
assign rx_data_to_sram = download_enable ? rx_data : 8'h00;

dsu_sram_ctrl sram_ctrl(
    .clk(clk),
    .rst(rst),
    .rxd(rx_data_to_sram),
    .rxd_ready(rx_data_rdy && download_enable),
    .sram_ce(sram_ce),
    .sram_we(sram_we),
    .sram_addr(sram_addr),
    .sram_wdata(sram_wdata),
    .download_enable(download_enable)
);
	
endmodule