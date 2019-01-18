// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module dsu_sram_ctrl(
	clk, rst,

	rxd, rxd_ready,

	sram_ce, sram_we, sram_addr, sram_wdata,

	download_enable
);

input   clk;
input   rst;

// receiver signals
input [7:0]	rxd;
input       rxd_ready;

// sram interface
output      sram_ce;
output      sram_we;
output [31:0]	sram_wdata;
output [`IOCM_Word_BW-1:0]	sram_addr;

// control signals
input       download_enable;

//
//
//
reg [`IOCM_Word_BW-1:0] sram_addr;
reg     sram_ce;
reg	    sram_we;

reg [31:0]  receive_data;
reg [1:0]   byte_cnt;

//
// data contenation
//
always@(posedge clk or `dsu_RST_EVENT rst) begin
	if(rst==`dsu_RST_VALUE)
		byte_cnt <= 2'b00;
	else if(byte_cnt == 2'b11 && rxd_ready)
		byte_cnt <= 2'b00;
	else if(rxd_ready)
		byte_cnt <= byte_cnt +1'b1;
end

always@(posedge clk or `dsu_RST_EVENT rst) begin
	if(rst==`dsu_RST_VALUE)
		receive_data <= 32'h0000_0000;
	else if(rxd_ready)
		`ifndef BIGMODEL
				case(byte_cnt)      // little-endian
					2'b00 : receive_data[7:0]	<= rxd;
					2'b01 : receive_data[15:8] 	<= rxd;
					2'b10 : receive_data[23:16]	<= rxd;
					2'b11 : receive_data[31:24]	<= rxd;
				endcase
		`else
				case(byte_cnt)      // big-endian
					2'b11 : receive_data[7:0]	<= rxd;
					2'b10 : receive_data[15:8] 	<= rxd;
					2'b01 : receive_data[23:16]	<= rxd;
					2'b00 : receive_data[31:24]	<= rxd;
				endcase
		`endif	
end	

assign  sram_wdata = receive_data;

// access control signal for synchronous sram interface
always@(posedge clk or `dsu_RST_EVENT rst) begin
	if(rst==`dsu_RST_VALUE) begin
		sram_we <= 1'b0;
		sram_ce <= 1'b0;
    end
	else if(byte_cnt == 2'b11 && rxd_ready) begin
		sram_we <= 1'b1;
		sram_ce <= 1'b1;
    end
	else begin
		sram_we <= 1'b0;
		sram_ce <= 1'b0;
    end
end	
	
// address signals for synchronous sram interface
always@(posedge clk or `dsu_RST_EVENT rst) begin
	if(rst==`dsu_RST_VALUE)
		sram_addr <=`IOCM_Word_BW'd0;
	else if(sram_ce && sram_we)
		sram_addr <= sram_addr + 1'b1;
end
	
endmodule


