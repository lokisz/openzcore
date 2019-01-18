// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module tb_dsu_tx(clk, rst, TxD_start, TxD_data, TxD, TxD_busy);
input clk, rst, TxD_start;
input [7:0] TxD_data;
output TxD, TxD_busy;


parameter Baud = 115200;
//parameter Baud = 9600;
parameter RegisterInputData = 1;	// in RegisterInputData mode, the input doesn't have to stay valid while the character is been transmitted

// Baud generator
parameter BaudGeneratorAccWidth = 16;
reg [BaudGeneratorAccWidth:0] BaudGeneratorAcc;

`ifdef FPGA_50MHZ
wire [BaudGeneratorAccWidth:0] BaudGeneratorInc = 17'h00097;  //for 115200 BPS at 50MHz.
`endif

`ifdef FPGA_32MHZ
wire [BaudGeneratorAccWidth:0] BaudGeneratorInc = 17'h000ec;  //for 115200 BPS at 32MHZ
`endif

`ifdef FPGA_64MHZ
wire [BaudGeneratorAccWidth:0] BaudGeneratorInc = 17'h00076;	//for 115200 BPS at 64MHZ
`endif

wire BaudTick = BaudGeneratorAcc[BaudGeneratorAccWidth];
wire TxD_busy;

always@(posedge clk or `dsu_RST_EVENT rst)		
   begin
    if(rst==`dsu_RST_VALUE)
    	 BaudGeneratorAcc <= 16'h0000;
    else if(TxD_busy) 
       BaudGeneratorAcc <= BaudGeneratorAcc[BaudGeneratorAccWidth-1:0] + BaudGeneratorInc;
   end

// Transmitter state machine
reg [3:0] state;
wire TxD_ready = (state==0);
assign TxD_busy = ~TxD_ready;

reg [7:0] TxD_dataReg;
always@(posedge clk or `dsu_RST_EVENT rst)		
   begin
    if(rst==`dsu_RST_VALUE)
    TxD_dataReg <= 8'h00;
  else if(TxD_ready & TxD_start) 
    TxD_dataReg <= TxD_data;
  end
wire [7:0] TxD_dataD = RegisterInputData ? TxD_dataReg : TxD_data;

always@(posedge clk or `dsu_RST_EVENT rst)		
   begin
    if(rst==`dsu_RST_VALUE)
       state <= 4'h0;
    else
			case(state)
				4'b0000: if(TxD_start) state <= 4'b0001;
				4'b0001: if(BaudTick) state <= 4'b0100;
				4'b0100: if(BaudTick) state <= 4'b1000;  // start
				4'b1000: if(BaudTick) state <= 4'b1001;  // bit 0
				4'b1001: if(BaudTick) state <= 4'b1010;  // bit 1
				4'b1010: if(BaudTick) state <= 4'b1011;  // bit 2
				4'b1011: if(BaudTick) state <= 4'b1100;  // bit 3
				4'b1100: if(BaudTick) state <= 4'b1101;  // bit 4
				4'b1101: if(BaudTick) state <= 4'b1110;  // bit 5
				4'b1110: if(BaudTick) state <= 4'b1111;  // bit 6
				4'b1111: if(BaudTick) state <= 4'b0010;  // bit 7
				4'b0010: if(BaudTick) state <= 4'b0011;  // stop1	
				4'b0011: if(BaudTick) state <= 4'b0000;  // stop2
				default: if(BaudTick) state <= 4'b0000;
			endcase
	end

// Output mux
reg muxbit;
always @ ( * )
 begin
			case(state[2:0])
				3'd0: muxbit <= TxD_dataD[0];
				3'd1: muxbit <= TxD_dataD[1];	
				3'd2: muxbit <= TxD_dataD[2];
				3'd3: muxbit <= TxD_dataD[3];
				3'd4: muxbit <= TxD_dataD[4];
				3'd5: muxbit <= TxD_dataD[5];
				3'd6: muxbit <= TxD_dataD[6];
				3'd7: muxbit <= TxD_dataD[7];
			endcase
 end
// Put together the start, data and stop bits
reg TxD;
always@(posedge clk or `dsu_RST_EVENT rst)		
   begin
    if(rst==`dsu_RST_VALUE)
         TxD <= 1'b1;
      else  
 				 TxD <= (state<4) | (state[3] & muxbit);  // register the output to make it glitch free
 	end

endmodule