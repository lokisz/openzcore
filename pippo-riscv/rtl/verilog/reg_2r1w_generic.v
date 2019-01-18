/*
 * File:        reg_2r1w_generic.v
 * Project:     pippo
 * Designer:    fang@ali
 * Mainteiner:  fang@ali
 * Checker:
 * Description:
 *      Register file: 32 GPR£¬ 2r1w access port
 * Note:
 *      1) The read port is edge-trigger for address, the data is available at next cycle. 
 *      2) The write port is edge-trigger, so the value is readable in next cycle
 *  
 */
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module reg_2r1w_generic(

			clk,
			rst,
			
			rda_en,
			rda_addr,
			rda_do,
			rdb_en,
			rdb_addr,
			rdb_do,
			
			wr_en,
			wr_addr,
			wr_data
);

/*
`ifdef USE_TRISTATE
    `define WordZ                   32'bzzzz_zzzz_zzzz_zzzz
`else
    `define WordZ                   32'b0000_0000_0000_0000
`endif
*/    
`define WordZero                `OPERAND_WIDTH'd0
parameter width = `OPERAND_WIDTH;

//
// I/O
//
input   clk;
input   rst;

// read access
input   rda_en;
input   rdb_en;
input   rdc_en;
input   [4:0]   rda_addr;
input   [4:0]   rdb_addr;
input   [4:0]   rdc_addr;

output  [width-1:0]  rda_do;
output  [width-1:0]  rdb_do;
output  [width-1:0]  rdc_do;

// write access
input   wr_en;
input   [4:0]   wr_addr;
input   [width-1:0]  wr_data;

//
// 32 gprs
//      
reg [width-1:0]  gprs0;
reg [width-1:0]  gprs1;
reg [width-1:0]  gprs2;
reg [width-1:0]  gprs3;
reg [width-1:0]  gprs4;
reg [width-1:0]  gprs5;
reg [width-1:0]  gprs6;
reg [width-1:0]  gprs7;
reg [width-1:0]  gprs8;
reg [width-1:0]  gprs9;
reg [width-1:0]  gprs10;
reg [width-1:0]  gprs11;
reg [width-1:0]  gprs12;
reg [width-1:0]  gprs13;
reg [width-1:0]  gprs14;
reg [width-1:0]  gprs15;
reg [width-1:0]  gprs16;
reg [width-1:0]  gprs17;
reg [width-1:0]  gprs18;
reg [width-1:0]  gprs19;
reg [width-1:0]  gprs20;
reg [width-1:0]  gprs21;
reg [width-1:0]  gprs22;
reg [width-1:0]  gprs23;
reg [width-1:0]  gprs24;
reg [width-1:0]  gprs25;
reg [width-1:0]  gprs26;
reg [width-1:0]  gprs27;
reg [width-1:0]  gprs28;
reg [width-1:0]  gprs29;
reg [width-1:0]  gprs30;
reg [width-1:0]  gprs31;

reg [width-1:0]  rda_do;
reg [width-1:0]  rdb_do;
reg [4:0]   ex_rda_addr;
reg [4:0]   ex_rdb_addr;

wire        rda_en;
wire        rdb_en;
     
//
// Logic
//
always @(posedge clk or posedge rst) begin
	if (rst) begin
		ex_rda_addr <= #1 5'h00;
	end
	else if (rda_en) begin
		ex_rda_addr <= #1 rda_addr;
	end
end

always @(posedge clk or posedge rst) begin
	if (rst) begin
		ex_rdb_addr <= #1 5'h00;
	end
	else if (rdb_en) begin
		ex_rdb_addr <= #1 rdb_addr;
	end
end


// A port for read
always @(*) begin
	case (ex_rda_addr)
	5'b0_0000:
		rda_do = gprs0;
	5'b0_0001:
		rda_do = gprs1;
	5'b0_0010:
		rda_do = gprs2;
	5'b0_0011:
		rda_do = gprs3;
	5'b0_0100:
		rda_do = gprs4;
	5'b0_0101:
		rda_do = gprs5;
	5'b0_0110:
		rda_do = gprs6;
	5'b0_0111:
		rda_do = gprs7;
	5'b0_1000:
		rda_do = gprs8;
	5'b0_1001:
		rda_do = gprs9;
	5'b0_1010:
		rda_do = gprs10;
	5'b0_1011:
		rda_do = gprs11;		
	5'b0_1100:
		rda_do = gprs12;
	5'b0_1101:
		rda_do = gprs13;
	5'b0_1110:
		rda_do = gprs14;
	5'b0_1111:
		rda_do = gprs15;
	5'b1_0000:
		rda_do = gprs16;
	5'b1_0001:
		rda_do = gprs17;
	5'b1_0010:
		rda_do = gprs18;
	5'b1_0011:
		rda_do = gprs19;
	5'b1_0100:
		rda_do = gprs20;
	5'b1_0101:
		rda_do = gprs21;
	5'b1_0110:
		rda_do = gprs22;
	5'b1_0111:
		rda_do = gprs23;
	5'b1_1000:
		rda_do = gprs24;
	5'b1_1001:
		rda_do = gprs25;
	5'b1_1010:
		rda_do = gprs26;
	5'b1_1011:
		rda_do = gprs27;		
	5'b1_1100:
		rda_do = gprs28;
	5'b1_1101:
		rda_do = gprs29;
	5'b1_1110:
		rda_do = gprs30;
	5'b1_1111:
		rda_do = gprs31;
	endcase
end

// B port for read
always @(*) begin
    case (ex_rdb_addr)
		5'b0_0000:
			rdb_do = gprs0;
		5'b0_0001:
			rdb_do = gprs1;
		5'b0_0010:
			rdb_do = gprs2;
		5'b0_0011:
			rdb_do = gprs3;
		5'b0_0100:
			rdb_do = gprs4;
		5'b0_0101:
			rdb_do = gprs5;
		5'b0_0110:
			rdb_do = gprs6;
		5'b0_0111:
			rdb_do = gprs7;
		5'b0_1000:
			rdb_do = gprs8;
		5'b0_1001:
			rdb_do = gprs9;
		5'b0_1010:
			rdb_do = gprs10;
		5'b0_1011:
			rdb_do = gprs11;		
		5'b0_1100:
			rdb_do = gprs12;
		5'b0_1101:
			rdb_do = gprs13;
		5'b0_1110:
			rdb_do = gprs14;
		5'b0_1111:
			rdb_do = gprs15;
		5'b1_0000:
			rdb_do = gprs16;
		5'b1_0001:
			rdb_do = gprs17;
		5'b1_0010:
			rdb_do = gprs18;
		5'b1_0011:
			rdb_do = gprs19;
		5'b1_0100:
			rdb_do = gprs20;
		5'b1_0101:
			rdb_do = gprs21;
		5'b1_0110:
			rdb_do = gprs22;
		5'b1_0111:
			rdb_do = gprs23;
		5'b1_1000:
			rdb_do = gprs24;
		5'b1_1001:
			rdb_do = gprs25;
		5'b1_1010:
			rdb_do = gprs26;
		5'b1_1011:
			rdb_do = gprs27;		
		5'b1_1100:
			rdb_do = gprs28;
		5'b1_1101:
			rdb_do = gprs29;
		5'b1_1110:
			rdb_do = gprs30;
		5'b1_1111:
			rdb_do = gprs31;
	endcase
end


// Write Port
always @(posedge clk or posedge rst) begin
	if(rst)
	begin
		//initial the register file
		gprs0 <= `WordZero;
		gprs1 <= `WordZero;
		gprs2 <= `WordZero;
		gprs3 <= `WordZero;
		gprs4 <= `WordZero;
		gprs5 <= `WordZero;
		gprs6 <= `WordZero;
		gprs7 <= `WordZero;
		gprs8 <= `WordZero;
		gprs9 <= `WordZero;
		gprs10 <= `WordZero;
		gprs11 <= `WordZero;
		gprs12 <= `WordZero;
		gprs13 <= `WordZero;
		gprs14 <= `WordZero;
		gprs15 <= `WordZero;
		gprs16 <= `WordZero;
		gprs17 <= `WordZero;
		gprs18 <= `WordZero;
		gprs19 <= `WordZero;
		gprs20 <= `WordZero;
		gprs21 <= `WordZero;
		gprs22 <= `WordZero;
		gprs23 <= `WordZero;
		gprs24 <= `WordZero;
		gprs25 <= `WordZero;
		gprs26 <= `WordZero;
		gprs27 <= `WordZero;
		gprs28 <= `WordZero;
		gprs29 <= `WordZero;
		gprs30 <= `WordZero;
		gprs31 <= `WordZero;
	end
	else begin
        if(wr_en==1'b1) begin
		   case (wr_addr)
			5'b0_0000:
				gprs0 <= wr_data;
			5'b0_0001:
				gprs1 <= wr_data;
			5'b0_0010:
				gprs2 <= wr_data;
			5'b0_0011:
				gprs3 <= wr_data;
			5'b0_0100:
				gprs4 <= wr_data;
			5'b0_0101:
				gprs5 <= wr_data;
			5'b0_0110:
				gprs6 <= wr_data;
			5'b0_0111:
				gprs7 <= wr_data;
			5'b0_1000:
				gprs8 <= wr_data;
			5'b0_1001:
			    gprs9 <= wr_data;
			5'b0_1010:
				gprs10 <= wr_data;
			5'b0_1011:
				gprs11 <= wr_data;
			5'b0_1100:
				gprs12 <= wr_data;
			5'b0_1101:
				gprs13 <= wr_data;
			5'b0_1110:
				gprs14 <= wr_data;
			5'b0_1111:
				gprs15 <= wr_data;
			5'b1_0000:
				gprs16 <= wr_data;
			5'b1_0001:
				gprs17 <= wr_data;
			5'b1_0010:
				gprs18 <= wr_data;
			5'b1_0011:
				gprs19 <= wr_data;
			5'b1_0100:
				gprs20 <= wr_data;
			5'b1_0101:
				gprs21 <= wr_data;
			5'b1_0110:
				gprs22 <= wr_data;
			5'b1_0111:
				gprs23 <= wr_data;
			5'b1_1000:
				gprs24 <= wr_data;
			5'b1_1001:
			    gprs25 <= wr_data;
			5'b1_1010:
				gprs26 <= wr_data;
			5'b1_1011:
				gprs27 <= wr_data;
			5'b1_1100:
				gprs28 <= wr_data;
			5'b1_1101:
				gprs29 <= wr_data;
			5'b1_1110:
				gprs30 <= wr_data;
			5'b1_1111:
				gprs31 <= wr_data;	
		   endcase
		end
	end
end

endmodule


