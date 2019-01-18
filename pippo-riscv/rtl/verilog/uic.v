`define uic_RST_EVENT     posedge
`define uic_RST_VALUE     1'b1

module uic(
    clk,
    rst,
    
    uicsr_we,
    uicsrs_we,
    uicer_we,
    uiccr_we,
    uicpr_we,
    uictr_we,
    uicvcr_we,
    spr_dat_i,
    
    interrupt_in,
    
    uic_noncrit_intrp,
    uic_crit_intrp,

    
    uicsr,
    uicsrs,
    uicer,
    uiccr,
    uicmsr,
    uicpr,
    uictr,
    uicvr,
    uicvcr
    );
parameter UIC_VECTOR_OFFSET = 512 ;  
 
input    clk;
input    rst;
 //reg write enable   
input    uicsr_we;
input    uicsrs_we;
input    uicer_we;
input    uiccr_we;
input    uicpr_we;
input    uictr_we;
input    uicvcr_we;

input[31:0]    spr_dat_i;
//interrupt input
input[31:0]    interrupt_in;
//interrupt output
output      uic_noncrit_intrp;
output      uic_crit_intrp;
// spr read access
output[31:0]    uicsr;
output[31:0]    uicsrs;
output[31:0]    uicer;
output[31:0]    uiccr;
output[31:0]    uicpr;
output[31:0]    uictr;
output[31:0]    uicvr;
output[31:0]    uicvcr;
output[31:0]    uicmsr;
//status and config register
reg[31:0]    uicsr;
reg[31:0]    uicer;
reg[31:0]    uiccr;
reg[31:0]    uicpr;
reg[31:0]    uictr;

reg[31:0]    uicvcr;

//reg[31:0]    inturrupt_in_d;
reg[31:0]   interrupt_reg;
integer       b,c,i;

// interrupt detect 
/*always@(posedge clk or `uic_RST_EVENT rst)
	if(rst==`uic_RST_VALUE)
		inturrupt_in_d <= 1'b0;
	else 
		inturrupt_in_d <= interrupt_in;
*/		


//uic status register  
//  note: uicsr is set by hardware or uicsrs, read and clear(write-to-clear) by software 
//write uicsrs to set the uicsr,  read uicsrs will return the contents of the uicsr

always @(posedge clk or `uic_RST_EVENT rst)
	if (rst == `uic_RST_VALUE)
	    uicsr<=32'd0;
	else if(uicsr_we)
	    uicsr<=uicsr&(~spr_dat_i);
    else if(uicsrs_we)
	    uicsr<=uicsr|spr_dat_i;
	else
	    uicsr<=uicsr|interrupt_reg;

	    
assign uicsrs=uicsr;

//uic enable register
//uicer is set by software   
always @(posedge clk or `uic_RST_EVENT rst)
	if (rst == `uic_RST_VALUE)
	    uicer<=32'd0;
	else if(uicer_we)
	    uicer<=spr_dat_i;
	    
//uic masked status (register)
//uicmsr is set by hardware   

assign  uicmsr=uicsr&uicer;	    

//uic critical register
//uiccr is set by software   
always @(posedge clk or `uic_RST_EVENT rst)
	if (rst == `uic_RST_VALUE)
	    uiccr<=32'd0;
	else if(uiccr_we)
	    uiccr<=spr_dat_i;
	    
//uic polarity register
//uicpr is set by software
always @(posedge clk or `uic_RST_EVENT rst)
	if (rst == `uic_RST_VALUE)
	    uicpr<=32'd0;
	else if(uicpr_we)
	    uicpr<=spr_dat_i;	    
	    
//uic trigger register
//uictr is set by software
always @(posedge clk or `uic_RST_EVENT rst)
	if (rst == `uic_RST_VALUE)
	    uictr<=32'd0;
	else if(uictr_we)
	    uictr<=spr_dat_i;	


//*******************************************
//interrupts capture  logic
wire[31:0] pol_int;
wire[31:0] pol_int_t;
assign pol_int=interrupt_in ~^ uicpr;
assign pol_int_t=pol_int;
reg[31:0] unsync_int;
reg[31:0] msync_int;
reg[31:0] sync_int;
reg[31:0] sync_int_d;
reg[31:0] dff;
wire[31:0] n_dff;


//*******************************
always @(posedge pol_int_t[0] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[0]<=0;
    else 
        dff[0]<=n_dff[0];

always @(posedge pol_int_t[1] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[1]<=0;
    else 
        dff[1]<=n_dff[1];
always @(posedge pol_int_t[2] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[2]<=0;
    else 
        dff[2]<=n_dff[2];
always @(posedge pol_int_t[3] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[3]<=0;
    else 
        dff[3]<=n_dff[3];
always @(posedge pol_int_t[4] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[4]<=0;
    else 
        dff[4]<=n_dff[4];
always @(posedge pol_int_t[5] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[5]<=0;
    else 
        dff[5]<=n_dff[5];
always @(posedge pol_int_t[6] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[6]<=0;
    else 
        dff[6]<=n_dff[6];
always @(posedge pol_int_t[7] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[7]<=0;
    else 
        dff[7]<=n_dff[7];        
always @(posedge pol_int_t[8] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[8]<=0;
    else 
        dff[8]<=n_dff[8];
always @(posedge pol_int_t[9] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[9]<=0;
    else 
        dff[9]<=n_dff[9];
always @(posedge pol_int_t[10] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[10]<=0;
    else 
        dff[10]<=n_dff[10];
always @(posedge pol_int_t[11] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[11]<=0;
    else 
        dff[11]<=n_dff[11];
always @(posedge pol_int_t[12] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[12]<=0;
    else 
        dff[12]<=n_dff[12];
always @(posedge pol_int_t[13] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[13]<=0;
    else 
        dff[13]<=n_dff[13];
always @(posedge pol_int_t[14] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[14]<=0;
    else 
        dff[14]<=n_dff[14];
always @(posedge pol_int_t[15] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[15]<=0;
    else 
        dff[15]<=n_dff[15];
always @(posedge pol_int_t[16] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[16]<=0;
    else 
        dff[16]<=n_dff[16];
always @(posedge pol_int_t[17] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[17]<=0;
    else 
        dff[17]<=n_dff[17];
always @(posedge pol_int_t[18] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[18]<=0;
    else 
        dff[18]<=n_dff[18];
always @(posedge pol_int_t[19] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[19]<=0;
    else 
        dff[19]<=n_dff[19];
always @(posedge pol_int_t[20] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[20]<=0;
    else 
        dff[20]<=n_dff[20];
always @(posedge pol_int_t[21] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[21]<=0;
    else 
        dff[21]<=n_dff[21];
always @(posedge pol_int_t[22] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[22]<=0;
    else 
        dff[22]<=n_dff[22];
always @(posedge pol_int_t[23] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[23]<=0;
    else 
        dff[23]<=n_dff[23];
always @(posedge pol_int_t[24] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[24]<=0;
    else 
        dff[24]<=n_dff[24];
always @(posedge pol_int_t[25] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[25]<=0;
    else 
        dff[25]<=n_dff[25];
always @(posedge pol_int_t[26] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[26]<=0;
    else 
        dff[26]<=n_dff[26];
always @(posedge pol_int_t[27] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[27]<=0;
    else 
        dff[27]<=n_dff[27];
always @(posedge pol_int_t[28] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[28]<=0;
    else 
        dff[28]<=n_dff[28];
always @(posedge pol_int_t[29] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[29]<=0;
    else 
        dff[29]<=n_dff[29];
always @(posedge pol_int_t[30] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[30]<=0;
    else 
        dff[30]<=n_dff[30];
always @(posedge pol_int_t[31] or `uic_RST_EVENT rst)
    if (rst == `uic_RST_VALUE)
        dff[31]<=0;
    else 
        dff[31]<=n_dff[31];

assign n_dff=~dff;
//*******************************
always@(uictr or dff or pol_int)
    for (b=0;b<32;b=b+1)
        unsync_int[b]<=uictr? dff[b] : pol_int[b];
 //first sync latch       
always @(posedge clk or `uic_RST_EVENT rst )
    if (rst == `uic_RST_VALUE)
        msync_int<=32'd0;
    else    
        msync_int<=unsync_int;   
//second sync latch
always @(posedge clk or `uic_RST_EVENT rst )
    if (rst == `uic_RST_VALUE)
        sync_int<=32'd0;
    else
        sync_int<=msync_int;

// sync_int delay
always @(posedge clk or `uic_RST_EVENT rst )
    if (rst == `uic_RST_VALUE)
        sync_int_d<=32'd0;
    else
        sync_int_d<=sync_int;  
   
always@(pol_int or sync_int_d or sync_int or uictr)
    for (c=0;c<32;c=c+1)
        case(uictr[c])  
            1'b0: interrupt_reg[c] <= sync_int[c];
            1'b1: interrupt_reg[c] <= sync_int[c]^sync_int_d[c];
        endcase             
//  interrupt bit  generate
/*always @(posedge clk or `uic_RST_EVENT rst)
	if (rst == `uic_RST_VALUE)
	    interrupt_reg<=32'd0;
	else  interrupt_reg<=(
	                    ((~inturrupt_in_d&inturrupt_in&uicpr)|
	                    (inturrupt_in_d&~inturrupt_in&~uicpr))
	                    &uictr                                  //uictr"1"  edge sensitive
	                    )|
	                    ((inturrupt_in~^uicpr)&~uictr);         //uictr"0"  level sensitive
	                   
always @(inturrupt_in or inturrupt_in_d or uicpr or uictr)
interrupt_reg=(
	                    ((~inturrupt_in_d&inturrupt_in&uicpr)|
	                    (inturrupt_in_d&~inturrupt_in&~uicpr))
	                    &uictr                                  //uictr"1"  edge sensitive
	                    )|
	                    ((inturrupt_in~^uicpr)&~uictr);         //uictr"0"  level sensitive
*/

//uic vector configuration register
//uicvcr is set by software uicvcr[0] set to "0" that the LSB of the uicsr is the highest priority 
always @(posedge clk or `uic_RST_EVENT rst)
	if (rst == `uic_RST_VALUE)
	    uicvcr<=32'd0;
	else if(uicvcr_we)
	    uicvcr<=spr_dat_i;	
	    
//************************************************************
// Vector Generation Logic    	

// interrupt must be active, enabled and critical
//assign critl_status = status_reg & enable_reg & critical_reg;
	    
wire[31:0] 	critl_status; 
reg[31:0] 	prior_bits;   
reg[31:0] 	m_prior_bits;
reg[4:0]    bit_num;
assign critl_status = uicsr & uicer & uiccr;

always @(uicvcr or critl_status)
        for (i=0; i<32; i=i+1)
         prior_bits[i] <= uicvcr[0] ?
           critl_status[i] :
             critl_status[31-i];
             
always@(prior_bits)
      m_prior_bits <= prior_bits;

/* generate a multiplier based on the bit position of the highest */
/* priority critical interrupt */
always @(m_prior_bits) begin
        casez (m_prior_bits)
                32'b1???????????????????????????????: bit_num=5'b00000;
                32'b01??????????????????????????????: bit_num=5'b00001;
                32'b001?????????????????????????????: bit_num=5'b00010;
                32'b0001????????????????????????????: bit_num=5'b00011;
                32'b00001???????????????????????????: bit_num=5'b00100;
                32'b000001??????????????????????????: bit_num=5'b00101;
                32'b0000001?????????????????????????: bit_num=5'b00110;
                32'b00000001????????????????????????: bit_num=5'b00111;
                32'b000000001???????????????????????: bit_num=5'b01000;
                32'b0000000001??????????????????????: bit_num=5'b01001;
                32'b00000000001?????????????????????: bit_num=5'b01010;
                32'b000000000001????????????????????: bit_num=5'b01011;
                32'b0000000000001???????????????????: bit_num=5'b01100;
                32'b00000000000001??????????????????: bit_num=5'b01101;
                32'b000000000000001?????????????????: bit_num=5'b01110;
                32'b0000000000000001????????????????: bit_num=5'b01111;
                32'b00000000000000001???????????????: bit_num=5'b10000;
                32'b000000000000000001??????????????: bit_num=5'b10001;
                32'b0000000000000000001?????????????: bit_num=5'b10010;
                32'b00000000000000000001????????????: bit_num=5'b10011;
                32'b000000000000000000001???????????: bit_num=5'b10100;
                32'b0000000000000000000001??????????: bit_num=5'b10101;
                32'b00000000000000000000001?????????: bit_num=5'b10110;
                32'b000000000000000000000001????????: bit_num=5'b10111;
                32'b0000000000000000000000001???????: bit_num=5'b11000;
                32'b00000000000000000000000001??????: bit_num=5'b11001;
                32'b000000000000000000000000001?????: bit_num=5'b11010;
                32'b0000000000000000000000000001????: bit_num=5'b11011;
                32'b00000000000000000000000000001???: bit_num=5'b11100;
                32'b000000000000000000000000000001??: bit_num=5'b11101;
                32'b0000000000000000000000000000001?: bit_num=5'b11110;
                32'b00000000000000000000000000000001: bit_num=5'b11111;
                32'b00000000000000000000000000000000: bit_num=5'b00000;
                                             default: bit_num=5'b00000;
        endcase
end

wire[31:0]  vec_base;
wire[31:0]  bit_offset;
wire[31:0]  uicvr;
/* if there are no critical interrupts set address to 0's */
assign vec_base = (m_prior_bits==32'h00000000) ? 32'h00000000 :
                        {uicvcr[31:2], 2'b00};     

assign bit_offset = UIC_VECTOR_OFFSET * ({27'h0,bit_num[4:0]});     

/* calculate vector address  */
assign uicvr = vec_base + bit_offset;                           
 
/* critical and non_critical interrupts to core */
assign uic_crit_intrp    = | (uicsr & uicer & uiccr);
assign uic_noncrit_intrp = | (uicsr & uicer & ~uiccr);    
    
endmodule	    