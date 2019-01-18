/*
 * File:        pippo_timer.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description:
    一，实现PowerPC Embedded Architecture的二个物理计数器和三个定时器逻辑：
        物理计数器Time Base：64位，递增；TB的event有：
            Watchdog events: TBL[28], TBL[24], TBL[20], TBL[16]; 
            FIT events: TBL[20], TBL[16], TBL[12], TBL[8]; 
        物理计数器PIT：32位，递减；当检测到全零时，发出PIT event            
    二，实现PowerPC Embedded Architecture的定时器寄存器和相关逻辑
        Timer Status Register(TSR)
        Timer Control Register(TCR)
   Task.I
        [TBV]tsr[enw]在每次watchdog timeout发生时置起？
        [TBD]tsr[pis]是在中断发生后才置起？还是发出中断请求即可？
        参考中断SPEC的Timer相关部分，确定event和tsr设置等时序
 */
 
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_timer(
	clk, rst, 

    tbl, tbu, pit, tsr, tcr, 
    tbl_we, tbu_we, pit_we, tsr_we, tcr_we, 
    spr_dat_i, 
    
	sig_pit, sig_fit, sig_watchdog,
	rqt_core_rst, rqt_sys_rst, rqt_chip_rst
);

input		clk;		
input		rst;		

// spr write access
input   tbl_we;
input   tbu_we;
input   pit_we;
input   tsr_we;
input   tcr_we;
input	[31:0]	spr_dat_i;	

// spr read access
output	[31:0]	tbl;	
output	[31:0]	tbu;	
output	[31:0]  pit; 
output	[31:0]  tsr; 
output	[31:0]  tcr; 

// interrupt request
output          sig_fit;
output		    sig_pit;
output          sig_watchdog;

// reset request
output          rqt_core_rst;
output          rqt_sys_rst;
output          rqt_chip_rst;

//
// physical counter and control register
//
reg	[32:0]	tbl;	
reg	[31:0]	tbu;	
reg [31:0]  pit; 
reg [31:0]  pit_lastload; 
reg [31:0]  tsr; 
reg [31:0]  tcr; 

wire        watchdog_timeout;
wire [1:0]  event_watchdog_rst; 
wire        event_watchdog, event_pit, event_fit;

// tsr & tcr
wire    [1:0]   tsr_wrs;
wire            tsr_enw, tsr_wis, tsr_pis, tsr_fis;
wire    [1:0]   tcr_wrc;
wire    [1:0]   tcr_wp;
wire    [1:0]   tcr_fp;
wire            tcr_wie, tcr_pie, tcr_fie, tcr_are;

wire    [32:0]  tbl_new;

//
// timer status register
//  note: tsr is set by hardware, read and clear(write-to-clear) by software 
//

always @(posedge clk or `pippo_RST_EVENT rst)
	if (rst == `pippo_RST_VALUE)
		tsr <= 32'd0;
	else if (tsr_we)
		tsr <=  tsr & ~spr_dat_i;
    else 
        tsr <= {watchdog_timeout, event_watchdog, event_watchdog_rst, event_pit, event_fit, tsr[25:0]};

// tsr output
assign tsr_enw = tsr[`pippo_TSR_ENW_BITS];      // enable next watchdog
assign tsr_wis = tsr[`pippo_TSR_WIS_BITS];      // watchdog interrupt status
assign tsr_wrs = tsr[`pippo_TSR_WRS_BITS];      // watchdog reset status
assign tsr_pis = tsr[`pippo_TSR_PIS_BITS];      // pit interrupt status
assign tsr_fis = tsr[`pippo_TSR_FIS_BITS];      // fit interrupt status

//
// timer control register
//  note: tcr_wrc can only be clear by rst
//  [TBV] how about write to wrc twice by different value?
//
always @(posedge clk or `pippo_RST_EVENT rst)
	if (rst == `pippo_RST_VALUE)
		tcr <= 32'd0;
	else if (tcr_we)
		tcr <=  {spr_dat_i[31:30], tcr_wrc | spr_dat_i[`pippo_TCR_WRC_BITS], spr_dat_i[27:0]};

assign tcr_wp = tcr[`pippo_TCR_WP_BITS];        // watchdog period
assign tcr_wrc = tcr[`pippo_TCR_WRC_BITS];      // watchdog reset control
assign tcr_wie = tcr[`pippo_TCR_WIE_BITS];      // watchdog interrupt enable
assign tcr_pie = tcr[`pippo_TCR_PIE_BITS];      // pit interrupt enable
assign tcr_fp = tcr[`pippo_TCR_FP_BITS];        // fit period
assign tcr_fie = tcr[`pippo_TCR_FIE_BITS];      // fit interrupt enable
assign tcr_are = tcr[`pippo_TCR_ARE_BITS];      // auto reload enable

//
// Time Base 
//
assign tbl_new = tbl + 33'd1; 

always @(posedge clk or `pippo_RST_EVENT rst)
	if (rst == `pippo_RST_VALUE)
		tbl <= 33'd0;
	else if (tbl_we)
		tbl <=  {1'b0, spr_dat_i};
	else 
		tbl <= tbl_new;

assign tbl_carry = tbl_new[32] ^ tbl[32]; 

always @(posedge clk or `pippo_RST_EVENT rst)
	if (rst == `pippo_RST_VALUE)
		tbu <= 32'd0;
	else if (tbu_we)
		tbu <=  spr_dat_i;
	else 
		tbu <= tbu + tbl_carry;

//
// watchdog timeout
//
assign watchdog_timeout = 
        ((tcr_wp == 2'b00) & tbl[17]) | ((tcr_wp == 2'b01) & tbl[21]) |
        ((tcr_wp == 2'b10) & tbl[25]) | ((tcr_wp == 2'b11) & tbl[29]);
assign event_watchdog = watchdog_timeout & ~tsr_wis & tsr_enw;          // [tbv]
assign sig_watchdog = watchdog_timeout & ~tsr_wis & tsr_enw & tcr_wie; 

// watchdog reset logic
assign rqt_core_rst = watchdog_timeout & tsr_wis & tsr_enw & (tcr_wrc == 2'b01); 
assign rqt_chip_rst = watchdog_timeout & tsr_wis & tsr_enw & (tcr_wrc == 2'b10); 
assign rqt_sys_rst = watchdog_timeout & tsr_wis & tsr_enw & (tcr_wrc == 2'b11); 

assign event_watchdog_rst[1] = rqt_sys_rst | rqt_chip_rst;
assign event_watchdog_rst[0] = rqt_core_rst | rqt_sys_rst;

//
// FIT
//
assign event_fit = 
        ((tcr_fp == 2'b00) & tbl[9]) | ((tcr_fp == 2'b01) & tbl[13]) |
        ((tcr_fp == 2'b10) & tbl[17]) | ((tcr_fp == 2'b11) & tbl[21]);
assign sig_fit = event_fit & tcr_fie; 

//
// PIT
//
always @(posedge clk or `pippo_RST_EVENT rst)
	if (rst == `pippo_RST_VALUE)
		pit_lastload <= 32'd0;
	else if (pit_we)
		pit_lastload <=  spr_dat_i;

always @(posedge clk or `pippo_RST_EVENT rst)
	if (rst == `pippo_RST_VALUE)
		pit <= 32'd0;
	else if (pit_we)
		pit <=  spr_dat_i;
    else if (event_pit & tcr_are)
        pit <= pit_lastload;
	else if (|pit)
		pit <= pit - 32'd1;

assign event_pit = ~(|pit[31:1]) & pit[0];
assign sig_pit = event_pit & tcr_pie; 


endmodule
