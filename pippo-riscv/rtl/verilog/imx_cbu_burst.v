/*
 * File:        imx_cbu_burst.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description: Core Bridge Unit
 *      桥接core的imx和片上互连协议
 * Task.I:
 */
 
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module imx_cbu_burst(
    clk, rst, clmode,
    
    wb_clk_i, wb_rst_i, wb_ack_i, wb_err_i, wb_rty_i, wb_dat_i,
    wb_cyc_o, wb_adr_o, wb_stb_o, wb_we_o, wb_sel_o, wb_dat_o,

    wb_cti_o, wb_bte_o,

    cbu_dat_i, cbu_adr_i, cbu_rqt_i, cbu_we_i, cbu_sel_i,
    cbu_ack_o, cbu_dat_o, cbu_err_o
);

parameter dw = 32;
parameter aw = 5;

//
// core clock, reset and clock control
//
input				clk;		// core clock
input				rst;		// core reset
input [1:0] 		clmode;		// clock ratio of core:bus: 00-1:1, 01-2:1, 10-not support, 11-4:1

//
// core-bridge unit interface
//
input [dw-1:0] 		cbu_dat_i;	// input data bus
input [aw-1:0] 		cbu_adr_i;	// address bus
input				cbu_rqt_i;	// WB cycle
input				cbu_we_i;	// WB write enable
input [3:0] 		cbu_sel_i;	// byte selects

output [31:0] 		cbu_dat_o;	// output data bus
output				cbu_ack_o;	// ack output
output				cbu_err_o;	// err output

//
// WISHBONE interface
//
input				wb_clk_i;	// clock input
input				wb_rst_i;	// reset input
input				wb_ack_i;	// normal termination
input				wb_err_i;	// termination w/ error
input				wb_rty_i;	// termination w/ retry
input [dw-1:0] 		wb_dat_i;	// input data bus
output				wb_cyc_o;	// cycle valid output
output [aw-1:0] 	wb_adr_o;	// address bus outputs
output				wb_stb_o;	// strobe output
output				wb_we_o;	// indicates write transfer
output [3:0] 		wb_sel_o;	// byte select outputs
output [dw-1:0] 	wb_dat_o;	// output data bus
output [2:0] 	    wb_cti_o;	// cycle type identifier
output [1:0] 	    wb_bte_o;	// burst type extension


//
// Registers
//
wire 				wb_ack;		// normal termination
reg [aw-1:0] 		wb_adr_o;	// address bus outputs
reg 				wb_cyc_o;	// cycle output
reg 				wb_stb_o;	// strobe output
reg 				wb_we_o;	// indicates write transfer
reg [3:0] 			wb_sel_o;	// byte select outputs
wire [dw-1:0] 	    wb_dat_o;	// output data bus

reg [2:0] 		    wb_cti_o;	// cycle type identifier
reg [1:0] 		    wb_bte_o;	// burst type extension
reg [1:0] 		    burst_len;	// burst counter

reg  				cbu_stb_reg;	// WB strobe
wire  				cbu_stb;	    // WB strobe
reg 				wb_cyc_nxt;	    // next WB cycle value
reg 				wb_stb_nxt;	    // next WB strobe value
reg [2:0] 			wb_cti_nxt;	    // next cycle type identifier value

reg 				wb_ack_cnt;	    // WB ack toggle counter
reg 				wb_err_cnt;	    // WB err toggle counter
reg 				wb_rty_cnt;	    // WB rty toggle counter
reg 				cbu_ack_cnt;	// cbu ack toggle counter
reg 				cbu_err_cnt;	// cbu err toggle counter
reg 				cbu_rty_cnt;	// cbu rty toggle counter
wire 				cbu_rty;	    // cbu rty indicator

reg [1:0] 			wb_fsm_state_cur;	// WB FSM - surrent state
reg [1:0] 			wb_fsm_state_nxt;	// WB FSM - next state
wire [1:0] 			wb_fsm_idle	= 2'h0;	// WB FSM state - IDLE
wire [1:0] 			wb_fsm_trans= 2'h1;	// WB FSM state - normal TRANSFER
wire [1:0] 			wb_fsm_last	= 2'h2;	// EB FSM state - LAST transfer

//
// logic implementation
//

//
// WB burst length counter
// 
always @(posedge wb_clk_i or `pippo_RST_EVENT wb_rst_i) begin
    if (wb_rst_i == `pippo_RST_VALUE) begin
        burst_len <=  2'h0;
    end
    else begin
    if (wb_fsm_state_cur == wb_fsm_idle)
        burst_len <=  2'h2;
    else if (wb_stb_o & wb_ack)
        burst_len <=  burst_len - 1'b1;
    end
end

// WISHBONE I/F <-> Internal RISC I/F conversion
assign wb_ack = wb_ack_i & !wb_err_i & !wb_rty_i;

//
// WB FSM - register part
// 
always @(posedge wb_clk_i or `pippo_RST_EVENT wb_rst_i) begin
    if (wb_rst_i == `pippo_RST_VALUE) 
        wb_fsm_state_cur <=  wb_fsm_idle;
    else 
        wb_fsm_state_cur <=  wb_fsm_state_nxt;
end

// 
// WB FSM - combinatorial part
// 
always @(*) begin
    case(wb_fsm_state_cur)
	
    	// IDLE 
        wb_fsm_idle : begin
            wb_cyc_nxt = cbu_cyc_i & cbu_stb;
            wb_stb_nxt = cbu_cyc_i & cbu_stb;
            wb_cti_nxt = {!cbu_cab_i, 1'b1, !cbu_cab_i};
            if (cbu_cyc_i & cbu_stb)
                wb_fsm_state_nxt = wb_fsm_trans;
    	    else
    	        wb_fsm_state_nxt = wb_fsm_idle;
    	end
    	
    	// normal TRANSFER
        wb_fsm_trans : begin
            wb_cyc_nxt = !wb_stb_o | !wb_err_i & !wb_rty_i & 
                !(wb_ack & wb_cti_o == 3'b111);
    	   
    	    wb_stb_nxt = !wb_stb_o | !wb_err_i & !wb_rty_i & !wb_ack | 
    			!wb_err_i & !wb_rty_i & wb_cti_o == 3'b010;
    	   
    	    wb_cti_nxt[2] = wb_stb_o & wb_ack & burst_len == 'h0 | wb_cti_o[2];
    	    wb_cti_nxt[1] = 1'b1  ;
    	    wb_cti_nxt[0] = wb_stb_o & wb_ack & burst_len == 'h0 | wb_cti_o[0];
    	   
    	    if ((!cbu_cyc_i | !cbu_stb | !cbu_cab_i | cbu_sel_i != wb_sel_o | cbu_we_i != wb_we_o) & wb_cti_o == 3'b010)
    	        wb_fsm_state_nxt = wb_fsm_last;
    	    else if ((wb_err_i | wb_rty_i | wb_ack & wb_cti_o==3'b111) & wb_stb_o)
    	        wb_fsm_state_nxt = wb_fsm_idle;
    	    else
    	        wb_fsm_state_nxt = wb_fsm_trans;
    	end
	
    	// LAST transfer
    	wb_fsm_last : begin
    	    wb_cyc_nxt = !wb_stb_o | !wb_err_i & !wb_rty_i & 
                !(wb_ack & wb_cti_o == 3'b111);
    	    wb_stb_nxt = !wb_stb_o | !wb_err_i & !wb_rty_i & 
    			!(wb_ack & wb_cti_o == 3'b111);
    	    wb_cti_nxt[2] = wb_ack & wb_stb_o | wb_cti_o[2];
    	    wb_cti_nxt[1] = 1'b1;
    	    wb_cti_nxt[0] = wb_ack & wb_stb_o | wb_cti_o[0];
    	    if ((wb_err_i | wb_rty_i | wb_ack & wb_cti_o == 3'b111) & wb_stb_o)
    	        wb_fsm_state_nxt = wb_fsm_idle;
    	    else
    	        wb_fsm_state_nxt = wb_fsm_last;
        end
	
        default:begin
           wb_cyc_nxt = 1'bx;
           wb_stb_nxt = 1'bx;
           wb_cti_nxt = 3'bxxx;
           wb_fsm_state_nxt = 2'bxx;
        end
    
    endcase
end

//
// wishbone request
// 
always @(posedge wb_clk_i or `pippo_RST_EVENT wb_rst_i) begin
    if (wb_rst_i == `pippo_RST_VALUE) begin
	    wb_cyc_o	<=  1'b0;
	    wb_stb_o	<=  1'b0;
	    wb_cti_o	<=  3'b111;
	    wb_bte_o	<=  2'b01;	// 4-beat wrap burst = constant
	    wb_we_o		<=  1'b0;
	    wb_sel_o	<=  4'hf;
	    wb_adr_o	<=  {aw{1'b0}};
    end
    else begin
	    wb_cyc_o <= wb_cyc_nxt;
            if (wb_ack & wb_cti_o == 3'b111) 
                wb_stb_o <=  1'b0;
            else
                wb_stb_o <=  wb_stb_nxt;
	    wb_cti_o <= wb_cti_nxt;
	    wb_bte_o <= 2'b01;	
	    
	 // we and sel - set at beginning of access 
	    if (wb_fsm_state_cur == wb_fsm_idle) begin
	        wb_we_o	<= cbu_we_i;
	        wb_sel_o <= cbu_sel_i;
	    end
	 // adr - set at beginning of access and changed at every beat termination 
	    if (wb_fsm_state_cur == wb_fsm_idle) begin
	        wb_adr_o <= cbu_adr_i;
	    end 
	    else if (wb_stb_o & wb_ack) begin
	        wb_adr_o[3:2] <= wb_adr_o[3:2] + 1'b1;
	    end
    end
end

assign wb_dat_o = cbu_dat_i; 

//
// WB & cbu termination toggle counters
// 
always @(posedge wb_clk_i or `pippo_RST_EVENT wb_rst_i) begin
    if (wb_rst_i == `pippo_RST_VALUE) begin
	    wb_ack_cnt	<=  1'b0;
	    wb_err_cnt	<=  1'b0;
	    wb_rty_cnt	<=  1'b0;
    end
    else begin
	    // WB ack toggle counter
	    if (wb_fsm_state_cur == wb_fsm_idle | !(|clmode))
	        wb_ack_cnt	<=  1'b0;
	    else if (wb_stb_o & wb_ack)
	        wb_ack_cnt	<=  !wb_ack_cnt;

	    // WB err toggle counter
	    if (wb_fsm_state_cur == wb_fsm_idle | !(|clmode))
	        wb_err_cnt	<=  1'b0;
	    else if (wb_stb_o & wb_err_i)
	        wb_err_cnt	<=  !wb_err_cnt;

	    // WB rty toggle counter
	    if (wb_fsm_state_cur == wb_fsm_idle | !(|clmode))
	        wb_rty_cnt	<=  1'b0;
	    else if (wb_stb_o & wb_rty_i)
	        wb_rty_cnt	<=  !wb_rty_cnt;
    end
end

always @(posedge clk or `pippo_RST_EVENT rst) begin
    if (rst == `pippo_RST_VALUE) begin
        cbu_stb_reg	<=  1'b0;
    end
    else begin
	    if (cbu_stb_i & !cbu_cab_i & cbu_ack_o)
	        cbu_stb_reg	<=  1'b0;
	    else
	        cbu_stb_reg	<=  cbu_stb_i;
    end
end

assign cbu_stb = cbu_stb_i & cbu_stb_reg;

always @(posedge clk or `pippo_RST_EVENT rst) begin
    if (rst == `pippo_RST_VALUE) begin
	    cbu_ack_cnt	<=  1'b0;
	    cbu_err_cnt	<=  1'b0;
	    cbu_rty_cnt	<=  1'b0;
    end
    else begin
	    // cbu ack toggle counter
	    if (wb_fsm_state_cur == wb_fsm_idle | !(|clmode))
	        cbu_ack_cnt	<=  1'b0 ;
	    else if (cbu_ack_o)
	        cbu_ack_cnt	<=  !cbu_ack_cnt ;

	    // cbu err toggle counter
	    if (wb_fsm_state_cur == wb_fsm_idle | !(|clmode))
	        cbu_err_cnt	<=  1'b0 ;
	    else if (wb_err_i & cbu_err_o)
	        cbu_err_cnt	<=  !cbu_err_cnt ;

	    // cbu rty toggle counter
	    if (wb_fsm_state_cur == wb_fsm_idle | !(|clmode))
	        cbu_rty_cnt	<=  1'b0 ;
	    else if (cbu_rty)
	        cbu_rty_cnt	<=  !cbu_rty_cnt ;
    end
end

//
// core response 
//
assign	cbu_dat_o	= wb_dat_i;
assign	cbu_rty		= (wb_fsm_state_cur == wb_fsm_trans) & wb_rty_i & wb_stb_o & (wb_rty_cnt ~^ cbu_rty_cnt);
assign	cbu_ack_o	= (wb_fsm_state_cur == wb_fsm_trans) & wb_ack & wb_stb_o & (wb_ack_cnt ~^ cbu_ack_cnt);
assign	cbu_err_o	= (wb_fsm_state_cur == wb_fsm_trans) & wb_err_i & wb_stb_o & (wb_err_cnt ~^ cbu_err_cnt);


endmodule
