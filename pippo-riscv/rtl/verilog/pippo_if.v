/*
 * File:        pippo_if.v
 * Project:     pippo
 * Designer:    fang@ali
 * Mainteiner:  fang@ali
 * Checker: 
 * Assigner:
 * Description:
    一，IF段的主要功能如下
        1，	连接流水线和i-side IMX部件：发送取指地址以取回指令；将取回指令和指令地址送入流水线寄存器；
        2，	更新和维护PC值
        3，	处理后续流水线发出的跳转请求
        4，	如果IF段有exception发生，将相关请求信号送至后端流水等待except模块统一处理
    二，设计规范
    NPC的来源分类：
        1，	顺序取指，PC+4
        2，	分支跳转，npc_branch来自分支处理模块(根据EXE段执行结果从跳转目标地址或SNIA中选择)
        3，	中断处理，npc_except来自中断处理模块，为中断处理程序入口地址
        4， 同步指令执行（fence/fencei等隐性指令），npc_sync来自同步指令的SNIA；
    PC维护和更新：
        1， 正常情况－顺序指令流，使用NPC更新PC寄存器
        2， 顺序指令流水线冻结时，保持现有PC值，等待冻结解除后继续取指            
        3， 指令流变换
            1）中断处理：EXE段，中断处理开始（包括rfi/rfci），发出flush_except信号；等待npc_except有效后更新PC并取指
            2）分支指令处理－现采用a方案
                a）采用静态分支预测技术，所有分支指令都预测为NT。当在EXE段获取分支结果为Taken时，发出flush_branch信号，
                   然后等待npc_branch有效后更新PC并取指。当预测正确时，按照如上顺序指令流处理方法。
                b）ID段发现分支指令，发出bp_stall信号，此时下一条指令的取指尚未完成（锁存进入流水线）。
                   根据EXE段结果：
                        branch_taken    跳转。使用npc_branch更新pc并取指
                        branch_nt       不跳转。使用现pc取指 
            注：flushpipe信号为以上两种情况的综合，由pipectrl模块产生
    IMX接口处理－包含复杂的分布式协同
        取指请求的发送－根据处理器流水线状态和IMX返回的状态，同步更新取指请求和取指地址（PC）
            rqt_valid逻辑－取决于流水线状态
            PC更新逻辑－取决于流水线状态和IMX返回状态
        取指响应的接收－同样需要根据处理器流水线状态和IMX返回的状态，决定是否寄存当前响应进入流水线
            取指请求的取消和变换情况的处理
            有效返回并不一定会进入流水线
    三，IF/ID流水寄存器逻辑-流水输出包括id_valid, id_inst, id_cia, id_snia；流水维护和更新策略参考pipectrl模块，基本如下：
        1， 正常情况－取指成功，且流水线正常：寄存并送出取回指令
        2， 中断处理开始，except模块发出flushpipe信号以刷新流水线：插入NOP，并置id_valid为无效
        3， 当流水线冻结（if_freeze或id_freeze有效）时，IF/ID插入NOP或KCS bubble
            包括等待总线响应（ack_i/err_i无效，即if_stall），而其他正常时；
 * Task.I:
        [TBD]imx协议验证和改进
            目前i-imx采用分离的地址和数据tsc传输，并支持一级pipeling方式；
                目前rqt_valid总是有效－除流水线刷新(flushpipe)和冻结(id_freeze)时；
                在slave返回地址响应(!rty_i)后，PC更新，发出新的地址请求－间接保证不会重复送出之前的地址请求
                如果在地址响应和数据响应之间插入等待周期(即!rty_i和ack_i之间)，slave须保证工作正常－请求完成顺序
            减少imc返回的addr位宽，或者[1:0]也不需要；
            地址请求和响应（rqt->!rty_i）逻辑回路－模块之间都是组合逻辑（fetcher和biu/imc），且需注意线延时；
            复位情况的验证－slave必须同步，避免取回复位之前请求的数据；
            IMX下地址阶段和数据阶段同时结束的情况处理，即(!rty)和ack同时有效
            和wbv4规范的异同
                是否设置tsc_busy位表示总线传输进行中；
                待比较wb规范中!rty_i的时序问题－和ack_i的关系；
        [TBV]取指逻辑在取指冻结，流水线刷新，if_stall和当前取指所处阶段等各种情况结合下的处理和设计验证
            即PC的维护（包括NPC逻辑）和rqt_valid的同步更新必须保证
                1，发出正确的取指请求－根据当前流水线状态；
                2，仅寄存一次指令进入流水线；
                3，不会发生重复取指和不会丢失取回的指令；
        [TBD]当流水线返回取指错误（err_i）时，相应except请求在EXE段将得到处理；cia送入流水线，置id_valid为0；
            err_i取回的数据是什么？送入流水线会在id段产生其他行为？
            考虑插入NOP以避免错误数据的副作用，或用id_valid置所有id段输出为无效；
        [TBV] imx, pipelining and PC bahavior
            1. npc_branch, npc_except can't assert simultaneously
            2. after recover from freezing, first fetch request from previous pc/npc?
            3. the case of canceling fetch request(cache miss, and inst transfer event happened)
                processing by imx protocol: diassert rqt, then diassert ack.
            4. check id_snia timing
            5. coding style of pipelining register
        [TBD]make "itlb, immu" logic conditional
 * Task.II:
 *      full synthesis & verification
 *      rtl performance refactor-improve speed, reduce power and area.
 *      to improve fetch performance, add buffer to register fetched instruction and address at some case
 */

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_if(

	clk, rst,

	iimx_adr_o, iimx_rqt_o, iimx_rty_i, 
	iimx_ack_i, iimx_err_i, iimx_dat_i, iimx_adr_i, 

	npc_branch, npc_except, npc_branch_valid, npc_except_valid, 
	
	pc_freeze, if_stall, if_freeze, id_freeze, flushpipe, 
	
	id_inst, id_cia, id_valid, id_snia,
	
	id_sig_ibuserr
);

input				clk;
input				rst;

// IMX I/F to fetch instructions (from cache/ocm/memory controller)
input	[31:0]		iimx_dat_i;
input				iimx_ack_i;
input				iimx_err_i;
input	[31:0]		iimx_adr_i;
input				iimx_rty_i;
output	[31:0]		iimx_adr_o;
output				iimx_rqt_o;

// inst. flow transfer
input   [31:0]      npc_branch;         
input               npc_branch_valid;   
input   [31:0]      npc_except;         // keep least two address bits to code ISR entrance easily.
input               npc_except_valid;   

// pipeline control
input               pc_freeze;          
input				if_freeze;        
input				id_freeze;          
input				flushpipe;
output				if_stall;           

// pipeling register
output              id_valid;           
output	[31:0]		id_inst;            
output	[31:0]		id_cia;             
output	[31:0]		id_snia;            

// exception request
output				id_sig_ibuserr;     

//
// 
//
reg                 id_valid;
reg     [31:0]		id_inst;
reg     [31:0]		id_cia;
reg     [31:0]		id_snia;
reg                 id_sig_ibuserr;

reg     [31:0]      npc_value; 
reg                 if_valid;
reg     [31:0]      if_inst;          
reg     [31:0]      if_cia;
reg                 if_sig_ibuserr; 
reg	    [31:0]		pc;
wire                if_rqt_valid; 
wire    [32:0]      pcadd4;             
wire                except_ibuserr;

//
// send fetch rqt to I-IMX
//
assign iimx_adr_o = pc; 
assign iimx_rqt_o = if_rqt_valid; 

// stall request, waiting for I-IMX response 
assign if_stall = ~iimx_ack_i; 

//
// PC Register logic
//

// NPC source
assign pcadd4={(pc[31:2]+31'd1), 2'b00};

// 
assign fetch_lost = id_freeze & iimx_ack_i;

always @(npc_branch_valid or npc_except_valid or id_freeze or iimx_adr_i or
        fetch_lost or npc_branch or npc_except or pc or pcadd4) begin
	casex ({npc_branch_valid, npc_except_valid, id_freeze, fetch_lost})	// synopsys parallel_case
	    4'b0000: begin
	        npc_value = pcadd4[31:0];
	    end
		4'b0100: begin
			npc_value = npc_except;
		end
		4'b1000: begin
			npc_value = npc_branch;
		end
		4'b0010: begin                   // if pipeline(if_freeze & id_freeze) is freezon, keep current pc
			npc_value = pc;
		end		
		4'b0011: begin
		    npc_value = iimx_adr_i;     // fetch current fetched instruction again
        end
        
		default: begin     
			npc_value = pc;
		end		
    endcase
end

//
// PC update policy
//
//  1. after hard reset, keep RESET_VECTOR (pcadd4[32]) to avoid overflow
//  2. update pc when 
//      a) flushpipe: flush pipeline, fetch with npc
//      b) !iimx_rty_i: current transaction is complete
//      c) fetch lost
always @(posedge clk or posedge rst) begin
    if (rst)
        pc <= #1 `pippo_RESET_VECTOR;
    else if (pcadd4[32] & !flushpipe)   // !flushpipe: keep normal when fetched first branch inst
        pc <= #1 `pippo_RESET_VECTOR;
    else if (flushpipe | (!iimx_rty_i) | fetch_lost) 
        pc <= #1 npc_value;
end

//
// inst. fetch request logic: if_rqt_valid signal
//  1. when flushpipe assert(pc_freeze asserts), if_rqt_valid disassert
//  2. when pipeline freeze(id_freeze freeze), if_rqt_valid disassert to reduce memory access
//          NOT use if_freeze signal to exclude if_stall deadlock, or additional logic is needed for biu
//          deadloack: if_freeze assert at wait state, if you diassert this request, system stop forever.
//  3. hard-reset case: if_rqt_valid assert until RESET_VECTOR's ack(!iimx_rty_i) come, then keep disassertion.
//  At normal state, always send fetch request - if_rqt_valid keep assert, when address ack(!iimx_rty_i) assert
//     or disassert. including case that if_stall asserts, to avoid deadlock: no new fetch send out
//          deadlock: (if_stall raised -> if_freeze assert -> rqt_valid disassert forever)

// after hard reset, to avoid fetch reset vector many times, 
//      "!flushpipe" logic is to reduce one cycle delay after reset vector's branch
reg rst_rqt_done;
always @(posedge clk or posedge rst) begin
    if (rst)
        rst_rqt_done <= #1 1'b0;
    else if (flushpipe)
        rst_rqt_done <= #1 1'b0;    
    else if (pcadd4[32] & !iimx_rty_i)
        rst_rqt_done <= #1 1'b1;     
end

// to check the timing of rqt_valid signal - rqt_valid must have budget for addr_ack logic, i.e.:
// at IMX address phase(cycle 1):
//      Master send out if_rqt_valid, slave check address and give back addr_ack;
//      Addr_ack(!iimx_rty_i) signal must satisfy the setup time, when inputting to master;
// [TBD] register out if_rqt_valid signal to improve timing, add a pipeline bubble under some case
//      note: keep pace with pc update
assign if_rqt_valid = !(pc_freeze | id_freeze | (pcadd4[32] & rst_rqt_done));

//
// IF/ID pipelining logic
//
always @(iimx_ack_i or if_freeze or id_freeze or flushpipe or
        if_valid or if_inst or if_cia or 
        id_valid or id_inst or id_cia or 
        iimx_dat_i or iimx_adr_i or 
        except_ibuserr or id_sig_ibuserr) begin
	casex ({iimx_ack_i, if_freeze, id_freeze, flushpipe})	// synopsys parallel_case
		4'b1000: begin       // Normal pipelining. I-IMX returns valid value
            if_valid = 1'b1; 
            if_inst = iimx_dat_i;
            if_cia = iimx_adr_i;
            if_sig_ibuserr = except_ibuserr;
		end
		4'bxxx1: begin       // flushpipe is asserted, insert NOP bubble
            if_valid = 1'b0; 
            if_inst = `pippo_PWR_NOP;
            if_cia = id_cia;
            if_sig_ibuserr = 1'b0;
		end
		4'bx100: begin       // if_freeze is asserted, id_freeze is disasserted, insert NOP bubble
            if_valid = 1'b0; 
            if_inst = `pippo_PWR_NOP;
            if_cia = id_cia;
            if_sig_ibuserr = 1'b0;
		end
		4'bx110: begin       // if_freeze/id_freeze is asserted, insert KCS bubble
            if_valid = id_valid; 
            if_inst = id_inst;
            if_cia = id_cia;
            if_sig_ibuserr = id_sig_ibuserr;
		end
		default: begin      // [TBV]iimx_err_i is asserted
            if_valid = 1'b0;     
            if_inst = `pippo_PWR_NOP;
            if_cia = id_cia;
            if_sig_ibuserr = except_ibuserr;
		end		
    endcase
end

always @(posedge clk or posedge rst) begin
    if(rst) begin
        id_valid <= #1 1'b0;
        id_inst <= #1 `pippo_PWR_NOP;
        id_cia <= #1 32'd0;
        id_snia <= #1 32'd0; 
        id_sig_ibuserr = 1'b0;
    end
    else begin
        id_valid <= #1 if_valid;
        id_inst <= #1 if_inst;
        id_cia <= #1 if_cia;
        id_snia <= #1 pcadd4[31:0]; 
        id_sig_ibuserr = if_sig_ibuserr;
        `ifdef pippo_VERBOSE
        // synopsys translate_off
            $display("%t: id_valid <= %h", $time, id_valid);
            $display("%t: id_inst <= %h", $time, id_inst);
            $display("%t: id_cia <= %h", $time, id_cia);
            $display("%t: id_snia <= %h", $time, id_snia);
        // synopsys translate_on
        `endif
    end
end

//
// except request from IF stage
//
assign except_ibuserr = iimx_err_i & iimx_ack_i;    // err valid only when ack assert

endmodule


