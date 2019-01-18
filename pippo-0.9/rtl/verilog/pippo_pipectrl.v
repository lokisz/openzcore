/*
 * File:        pippo_pipectrl.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description:
 *      流水线控制信号，主要产生
 *          pc_freeze：冻结PC寄存器，保存当前值
 *          if_freeze：表示IF段无法接受新操作
 *          id_freeze：表示ID段无法接受新操作
 *          ex_freeze：表示EXE段无法接受新操作
 *          wb_freeze：冻结写回操作，由于目前的三级流水设计，wb_freeze等同于ex_freeze
 *      以控制流水线寄存器（主要是xx_cia和xx_inst等）行为。
 * Specification
    1. 冻结流水线
        在流水线遇到以下情形，必须对流水线施加控制以保证指令流顺序执行并完成。
        （1）	某些指令类型需要增加流水级，如访存指令需增加MA流水级
        （2）	某流水段由于各种原因需要多个周期完成时，如复杂算术运算指令
        （3）	指令执行失败，如访存指令引发的中断，需要中止指令的完成
        （4）   以上多种情况同时发生（访存指令访问缓存但miss） 
        各流水阶段引发的stall请求如下：
            IF段：取回指令的等待周期，有if模块的if_stall请求
            ID段：无（译码检测到分支指令，有id模块的bp_stall请求）
            EX段：多周期执行操作（访存和复杂算术运算指令），有lsu_stall和multicycle_stall请求
        在综合评估流水线状态和stall请求之后，产生freeze信号控制流水线推进－正常流动，插入Bubble（NOP或KCS）等
        各流水段冻结（freeze）规则：当现阶段产生stall请求，前一级流水必须冻结，以避免overlapping
            当第一级流水(IF)出现stall请求时，IF段冻结
                [同一周期内IF miss I$，访存指令hit D$的情况]
        冻结信号应用规则如下：每一级的流水寄存器维护受前后两级流水线的freeze信号控制。
            当前端冻结后端正常时，插入NOP；
            当前端冻结后端冻结时，保持当前状态；
            当前端正常后端正常时，正常推进
            前端正常后端冻结的情况不符合freeze信号产生规则
    2. 刷新流水线
        除受freeze信号控制外，流水线在下面情况下需要刷新流水线－排空现有指令，重新取指
        （1）中断处理模块产生的flush_except以刷新流水线
        （2）分支处理模块由于分支预测错误产生的flush_branch以刷新流水线
    3. PC维护和更新
        PC维护和更新有两种情况需要分开处理
        （1）流水线冻结时，保持现有取指地址PC，待冻结解除后继续取指
        （2）流水线刷新时，需要放弃现有PC，待刷新完成后分别使用npc_xxx重新取指
        pc_freeze在从有效状态恢复时，
 * List2do: 
 *      [TBV] timing relationship between flushpipe and freeze.
 *
 */

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_pipectrl(

	flush_except, flush_branch, flushpipe, 

	if_stall, lsu_stall, multicycle_cnt, asyn_stall, 

	pc_freeze, if_freeze, id_freeze, ex_freeze, wb_freeze
);

//
// I/O
//

// stall request from pipeline stages
input				if_stall;                       
input				lsu_stall;      
input               asyn_stall;
input   [1:0]	    multicycle_cnt; 

// freeze command to pipeline stages
output				pc_freeze;
output				if_freeze;
output				id_freeze;
output				ex_freeze;
output				wb_freeze;

// flush control
input				flush_except;
input               flush_branch;
output              flushpipe; 

//
// Internal wires and regs
//
wire			multicycle_stall;

//
// flush pipelining 
//  at the last cycle of flushpipe assertion, new fetch address will send to pc register
//  1. when flush is raised by except processing, assert for two cycles.
//  2. when flush is raised by branch processing, assert for one cycle.
//
assign flushpipe = flush_except | flush_branch; 

//
// PC freeze signal
//
// pc_freeze is just for dis-asserting fetch request
assign pc_freeze = flushpipe;

//
// Pipeline freeze generation:
//

assign multicycle_stall = |multicycle_cnt;

// notes for asyn_stall: see except module
assign if_freeze = if_stall | id_freeze;
assign id_freeze = ex_freeze;
assign ex_freeze = wb_freeze | asyn_stall; 
assign wb_freeze = lsu_stall | multicycle_stall;

/*
Implementation: Memory access instructions with update, under 3r1w rf design

a. Load with update instructions
    pipectrl.v
        注意：multicycle_stall条件也须变化，当wb_twice有效时，下一条指令引起multicycle_stall不能置起
    rf.v
      we & (~wb_freeze | wb_at_fze)  //write enable logic  
    wb_twice come from rfwbop, and more to do:
        1. logic for write address and source
        2. write order of EA and loaded data?
        3. wb_atfze有效时间只能一拍    
b. Store with update instructions
    注意需要同步store的完成和update（写EA回RA）的完成

Logic.a: 可以写回时，冻结wb，写回一次，再解冻完成第二次写回；
  pipectrl.v
    //wb_atfze means: wb_freeze is asserted by wb_twice only, at this case, write-back can go.
    assign wb_freeze_a = lsu_stall | multicycle_stall;
    assign wb_freeze = wb_freeze_a | wb_atfze;

    always @(posedge clk or posedge rst) begin
    	if (rst)
    		wb_atfze <= #1 1'b0;
    	else if(wb_freeze_a)         
    		wb_atfze <= #1 rfwb_uops[0] & rfwb_uops[1];
        else
            wb_atfze <= #1 1'b0;
    end

  reg_gprs.v
    assign rf_we = (~flushpipe) & ((wea & ~wb_freeze) |(wea & wb_atfze) | (web & ~wb_freeze)) ;
    assign rf_addrw = wea & (!web | wb_atfze) ? addrwa : web ? addrwb : 5'd0;
    assign rf_dataw = wea & (!web | wb_atfze) ? datawa : web ? datawb : 32'd0;
  
  operandmux.v/wbmux.v
    如果使用转发逻辑，则wra和wrb的转发需要增加有效逻辑；计划实现于wbmux模块
        (gpr_addr_rda == ex_gpr_addr_wra) && (ex_rfwb_uops[0] & !ex_rfwb_uops[1])
        (gpr_addr_rda == ex_gpr_addr_wrb) && (ex_rfwb_uops[1] & !wb_atfze)
  
Logic.b: 先写回一次，再冻结wb段完成第二次写回；导致问题：
    1，下一条指令已经进入exe段，如果是多周期指令则会导致wb_atfze保持有效
    2，需要寄存addr_wr和ea等
assign wb_freeze_a = lsu_stall | multicycle_stall;
assign wb_freeze = wb_freeze_a | wb_atfze;

always @(posedge clk or posedge rst) begin
	if (rst)
		wb_atfze <= #1 1'b0;
	else if(!wb_freeze_a)         
		wb_atfze <= #1 rfwb_uops[0] & rfwb_uops[1];
end


*/

endmodule
