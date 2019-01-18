/*
 * File:        pippo_bpu.v
 * Project:     pippo
 * Designer:    fang@ali
 * Mainteiner:  fang@ali
 * Checker:
 * Description:
 *      一，主要功能和逻辑
 *          a）判断分支条件－实现比较逻辑
 *          b）计算分支目标地址           
 *          c）jump with link的判断在id段完成，并由rfwb_uops处理更新逻辑
 *      二，分支处理时序
 *          Cycle 1：对应分支指令的EXE段，如果分支不跳转，一切正常；如果分支跳转，则
 *              送出npc_branch和有效信号至PC；
 *              置flush_branch为有效，刷新流水线
 *          Cycle 2：IF/ID/EXE完成刷新    
 *              PC将新的取指请求送到指令总线接口
 * Task.I
 *      [TBO]逻辑优化-比较处理复用alu逻辑，返回判断结果即可；地址生成复用LSU的地址加法器；
 *      [TBV]分支目标地址生成：偏移为负数情况，溢出的处理-软件保证不会生成？
 */
 
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_bpu(

    clk, rst, 
    
    bpu_uops, 
    
    branch_immoffset, bus_a, bus_b,
    
    cia, snia, 
    
    npc_branch_valid, npc_branch, flush_branch
);

//
// Internal i/f
//

input           clk;
input           rst;

//uops and operands
input   [4:0]   bpu_uops;

// for target address generation and LR update
input   [31:0]  branch_immoffset; 
input   [31:0]  bus_a, bus_b;
input   [31:0]  cia; 
input   [31:0]  snia; 

// flush pipeline when branch is taken, at current implementation
output          flush_branch; 
output          npc_branch_valid;
output  [31:0]  npc_branch; 

//
// Internal wires/regs
//
reg     [31:0]  branch_target;
reg             condition_pass;

//
// logic
//

// bpu_uops[`BPUOP_SCMP_BIT] assertion(1'b1) means signed compare, diassertion(1'b0) means unsigned compare
assign cmp_a = {bus_a[width-1] ^ bpu_uops[`BPUOP_SCMP_BIT], bus_a[width-2:0]};
assign cmp_b = {bus_b[width-1] ^ bpu_uops[`BPUOP_SCMP_BIT], bus_b[width-2:0]};


always @(bpu_uops or bus_a or bus_b or cmp_a or cmp_b or cia) begin
    condition_pass = 1'b0;
    branch_target = 32'd0; 
    case (bpu_uops[2:0])
        
        `BPUOP_NOP: branch_target = 32'd0; 
        
        `BPUOP_REGIMM: begin
            branch_target = branch_immoffset + bus_a; 
        end

        `BPUOP_PCIMM: begin
            branch_target = branch_immoffset + cia; 
        end

        `BPUOP_CBEQ: begin
            condition_pass = (bus_a == bus_b); 
            branch_target = branch_immoffset + cia;         
        end
        
        `BPUOP_CBNE: begin
            condition_pass = (bus_a !== bus_b); 
            branch_target = branch_immoffset + cia;         
        end

        `BPUOP_CBLT: begin
            condition_pass = (cmp_a < cmp_b); 
            branch_target = branch_immoffset + cia;         
        end

        `BPUOP_CBGE: begin
            condition_pass = (cmp_a > cmp_b); 
            branch_target = branch_immoffset + cia;         
        end

        `BPUOP_CBLTU: begin
            condition_pass = (cmp_a < cmp_b); 
            branch_target = branch_immoffset + cia;         
        end

        `BPUOP_CGEU: begin
            condition_pass = (cmp_a > cmp_b); 
            branch_target = branch_immoffset + cia;         
        end

    endcase        
end

//
// output
//
// unconditional branch or taken conditional branch
assign flush_branch = (bpu_uops[`BPUOP_JUMP_BIT] | ( !bpu_uops[`BPUOP_JUMP_BIT] & condition_pass & |bpu_uops);   

// Output to pipeline control and fetch unit
assign npc_branch_valid = flush_branch; 
assign npc_branch = branch_target;

endmodule


