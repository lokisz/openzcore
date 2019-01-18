/*
 * File:        pippo_id.v
 * Project:     pippo
 * Designer:    fang@ali
 * Mainteiner:  fang@ali
 * Checker:
 * Assigner:    
 * Description:
 *  一 ID段主要操作（逻辑结构）包括：
 *      1，指令识别和译码：根据id_inst的OPCD, OP_F3和OP_F7域识别指令类型，生成相应的uops，并产生操作数选择的控制信号（如，
 *          目的和源操作数位置选择，立即数拓展类型选择）
 *      2，根据译码（操作数地址和使能信号），访问RF（GPRs，SPRs，MSR）获取操作数
 *          GPRs和部分SPRs在ID段发出请求，EXE段可用
 *      3，根据指令和上下文，生成位于EXE段前端的operandmux控制信号，选择操作数来源-reg/imm/fwd
 *      4，Hazard检测和相应后端流水线控制－插入Bubble和转发等，以及特殊指令处理
 *          a)	数据冲突，比较id_inst与后端流水操作数设置wbmux转发。对于四级流水设计，仅需支持wb数据到ALU数据端转发。
 *          b)	识别非流水的多周期指令，根据流水线设计在适当时候发出相应请求－进行多周期计数。
 *  二 注意以下数类处理器管理指令的处理：在ID段仅生成id_sig_xxx。
 *      1，显性同步指令：fence, fence.i：
 *      2，系统管理指令：scall, sbreak, 发送至except模块处理
 *  三 处理器模式检查（Supervisor/Privilleged和User/Problem State）
 *  四 同步指令处理
 *      同步涉及两方面的处理
 *          a）流水线和流水段中buffer内容的管理；
 *          b）un-core的管理，如cache, mmu和外设等。
 *  五 Jump with Link指令的处理
 *      Jump处理逻辑位于bpu，link逻辑处理在id段解决-译码出相应的寄存器写回uops；
 * Task:
 *      [TBD]同步指令处理
 *          同步指令在id段给出sig_x；
 *          隐性同步指令在ex/wb段，在完成写回的同时，送出ex_sig_x完成指令流变换实现
 *              如果隐性同步指令在ex/wb阶段引发中断行为－例如supervisor状态检测；按照正常中断产生处理；
 *          同步指令的处理在except模块或sprs模块执行？
 *      [TBV]转发逻辑
 * 
 */

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_id(

	clk, rst,

	id_inst, id_cia, id_snia, id_valid, 
	ex_inst, ex_cia, ex_snia, ex_valid,
	
	gpr_addr_rda, gpr_addr_rdb, gpr_addr_rdc, gpr_rda_en, gpr_rdb_en, gpr_rdc_en, 
	ex_imm, ex_sel_a, ex_sel_b, 
	
	ex_branch_addrofs, reg_zero, ex_spr_addr, 
	
    set_atomic, clear_atomic,

	ex_bpu_uops, ex_alu_uops, ex_cr_uops, ex_lsu_uops, ex_reg_uops, 
	
	ex_rfwb_uops, ex_gpr_addr_wr,
	
	multicycle_cnt, id_freeze, ex_freeze, wb_freeze, flushpipe, 
		
	sig_syscall, sig_rfi, sig_rfci, sig_eieio, sig_isync, sig_sync, sig_illegal, sig_emulate, 
	id_sig_ibuserr, sig_ibuserr
);

input					clk;
input					rst;

// pipeling registers
input					id_valid;           
input	[31:0]			id_inst;            
input	[29:0]			id_cia;             
input	[29:0]			id_snia;             

output					ex_valid;      
output	[31:0]			ex_inst;            
output	[29:0]			ex_cia;             
output	[29:0]			ex_snia;             

// operand signals: gpr read port - assert at current stage
output	[`GPR_ADDR_WIDTH-1:0]	gpr_rda_addr;
output	[`GPR_ADDR_WIDTH-1:0]	gpr_rdb_addr;
output					    gpr_rda_en;
output					    gpr_rdb_en;
output					    gpr_rdc_en;
// operand signals: imm and operandmux control - need pipeling, assert at following stage
output	[31:0]			    ex_imm;                       
output [`OPSEL_WIDTH-1:0]   ex_sel_a;
output [`OPSEL_WIDTH-1:0]   ex_sel_b;
// operand signals: address displacement or address - need pipeling, assert at following stage
output	[31:0]			    ex_branch_addrofs;
output                      reg_zero;             
output	[`SPR_ADDR_WIDTH-1:0]	ex_spr_addr;  

// uops signals, need pipeling, assert at following stages
output	[`BPUUOPS_WIDTH-1:0]	ex_bpu_uops;  
output	[`ALUUOPS_WIDTH-1:0]	ex_alu_uops;
output	[`LSUUOPS_WIDTH-1:0]	ex_lsu_uops;     
output	[`REGUOPS_WIDTH-1:0]	ex_reg_uops;     

// wb signals, need pipeling
//      note: write-back enable signals are encoded in rfwb_uops
output	[`RFWBUOPS_WIDTH-1:0]	ex_rfwb_uops;       
output	[`GPR_ADDR_WIDTH-1:0]	ex_gpr_wr_addr;

// pipeling control signals
input					id_freeze;          
input					ex_freeze;          
input					wb_freeze;          
input					flushpipe;          

// atomic memory access for lsu
output                  set_atomic;
output                  clear_atomic;

// multicycle instruction counter, assert at following stages
output	[`MULTICYCLE_WIDTH-1:0]	multicycle_cnt;

// exception request signals
output					sig_scall;
output					sig_break;
output					sig_illegal;
// synchronization signals
output					sig_fence;
output                  sig_fencei;
// exception requests pipeling from IF stage
input                   id_sig_ibuserr;
output                  sig_ibuserr;
    
//
// Whole decoder
//
reg	[`BPUUOPS_WIDTH-1:0]	id_bpu_uops;  
reg	[`ALUUOPS_WIDTH-1:0]	id_alu_uops;
reg	[`LSUUOPS_WIDTH-1:0]	id_lsu_uops;     
reg	[`REGUOPS_WIDTH-1:0]	id_reg_uops;     

reg [`GPR_ADDR_WIDTH-1:0]	gpr_rda_addr;
reg	[`GPR_ADDR_WIDTH-1:0]	gpr_rdb_addr;
reg					        gpr_rda_en;
reg					        gpr_rdb_en;

reg	[`RFWBUOPS_WIDTH-1:0]	id_rfwb_uops;  
reg	[`GPR_ADDR_WIDTH-1:0]	id_gpr_wr_addr;

reg	[31:0]			id_branch_addrofs;
reg                 id_reg_zero; 
reg                 reg_zero; 
reg	[`SPR_ADDR_WIDTH-1:0]	id_spr_addr;
reg	[`OPERAND_WIDTH-1:0]    id_imm;   
reg                 sel_imm; 

reg                 id_set_atomic; 
reg                 set_atomic; 
reg                 id_clear_atomic; 
reg                 clear_atomic; 

reg [`MULTICYCLE_WIDTH-1:0] multicycle;

reg id_sig_illegal;
reg id_sig_scall;
reg id_sig_sbreak;
reg id_sig_fence;
reg id_sig_fencei;
    
// There are six types instruction code in RISC-V:
//      R-type:     {funct7, rs2, rs1, funct3, rd, opcode}
//      I-type:     {imm[11:0], rs1, funct3, rd, opcode}
//      S-type:     {imm[11:5], rs2, rs1, funct3, imm[4:0], opcode}
//      SB_type:    {imm[12], imm[10:5], rs2, rs1, funct3, imm[4:1], imm[11], opcode}
//      U-type:     {imm[31:12], rd, opcode}
//      UJ-type:    {imm[20], imm[10:1], imm[11], imm[19:12], rd, opcode}
always @(id_inst or id_cia or id_snia or id_valid) begin
    // EX/WB uops
    id_bpu_uops = `BPUOP_NOP; 
    id_alu_uops = `ALUOP_NOP; 
    id_lsu_uops = `LSUOP_NOP; 
    id_reg_uops = `REGOP_NOP;    
    // gprs access
    gpr_addr_rda = id_inst[`OPERAND_RS1_BITADDR];
    gpr_addr_rdb = id_inst[`OPERAND_RS2_BITADDR];
    gpr_rda_en = 1'b0; 
    gpr_rdb_en = 1'b0;    
    // wb
    id_rfwb_uops = `RFWBOP_NOP;
    id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];       
    // imm
    sel_imm = 1'b0; 
    id_imm = 32'd0; 
    // address operands
    id_branch_addrofs = 32'd0; 
    id_reg_zero = 1'b0; 
    id_spr_addr = 12'd0; 
    // atomic memory access
    id_set_atomic = 1'b0; 
    id_clear_atomic = 1'b0; 
    // multicycle instruction indicator: to extend pipeline stages - for memory access and complex insts 
    multicycle = `EXTEND_ZERO_CYCLES;   
    // exception request
    id_sig_illegal = 1'b1;
    id_sig_sbreak = 1'b0; 
    id_sig_scall = 1'b0;
    // synchronization request
    id_sig_fence = 1'b0;
    id_sig_fencei = 1'b0; 
    case (id_inst[`OPCD_BITADDR])		// synopsys parallel_case

        //  opcode: LUI
        //  form: U-type
        //  execution: alu
        //  inst: LUI rd, imm
        //  flowchart: (imm) -> (rd)
        `LUI_OPCD: begin
            id_alu_uops = `ALUOP_BYPASS;    
            id_imm = `U_IMM;
            sel_imm = 1'b1;                                         
            id_rfwb_uops = `RFWBOP_ALU;
            id_gpr_addr_wr = id_inst[`OPERAND_RD_BITADDR];
            id_sig_illegal = 1'b0;
        
        end
        
        //  opcode: AUIPC
        //  form: U-type
        //  execution: bpu
        //  inst: AUIPC rd, imm
        //  flowchart: add(pc, imm) -> (pc)
	    `AUIPC_OPCD: begin
            id_bpu_uops = `BPUOP_PCIMM; 
	        id_branch_addrofs = `U_IMM;
	        id_sig_illegal = 1'b0;
	    end

        //  opcode: JAL
        //  form: UJ-type
        //  execution: bpu
        //  inst: JAL rd, imm
        //  flowchart: 
        //      add(pc, imm) -> (pc)
        //              snia -> (rd)
        //  note: rd == x0 means jump withou link(J inst)
	    `JAL_OPCD: begin
            id_bpu_uops = `BPUOP_PCIMM; 
	        id_branch_addrofs = `UJ_IMM;
	        id_link_en = (id_inst[`OPERAND_RD_BITADDR] !== 5'b00000);       // syn tools should produce reduced OR
            id_rfwb_uops = `RFWBOP_BPU & {id_link_en, 3'b000};              // don't update link register
            id_gpr_addr_wr = id_inst[`OPERAND_RD_BITADDR];
	        id_sig_illegal = 1'b0;
	    end

        //  opcode: JALR
        //  form: I-type
        //  execution: bpu
        //  inst: JALR rd, rs1, imm
        //  flowchart: 
        //      add(rs1, imm) -> (pc)
        //               snia -> (rd)
        //  note: rd == x0 means jump withou link(J inst)
	    `JALR_OPCD: begin
            id_bpu_uops = `BPUOP_REGIMM; 
	        id_branch_addrofs = `I_IMM;
	        id_link_en = (id_inst[`OPERAND_RD_BITADDR] !== 5'b00000);
            id_rfwb_uops = `RFWBOP_BPU & {id_link_en, 3'b000};
            id_gpr_addr_wr = id_inst[`OPERAND_RD_BITADDR];
	        id_sig_illegal = 1'b0;
	    end

        //  opcode: BRANCH
        //  form: SB-type
        //  execution: bpu
        //  inst: BEQ rs1, rs2, imm
        //  inst: BNE rs1, rs2, imm
        //  inst: BLT rs1, rs2, imm
        //  inst: BGE rs1, rs2, imm
        //  inst: BLTU rs1, rs2, imm
        //  inst: BGEU rs1, rs2, imm
        //  flowchart:
        //      cmp(rs1, rs2) -> (c)
        //       add(pc, imm) -> (c)(pc)
        `BRANCH_OPCD: begin
             case (id_inst[`OPCD_F3_BITADDR]) // synopsys parallel_case
    
                    `BEQ_FUNC3_OPCD: begin
                        id_bpu_uops = `BPUOP_CBEQ; 
            	        id_branch_addrofs = {19{id_inst[31]}, id_inst[7], id_inst[30:25], id_inst[11:8], 1'b0};     // non standar SB_IMM	 
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_sig_illegal = 1'b0;
                    end            

                    `BNE_FUNC3_OPCD: begin
                        id_bpu_uops = `BPUOP_CBNE;
            	        id_branch_addrofs = {19{id_inst[31]}, id_inst[7], id_inst[30:25], id_inst[11:8], 1'b0};     // non standar SB_IMM	 
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_sig_illegal = 1'b0;
                    end            

                    `BLT_FUNC3_OPCD: begin
                        id_bpu_uops = `BPUOP_CBLT; 
            	        id_branch_addrofs = {19{id_inst[31]}, id_inst[7], id_inst[30:25], id_inst[11:8], 1'b0};     // non standar SB_IMM	 
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_sig_illegal = 1'b0;
                    end            
 
                     `BGE_FUNC3_OPCD: begin
                        id_bpu_uops = `BPUOP_CBGE;
            	        id_branch_addrofs = {19{id_inst[31]}, id_inst[7], id_inst[30:25], id_inst[11:8], 1'b0};     // non standar SB_IMM	 
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_sig_illegal = 1'b0;
                    end            

                    `BLTU_FUNC3_OPCD: begin
                        id_bpu_uops = `BPUOP_CBLTU;
            	        id_branch_addrofs = {19{id_inst[31]}, id_inst[7], id_inst[30:25], id_inst[11:8], 1'b0};     // non standar SB_IMM	 
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_sig_illegal = 1'b0;
                    end            

                    `BGEU_FUNC3_OPCD: begin
                        id_bpu_uops = `BPUOP_CBGEU;
            	        id_branch_addrofs = {19{id_inst[31]}, id_inst[7], id_inst[30:25], id_inst[11:8], 1'b0};     // non standar SB_IMM	 
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_sig_illegal = 1'b0;
                    end
            endcase
       end

        //  opcode: LOAD
        //  form: I-type
        //  execution: lsu
        //  inst: LB rd, rs1, imm
        //  inst: LH rd, rs1, imm
        //  inst: LW rd, rs1, imm
        //  inst: LBU rd, rs1, imm
        //  inst: LHU rd, rs1, imm
        //  inst: LWU rd, rs1, imm  // RV64I
        //  inst: LD rd, rs1, imm   // RV64I
        //  flowchart:
        //      add(rs1, imm) -> (addr)
        //          mem[addr] -> (rd)
        `LOAD_OPCD: begin
             case (id_inst[`OPCD_F3_BITADDR]) // synopsys parallel_case
                `LB_FUNC3_OPCD: begin
                        id_lsu_uops = `LSUOP_LB;    
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_LSU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end
                  
                `LH_FUNC3_OPCD: begin
                        id_lsu_uops = `LSUOP_LH;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_LSU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end
                
                `LW_FUNC3_OPCD: begin
                        id_lsu_uops = `LSUOP_LW;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_LSU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end
                
                `LBU_FUNC3_OPCD: begin
                        id_lsu_uops = `LSUOP_LBU;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_LSU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end
                
                `LHU_FUNC3_OPCD: begin
                        id_lsu_uops = `LSUOP_LHU;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_LSU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                `LWU_FUNC3_OPCD: begin
                        id_lsu_uops = `LSUOP_LWU;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_LSU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                `LD_FUNC3_OPCD: begin
                        id_lsu_uops = `LSUOP_LD;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_LSU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end
            endcase
    	end		       	
 
        //  opcode: STORE
        //  form: S-type
        //  execution: lsu
        //  inst: SB rs1, rs2, imm
        //  inst: SH rs1, rs2, imm
        //  inst: SW rs1, rs2, imm
        //  inst: SD rs1, rs2, imm  // RV64I
        //  flowchart:
        //      add(rs1, imm) -> (addr)
        //              (rs2) -> mem[addr]
        `STORE_OPCD: begin
             case (id_inst[`OPCD_F3_BITADDR]) // synopsys parallel_case
                `SB_FUNC3_OPCD: begin
                        id_lsu_uops = `LSUOP_SB;    
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_sig_illegal = 1'b0;
                end
        
                `SH_FUNC3_OPCD: begin
                        id_lsu_uops = `LSUOP_SH;    
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_sig_illegal = 1'b0;
                end

                `SW_FUNC3_OPCD: begin
                        id_lsu_uops = `LSUOP_SW;    
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_sig_illegal = 1'b0;
                end

                `SD_FUNC3_OPCD: begin
                        id_lsu_uops = `LSUOP_SD;    
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_sig_illegal = 1'b0;
                end
            endcase
        end

        //  opcode: OPIMM        
        //  form: I-type
        //  execution: alu
        //  inst: ADDI rd, rs1, imm
        //  inst: SLTI rd, rs1, imm
        //  inst: SLTIU rd, rs1, imm
        //  inst: XORI rd, rs1, imm
        //  inst: ORI rd, rs1, imm
        //  inst: ANDI rd, rs1, imm
        //  inst: SLLI rd, rs1, shamt
        //  inst: SRLI rd, rs1, shamt
        //  inst: SRAI rd, rs1, shamt
        //  flowchart:
        //      ADDI:   add(rs1, imm) -> (rd)
        //      SLTI:   sign-cmp(rs1, imm) -> (rd)
        //      SLTIU:  unsign-cmp(rs1, imm) -> (rd)
        //      ANDI... opcd(rs1, imm) -> (rd)
        //      SxxI:   shift(rs1, imm[4:0]) -> (rd)
        `OPIMM_OPCD: begin    
             case (id_inst[`OPCD_F3_BITADDR]) // synopsys parallel_case
                `ADDI_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_ADD; 
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                `SLTI_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_SLTI; 
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end
                
                `SLTIU_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_SLTIU;  
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                `XORI_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_XORI;  
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                `ORI_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_ORI;  
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                 `ANDI_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_ANDI;  
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                `SLLI_FUNC3_OPCD: begin
                    case (`id_inst[`OPCD_F3_BITADDR])
                        `SLLI_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_SLLI;  
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            id_imm = `I_IMM; 
                            sel_imm = 1'b1;                                         
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end
                    endcase

                `SRLI_FUNC3_OPCD: begin
                    case (`id_inst[`OPCD_F3_BITADDR])
                        `SRLI_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_SRLI;  
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            id_imm = `I_IMM; 
                            sel_imm = 1'b1;                                         
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end

                        `SRAI_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_SRAI;  
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            id_imm = `I_IMM; 
                            sel_imm = 1'b1;                                         
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end
                    endcase
                end
           endcase
        end

        //  opcode: OPIMM32
        //  form: I-type
        //  execution: alu
        //  inst: ADDIW rd, rs1, shamt  // 32bit under RV64I
        //  inst: SLLIW rd, rs1, shamt  // 32bit under RV64I
        //  inst: SRLIW rd, rs1, shamt  // 32bit under RV64I
        //  inst: SRAIW rd, rs1, shamt  // 32bit under RV64I
        //  note: 32bit inst will generate illegal exception if imm[5] != 0;
        `OPIMM32_OPCD: begin    
             case (id_inst[`OPCD_F3_BITADDR]) // synopsys parallel_case
                `ADDIW_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_ADDW; 
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                `SLLIW_FUNC3_OPCD: begin
                        //`define SLLIW_FUNC7_OPCD        7'b000_0000
                        id_alu_uops = `ALUOP_SLLIW; 
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_imm = `I_IMM; 
                        sel_imm = 1'b1;                                         
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                `SRLIW_FUNC3_OPCD: begin
                    case (`id_inst[`OPCD_F7_BITADDR])
                        `SRLIW_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_SRLIW;  
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            id_imm = `I_IMM; 
                            sel_imm = 1'b1;                                         
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end

                        `SRAIW_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_SRAIW;  
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            id_imm = `I_IMM; 
                            sel_imm = 1'b1;                                         
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end
                    endcase
                end
           endcase
        end

        //  opcode: OP
        //  form: R-type
        //  execution: alu
        //  inst: ADD rd, rs1, rs2
        //  inst: SUB rd, rs1, rs2
        //  inst: SLL rd, rs1, rs2
        //  inst: SLT rd, rs1, rs2
        //  inst: SLTU rd, rs1, rs2
        //  inst: XOR rd, rs1, rs2
        //  inst: SRL rd, rs1, rs2
        //  inst: SRA rd, rs1, rs2
        //  inst: OR rd, rs1, rs2
        //  inst: AND rd, rs1, rs2
        //  flowchart: xxx(rs1, rs2) -> (rd)
        `OP_OPCD: begin
             case (id_inst[`OPCD_F3_BITADDR]) // synopsys parallel_case
                `ADD_FUNC3_OPCD: begin
                    case (`id_inst[`OPCD_F7_BITADDR])
                        `ADD_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_ADD;
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                            gpr_rdb_en = 1'b1; 
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end

                        `SUB_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_SUB;
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                            gpr_rdb_en = 1'b1; 
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end                        
                    endcase
                end

                `SLL_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_SLL;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                `SLT_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_SLT;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                `SLTU_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_SLTU;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                `XOR_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_XOR;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end
                                
                `SRL_FUNC3_OPCD: begin
                    case (`id_inst[`OPCD_F7_BITADDR])
                        `SRL_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_SRL;
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                            gpr_rdb_en = 1'b1; 
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end

                        `SRA_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_SRA;
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                            gpr_rdb_en = 1'b1; 
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end                        
                    endcase
                end

                `OR_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_OR;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                `AND_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_AND;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end
            endcase
        end
        
        //  opcode: OP32
        //  form: R-type
        //  execution: alu
        //  inst: ADDW rd, rs1, rs2
        //  inst: SUBW rd, rs1, rs2
        //  inst: SLLW rd, rs1, rs2
        //  inst: SRLW rd, rs1, rs2
        //  inst: SRAW rd, rs1, rs2
        //  flowchart: xxx(rs1, rs2) -> (rd)
        `OP32_OPCD: begin
             case (id_inst[`OPCD_F3_BITADDR]) // synopsys parallel_case
                `ADDW_FUNC3_OPCD: begin
                    case (`id_inst[`OPCD_F7_BITADDR])
                        `ADDW_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_ADDW;
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                            gpr_rdb_en = 1'b1; 
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end

                        `SUBW_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_SUBW;
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                            gpr_rdb_en = 1'b1; 
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end                        
                    endcase
                end

                `SLLW_FUNC3_OPCD: begin
                        id_alu_uops = `ALUOP_SLLW;
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                        gpr_rdb_en = 1'b1; 
                        id_rfwb_uops = `RFWBOP_ALU;
                        id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end
        
                `SRLW_FUNC3_OPCD: begin
                    case (`id_inst[`OPCD_F7_BITADDR])
                        `ADDW_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_SRLW;
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                            gpr_rdb_en = 1'b1; 
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end

                        `SRAW_FUNC7_OPCD: begin
                            id_alu_uops = `ALUOP_SRAW;
                            gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                            gpr_rda_en = 1'b1; 
                            gpr_rdb_addr = id_inst[`OPERAND_RS2_BITADDR];
                            gpr_rdb_en = 1'b1; 
                            id_rfwb_uops = `RFWBOP_ALU;
                            id_gpr_wr_addr = id_inst[`OPERAND_RD_BITADDR];
                            id_sig_illegal = 1'b0;
                        end                        
                    endcase
                end
            endcase
        end
        
        // I-type
        //  inst: FENCE/FENCE.I
        //  execution: lsu
        //  flowchart: flush buffers
        `MISCMEM_OPCD: begin
             case (id_inst[`OPCD_F3_BITADDR]) // synopsys parallel_case
                `FENCE_FUNC3_OPCD: begin
                    id_sig_fence = 1; 
                    id_sig_illegal = 1'b0;
                end

                `FENCEI_FUNC3_OPCD: begin
                    id_sig_fencei = 1; 
                    id_sig_illegal = 1'b0;
                end
            endcase
        end
        
        //  opcode:
        //  form: I-type
        //  execution: sprs/exception
        //  inst: SCALL
        //  inst: SBREAK        
        //  inst: SRET
        `SYSTEM_OPCD: begin
             case (id_inst[`OPCD_F3_BITADDR]) // synopsys parallel_case
                `SCALL_FUNC3_OPCD: begin
                    case (`id_inst[`OPCD_F7_BITADDR])
                        `SCALL_FUNC7_OPCD: begin
                            id_sig_scall = 1; 
                            id_sig_illegal = 1'b0;
                        end
                        
                        `SBREAK_FUNC7_OPCD: begin
                            id_sig_sbreak = 1; 
                            id_sig_illegal = 1'b0;
                        end

                        `SRET_FUNC7_OPCD: begin
                            id_sig_sret = 1; 
                            id_sig_illegal = 1'b0;
                        end
                    endcase
                end

                //  inst: CSRRW rd, csr, rs1
                //  flowchart: 
                //      un-ext(csr) -> rd
                //      rs1         -> csr
                //      when rd = x0, ignore csr read operation
                `CSRRW_FUNC3_OPCD: begin
                        id_reg_uops = `REGOP_CSRRW;
                        id_spr_addr = id_inst[`SPR_12BITADDR];
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1;
                        id_rfwb_uops = (id_inst[`OPERAND_RD_BITADDR] == 5'b00000) ? `RFWBOP_NOP : `RFWBOP_SPRS;
                        id_gpr_addr_wr = id_inst[`OPERAND_RD_BITADDR];
                        id_sig_illegal = 1'b0;
                end

                //  inst: CSRRS rd, csr, rs1
                //  flowchart: 
                //      un-ext(csr) -> rd
                //      11..111 -> csr[XLEN-1: bit-position(rs1)]
                //      when rs1 = x0, ignore csr write operation
                `CSRRS_FUNC3_OPCD: begin
                        id_reg_uops = (id_inst[`OPERAND_RS1_BITADDR] == 5'b00000) ? `REGOP_NOP : `REGOP_CSRRS;
                        id_spr_addr = id_inst[`SPR_12BITADDR];
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_rfwb_uops = (id_inst[`OPERAND_RD_BITADDR] == 5'b00000) ? `RFWBOP_NOP : `RFWBOP_SPRS;
                        id_gpr_addr_wr = id_inst[`OPERAND_RD_BITADDR]; 
                        id_sig_illegal = 1'b0;
                end

                //  inst: CSRRC rd, csr, rs1
                //  flowchart: 
                //      un-ext(csr) -> rd
                //      00..000 -> csr[XLEN-1: bit-position(rs1)]      
                `CSRRC_FUNC3_OPCD: begin
                        id_reg_uops = `REGOP_CSRRC; 
                        id_spr_addr = id_inst[`SPR_12BITADDR]; 
                        gpr_rda_addr = id_inst[`OPERAND_RS1_BITADDR];
                        gpr_rda_en = 1'b1; 
                        id_rfwb_uops = (id_inst[`OPERAND_RD_BITADDR] == 5'b00000) ? `RFWBOP_NOP : `RFWBOP_SPRS;
                        id_gpr_addr_wr = id_inst[`OPERAND_RD_BITADDR]; 
                        id_sig_illegal = 1'b0;
                end

                //  inst: CSRRWI/CSRRSI/CSRRCI
                //      using un-ext(zimm[4:0]) instead of rs1 value                
                //  flowchart: 
                //      un-ext(csr) -> rd
                //      un-ext(zimm)-> csr
                //      when rd = x0, ignore csr read operation
                `CSRRWI_FUNC3_OPCD: begin
                        id_reg_uops = `REGOP_CSRRW;
                        id_spr_addr = id_inst[`SPR_12BITADDR];
                        id_imm = `CSR_IMM;
                        sel_imm = 1'b1;
                        id_rfwb_uops = (id_inst[`OPERAND_RD_BITADDR] == 5'b00000) ? `RFWBOP_NOP : `RFWBOP_SPRS;
                        id_gpr_addr_wr = id_inst[`OPERAND_RD_BITADDR]; 
                        id_sig_illegal = 1'b0;
                end

                //  flowchart: 
                //      un-ext(csr) -> rd
                //      11..111 -> csr[XLEN-1: bit-position(zimm)]
                //      when rs1 = x0, ignore csr write operation
                `CSRRSI_FUNC3_OPCD: begin
                        id_reg_uops = `REGOP_CSRRS;
                        id_spr_addr = id_inst[`SPR_12BITADDR];
                        id_imm = `CSR_IMM;
                        sel_imm = 1'b1;
                        id_rfwb_uops = (id_inst[`OPERAND_RD_BITADDR] == 5'b00000) ? `RFWBOP_NOP : `RFWBOP_SPRS;
                        id_gpr_addr_wr = id_inst[`OPERAND_RD_BITADDR]; 
                        id_sig_illegal = 1'b0;
                end

                //  flowchart: 
                //      un-ext(csr) -> rd
                //      00..000 -> csr[XLEN-1: bit-position(zimm)]      
                `CSRRCI_FUNC3_OPCD: begin
                        id_reg_uops = `REGOP_CSRRC;
                        id_spr_addr = id_inst[`SPR_12BITADDR];
                        id_imm = `CSR_IMM;
                        sel_imm = 1'b1;
                        id_rfwb_uops = (id_inst[`OPERAND_RD_BITADDR] == 5'b00000) ? `RFWBOP_NOP : `RFWBOP_SPRS;
                        id_gpr_addr_wr = id_inst[`OPERAND_RD_BITADDR]; 
                        id_sig_illegal = 1'b0;
                end
            endcase
        end

    endcase // OPCODE field
end

//
// Forwarding logic
//

// write address pipeling
// [TBV] reset and flush logic
reg	[`GPR_ADDR_WIDTH-1:0]	ex_gpr_addr_wr;

always @(posedge clk or posedge rst) begin
	if (rst) begin
		ex_gpr_addr_wr <= #1 5'd0;
    end
	else if (!ex_freeze & id_freeze | flushpipe) begin
		ex_gpr_addr_wr <= #1 5'd0;
	end
	else if (!ex_freeze) begin
		ex_gpr_addr_wr <= #1 id_gpr_addr_wr;
	end
end

// operandmux control signals: sel_a/sel_b
reg [`OPSEL_WIDTH-1:0]  id_sel_a;
reg [`OPSEL_WIDTH-1:0]  id_sel_b;
always @(gpr_addr_rda or ex_gpr_addr_wra or ex_gpr_addr_wrb or ex_rfwb_uops) begin
	if ((gpr_addr_rda == ex_gpr_addr_wra) && ex_rfwb_uops[0])
		id_sel_a = `OPSEL_WBFWD;
	else if ((gpr_addr_rda == ex_gpr_addr_wrb) && ex_rfwb_uops[1])
		id_sel_a = `OPSEL_WBFWD;
	else
		id_sel_a = `OPSEL_RF;
end

always @(sel_imm or gpr_addr_rdb or ex_gpr_addr_wra or ex_gpr_addr_wrb or ex_rfwb_uops) begin
	if (sel_imm)
		id_sel_b = `OPSEL_IMM;
	else if ((gpr_addr_rdb == ex_gpr_addr_wra) && ex_rfwb_uops[0])
		id_sel_b = `OPSEL_WBFWD;
	else if ((gpr_addr_rdb == ex_gpr_addr_wrb) && ex_rfwb_uops[1])
		id_sel_b = `OPSEL_WBFWD;
	else
		id_sel_b = `OPSEL_RF;
end		


// [TBV] operandmux信号（sel_a/sel_b）的复位逻辑和冻结逻辑
reg [`OPSEL_WIDTH-1:0]  ex_sel_a;
reg [`OPSEL_WIDTH-1:0]  ex_sel_b;
always @(posedge clk or posedge rst) begin
	if (rst) begin
		ex_sel_a <= #1 `OPSEL_RF;
		ex_sel_b <= #1 `OPSEL_RF;
	end
	else if (!ex_freeze & id_freeze | flushpipe) begin
		ex_sel_a <= #1 `OPSEL_RF;
		ex_sel_b <= #1 `OPSEL_RF;
	end
	else if (!ex_freeze) begin
		ex_sel_a <= #1 id_sel_a;
		ex_sel_b <= #1 id_sel_b;
	end
end

//	
// Multicycle stall, send at EXE stage
//
reg	[`MULTICYCLE_WIDTH-1:0]	multicycle_cnt;

always @(posedge clk or posedge rst) begin
	if (rst)
		multicycle_cnt <= #1 2'b00;
	else if (|multicycle_cnt)
		multicycle_cnt <= #1 multicycle_cnt - 2'd1;
	else if (|multicycle & !ex_freeze)             
		multicycle_cnt <= #1 multicycle;
end

//
// ID/EX pipelining logic
//
   
// pipeling of uops
reg	[`BPUUOPS_WIDTH-1:0]	    ex_bpu_uops;  
reg	[`ALUUOPS_WIDTH-1:0]		ex_alu_uops;
reg	[`LSUUOPS_WIDTH-1:0]		ex_lsu_uops;
reg	[`RFWBUOPS_WIDTH-1:0]		ex_reg_uops;
reg	[`RFWBUOPS_WIDTH-1:0]		ex_cr_uops;

always @(posedge clk or posedge rst) begin
	if (rst) begin
		ex_bpu_uops <= #1 {2'b00, `BPUOP_NOP};      // {AA, LK, `BPUOP_NOP}
		ex_alu_uops <= #1 {2'b00, `ALUOP_NOP};      // {OE, Rc, `ALUOP_ADD}
		ex_lsu_uops <= #1 `LSUOP_NOP;
		ex_reg_uops <= #1 `REGOP_NOP;
		ex_cr_uops <= #1 `CROP_NOP;		
	end	
	else if (!ex_freeze & id_freeze | flushpipe) begin
		ex_bpu_uops <= #1 {2'b00, `BPUOP_NOP};      // {AA, LK, `BPUOP_NOP}
		ex_alu_uops <= #1 {2'b00, `ALUOP_NOP};     // {OE, Rc, `ALUOP_ADD}
		ex_lsu_uops <= #1 `LSUOP_NOP;
		ex_reg_uops <= #1 `REGOP_NOP;
		ex_cr_uops <= #1 `CROP_NOP;		
	end
	else if (!ex_freeze) begin
		ex_bpu_uops <= #1 id_bpu_uops;
		ex_alu_uops <= #1 id_alu_uops;
		ex_lsu_uops <= #1 id_lsu_uops;
		ex_reg_uops <= #1 id_reg_uops;
		ex_cr_uops <= #1 id_cr_uops;
	end
end

// RFWB_UPOS pipelining
reg	[`RFWBUOPS_WIDTH-1:0]		ex_rfwb_uops;
always @(posedge clk or posedge rst) begin
	if (rst) begin
		ex_rfwb_uops <= #1 `RFWBOP_NOP;   	
	end	
	else if (!ex_freeze & id_freeze | flushpipe) begin
		ex_rfwb_uops <= #1 `RFWBOP_NOP;   
	end
	else if (!ex_freeze) begin
		ex_rfwb_uops <= #1 id_rfwb_uops;
	end
end

// pipeling of operands
reg	[29:0]			ex_branch_addrofs;
//reg	[31:0]			ex_lsu_addrofs;
reg	[9:0]			ex_spr_addr;                  
reg	[31:0]			ex_imm;   

always @(posedge clk or posedge rst) begin
	if (rst) begin
	    ex_branch_addrofs <= #1 30'd0;
//		ex_lsu_addrofs <= #1 32'd0;
		reg_zero <= #1 1'b0; 
		ex_spr_addr <= #1 10'd0;
		ex_imm <= #1 32'd0;
        set_atomic <= 1'b0; 
        clear_atomic <= 1'b0;  
	end
	else if (!ex_freeze & id_freeze | flushpipe) begin
	    ex_branch_addrofs <= #1 30'd0;
//		ex_lsu_addrofs <= #1 32'd0;
		reg_zero <= #1 1'b0; 
		ex_spr_addr <= #1 10'd0;
		ex_imm <= #1 32'd0;
        set_atomic <= 1'b0; 
        clear_atomic <= 1'b0;  
	end
	else if (!ex_freeze) begin
	    ex_branch_addrofs <= #1 id_branch_addrofs;
//		ex_lsu_addrofs <= #1 id_lsu_addrofs;
		reg_zero <= #1 id_reg_zero; 
		ex_spr_addr <= #1 id_spr_addr;
		ex_imm <= #1 id_imm;
        set_atomic <= id_set_atomic; 
        clear_atomic <= id_clear_atomic;  
	end
end

// pipelining of exception requests
reg					sig_scall;
reg					sig_sbreak;
reg					sig_fence;
reg					sig_fencei;
reg					sig_illegal;
reg					sig_ibuserr;

always @(posedge clk or posedge rst) begin
	if (rst) begin
		sig_illegal <= #1 1'b0;
		sig_scall <= #1 1'b0;
		sig_sbreak <= #1 1'b0;
		sig_fence <= #1 1'b0;
		sig_fencei <= #1 1'b0;
		sig_ibuserr <= #1 1'b0;
	end
	else if (!ex_freeze & id_freeze | flushpipe) begin
		sig_illegal <= #1 1'b0;
		sig_scall <= #1 1'b0;
		sig_sbreak <= #1 1'b0;
		sig_fence <= #1 1'b0;
		sig_fencei <= #1 1'b0;
		sig_ibuserr <= #1 1'b0;
	end
	else if (!ex_freeze) begin
		sig_illegal <= #1 id_sig_illegal;
		sig_scall <= #1 id_sig_scall;
		sig_sbreak <= #1 id_sig_scall;
		sig_fence <= #1 id_sig_isync;
		sig_fencei <= #1 id_sig_sync;
		sig_ibuserr <= #1 id_sig_ibuserr;
	end
end

// Pipelining inst./CIA/NIA
//  [TBD] the coding style of pipeling logic: functional and performance verification
reg					ex_valid, ex_valid_value;      
reg	[31:0]			ex_inst, ex_inst_value;
reg	[29:0]			ex_cia, ex_cia_value;
reg	[29:0]			ex_snia, ex_snia_value;             

always @(id_freeze or ex_freeze or flushpipe or
        id_valid or id_inst or id_cia or id_snia or
        ex_valid or ex_inst or ex_cia or ex_snia) begin
	casex ({id_freeze, ex_freeze, flushpipe})	// synopsys parallel_case
		3'b000: begin       // Normal pipelining. 
            ex_valid_value = id_valid; 
            ex_inst_value = id_inst;
            ex_cia_value = id_cia;
            ex_snia_value = id_snia;
		end
		3'bxx1: begin       // flushpipe is asserted, insert NOP bubble
            ex_valid_value = 1'b0; 
            ex_inst_value = `pippo_PWR_NOP;
            ex_cia_value = id_cia;  
            ex_snia_value = id_snia; 
		end
		4'b100: begin       // id_freeze is asserted, ex_freeze is disasserted, insert NOP bubble
            ex_valid_value = 1'b0; 
            ex_inst_value = `pippo_PWR_NOP;
            ex_cia_value = id_cia; 
            ex_snia_value = id_snia; 
		end
		4'b110: begin       // id_freeze/ex_freeze is asserted, insert KCS bubble
            ex_valid_value = id_valid; 
            ex_inst_value = id_inst;
            ex_cia_value = id_cia;
            ex_snia_value = id_snia; 
		end
		default: begin      
            ex_valid_value = 1'b0;     
            ex_inst_value = `pippo_PWR_NOP;
            ex_cia_value = id_cia;
            ex_snia_value = id_snia; 
		end		
    endcase
end

always @(posedge clk or posedge rst) begin
    if(rst) begin 
        ex_valid <= #1 1'b0;
        ex_inst <= #1 `pippo_PWR_NOP;
        ex_cia <= #1 30'd0;
        ex_snia <= #1 30'd0; 
    end
    else begin
        ex_valid <= #1 ex_valid_value;
        ex_inst <= #1 ex_inst_value;
        ex_cia <= #1 ex_cia_value;
        ex_snia <= #1 ex_snia_value; 
    end
end

endmodule

