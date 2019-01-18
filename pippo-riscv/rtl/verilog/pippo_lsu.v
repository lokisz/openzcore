/*
 * File:        pippo_lsu.v
 * Project:     pippo
 * Designer:    fang@ali
 * Mainteiner:  fang@ali
 * Checker:
 * Description:
 *      一，主要功能
                访存－数据对齐和拼接
                实现原子访存指令队
	            发出访存相关中断请求：align, dbuserr
        二，Big-endian implementation notes
        1，大小端是指数据字节（byte）或者位（bit）在单个字（word）中layout的方式和寻址规范；
                即：MSB/msb..LSB/lsb在字（word）中的排列和寻址
                大端格式如下：
                假设程序定义变量int i = 0x12345678                
                如果存储器字节地址按从左至右排列：
                    [2'b00][2'b01][2'b10][2'b11]
                    则字节数据分布为：[0x12][0x34][0x56][0x78]
                    注意：不同厂商可能存储器的字节排列顺序在视图上相反
                    [2'b11][2'b10][2'b01][2'b00]
                    则字节数据分布为：[0x78][0x56][0x34][0x12]                    
        2，大小端跟访存系统、总线和存储控制器，和CPU内部组织相关
                这里的CPU内部组织是指：Bit在CPU内部寄存器和数据通路的layout
                可统一假设CPU内部REG和数据通路排列为：左端为msb，右端为lsb
        3，对于CPU内部寄存器的域或位寻址，RISCV按照小端模式组织：
                从msb至lsb，bit地址由高到低的格式组织；
        4，寄存器域或位的寻址访问，和HDL描述风格相关；
            例如verilog中：无论bit按照小端描述[3:0]和大端描述[0:3]，0x1都为4'b0001；
        5，小端系统的数据地址和拼接规则如下
            1，	Byte访存，地址为2'b00
            程序和REG数据组织：	[31:8]-x; [7:0]-data
            MEM和总线数据组织：	[31:8]-x; [7:0]-x
            2，	Byte访存，地址为2'b01
            程序和REG数据组织：	[31:8]-x; [7:0]-data
            MEM和总线数据组织：	[31:16]-x; [15:8]-data; [7:0]-x
            3，	Byte访存，地址为2'b10
            程序和REG数据组织：	[31:8]-x; [7:0]-data
            MEM和总线数据组织：	[31:24]-x; [23:16]-data; [15:0]-x
            4，	Byte访存，地址为2'b11
            程序和REG数据组织：	[31:8]-x; [7:0]-data
            MEM和总线数据组织：	[31:24]-x; [23:0]-data
            5，	Halfword访存，地址为2'b00
            程序和REG数据组织：	[31:16]-x; [15:0]-data
            MEM和总线数据组织：	[31:16]-x; [15:0]-data
            6，	Halfword访存，地址为2'b10
            程序和REG数据组织：	[31:16]-x; [15:0]-data
            MEM和总线数据组织：	[31:16]-data; [15:0]-x
            7，	Word访存，地址为2'b00
            程序和REG数据组织：	[31:0]-data
            MEM和总线数据组织：	[31:0]-data
            其中，Haflword和word访存时若出现其他地址，则为非对齐访问。    
        三，注意事项
            1，中断的产生：目前pippo设计中，所有的非对齐访问都会产生中断，待参考手册
            2，地址生成和存储访问的多周期通过d-imx协议/multicycle实现；
                请求端lsu和响应端dmc之间为组合逻辑－可作为false/multi-cycle path
                d-imx: 通过imx协议的address和data phase实现多周期分布；
                multicycle/lsu_stall: 在core内部插入流水等待周期
 * Task.I:
 *      [TBD] 结合现在IMX协议处理
 *          mem2reg：等待ack有效才处理
 *          reg2mem：等待addr_ack有效才送出数据
 *          dmc部分是否需要返回地址的低位？
 *      [TBD] 地址生成和存储访问
 *          d-imx设计的注意事项：在rqt当拍进行地址译码能否保证时序要求
 *          修改d-imx协议，取消地址响应信号和回路，即去掉rty_i信号和相关逻辑
 *          流水化处理：插入流水寄存器；并支持连续的访存
 *      [TBD] 与流水线控制关系－去掉ID段multicycle控制访存指令执行时的冻结逻辑；改由lsu_stall控制冻结逻辑      
 *      [TBD] 在dmc模块如果加入write-buffer，对流水线影响－store完成的标志是写到wb即可, 如果在写回write-buffer内容发生错误，中断?
 *      [TBD]仅将load送至mem2reg，store送至reg2mem，能否提高性能或节省面积？
 *      [TBD]可否将数据拼接和对齐等功能移到imx中处理 
 */
 
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on
`include "def_pippo.v"

module pippo_lsu(

    clk, rst,
    
	lsu_uops, addrbase, addrofs, reg_zero, 
    lsu_datain, lsu_dataout, 
    lsu_stall, lsu_done,
    lsu_addr,
    
    set_atomic, clear_atomic,

    so, 
    cr0_lsu,
    cr0_lsu_we, 
    
    sig_align, sig_dbuserr,

	dimx_adr_o, dimx_rqt_o, dimx_we_o, dimx_sel_o, dimx_dat_o,
	dimx_dat_i, dimx_ack_i, dimx_err_i
);

parameter dw = `OPERAND_WIDTH;

//
// I/O
//
input       clk;
input       rst;

//
// Internal i/f
//
input	[31:0]		addrbase;
input	[31:0]		addrofs;
input	[dw-1:0]	lsu_datain;
input	[`LSUUOPS_WIDTH-1:0]	lsu_uops;

output	[dw-1:0]	lsu_dataout;
output  [31:0]      lsu_addr; 
output				lsu_stall;
output				lsu_done;
output				sig_align;
output				sig_dbuserr;

//
// External i/f to DC
//
output	[31:0]		dimx_adr_o;
output				dimx_rqt_o;
output				dimx_we_o;
output	[7:0]		dimx_sel_o;
output	[dw-1:0]		dimx_dat_o;
input	[dw-1:0]		dimx_dat_i;
input				dimx_ack_i;
input				dimx_err_i;

//
// Internal wires/regs
//
reg	[7:0]			dimx_sel_o;

wire [31:0]         lsu_addr; 
wire [3:0]          lsu_op; 

//
//
//
assign lsu_addr = addrbase + addrofs ;     // [TBD]to check address overflow?

//
// D-side IMX interface
//
// Note: the protocol is differnt than i-IMX
//  1, the request assert until the completion of data transaction
//  2, [TBD] eliminate address response(!rty_i)    
// Store Operation
//      cycle 1: 
//          master - send out rqt_valid(rqt_o)/addr_o/data_o/sel_o
//          slave - address decoding and if hit, give back addrack(!rty_i)
//                - slave can insert waiting cycles
//      cycle 2..:
//          slave - register data, and give dat_ack(ack_i), dictate transaction complete successfully
//          master - disassert rqt_valid
// Load Operation
//      cycle 1:
//          master - send out rqt_valid(rqt_o)/addr_o/sel_o
//          slave - address decoding and if hit, give back addrack(!rty_i)
//                - slave can insert waiting cycles
//      cycle 2..:
//          slave - send dat, and give dat_ack(ack_i), dictate transaction complete successfully
//          master - disassert rqt_valid, register dat_i, and diassert lsu_stall(advanced write-back)
//
assign dimx_rqt_o = (!dimx_ack_i) & (|lsu_op) & (!sig_align);
assign dimx_adr_o = lsu_addr; 

// (all store inst. 
assign dimx_we_o = (lsu_op[3];

// data selector for little-endian implementation: selector rule see specification
always @(lsu_op or dimx_adr_o)
	casex({lsu_op, dimx_adr_o[2:0]})
		{`LSUOP_SB, 3'b000} : dimx_sel_o = 8'b0000_0001;
		{`LSUOP_SB, 3'b001} : dimx_sel_o = 8'b0000_0010;
		{`LSUOP_SB, 3'b010} : dimx_sel_o = 8'b0000_0100;
		{`LSUOP_SB, 3'b011} : dimx_sel_o = 8'b0000_1000;
		{`LSUOP_SB, 3'b100} : dimx_sel_o = 8'b0001_0000;
		{`LSUOP_SB, 3'b101} : dimx_sel_o = 8'b0010_0000;
		{`LSUOP_SB, 3'b110} : dimx_sel_o = 8'b0100_0000;
		{`LSUOP_SB, 3'b111} : dimx_sel_o = 8'b1000_0000;
		{`LSUOP_SH, 3'b000} : dimx_sel_o = 8'b0000_0011;
		{`LSUOP_SH, 3'b010} : dimx_sel_o = 8'b0000_1100;
		{`LSUOP_SH, 3'b100} : dimx_sel_o = 8'b0011_0000;
		{`LSUOP_SH, 3'b110} : dimx_sel_o = 8'b1100_0000;
		{`LSUOP_SW, 3'b000} : dimx_sel_o = 8'b0000_1111;
		{`LSUOP_SW, 3'b100} : dimx_sel_o = 8'b1111_0000;
        {`LSUOP_SD, 3'b000} : dimx_sel_o = 8'b1111_1111;

		{`LSUOP_LB, 3'b000},
		{`LSUOP_LBU, 3'b000}: dimx_sel_o = 8'b0000_0001;
		{`LSUOP_LB, 3'b001},
		{`LSUOP_LBU, 3'b001}: dimx_sel_o = 8'b0000_0010;
		{`LSUOP_LB, 3'b010},
		{`LSUOP_LBU, 3'b010}: dimx_sel_o = 8'b0000_0100;
		{`LSUOP_LB, 3'b011},
		{`LSUOP_LBU, 3'b011}: dimx_sel_o = 8'b0000_1000;
		{`LSUOP_LB, 3'b100},
		{`LSUOP_LBU, 3'b100}: dimx_sel_o = 8'b0001_0000;
		{`LSUOP_LB, 3'b101},
		{`LSUOP_LBU, 3'b101}: dimx_sel_o = 8'b0010_0000;
		{`LSUOP_LB, 3'b110},
		{`LSUOP_LBU, 3'b110}: dimx_sel_o = 8'b0100_0000;
		{`LSUOP_LB, 3'b111},
		{`LSUOP_LBU, 3'b111}: dimx_sel_o = 8'b1000_0000;
		
		{`LSUOP_LH, 3'b000},
		{`LSUOP_LHU, 3'b000}: dimx_sel_o = 8'b0000_0011;
		{`LSUOP_LH, 3'b010},
		{`LSUOP_LHU, 3'b010}: dimx_sel_o = 8'b0000_1100;
		{`LSUOP_LH, 3'b100},
		{`LSUOP_LHU, 3'b100}: dimx_sel_o = 8'b0011_0000;
		{`LSUOP_LH, 3'b110},
		{`LSUOP_LHU, 3'b110}: dimx_sel_o = 8'b1100_0000;

		{`LSUOP_LW, 3'b000},
		{`LSUOP_LWU, 3'b000}: dimx_sel_o = 8'b0000_1111;
		{`LSUOP_LW, 3'b100},
		{`LSUOP_LWU, 3'b100}: dimx_sel_o = 8'b1111_0000;

		{`LSUOP_LD, 3'b000} : dimx_sel_o = 8'b1111_1111;

		default : dimx_sel_o = 8'b0000_0000;
	endcase

//
// Pipeline Control Signals
//

// lsu_stall assert until the completion of data transaction. 
assign lsu_stall = (|lsu_op) & !dimx_ack_i; 
assign lsu_done = (|lsu_op) & dimx_ack_i;     

//
// uops to op transfer
//
assign lsu_op = lsu_uops[3:0]; 

//
// memory-to-regfile aligner
//
lsu_mem2reg lsu_mem2reg(
	.addr(dimx_adr_o[2:0]),
	.lsu_op(lsu_op),
	.memdata(dimx_dat_i),
	.regdata(lsu_dataout)
);

//
// regfile-to-memory aligner
//
lsu_reg2mem lsu_reg2mem(
        .addr(dimx_adr_o[2:0]),
        .lsu_op(lsu_op),
        .regdata(lsu_datain),
        .memdata(dimx_dat_o)
);

//
// except request
//
assign sig_align = 
        ((lsu_op == `LSUOP_STH) | (lsu_op == `LSUOP_STHB) | (lsu_op == `LSUOP_LHZ) | 
            (lsu_op == `LSUOP_LHZB) | (lsu_op == `LSUOP_LHA)) & dimx_adr_o[0] | 
        ((lsu_op == `LSUOP_STW) | (lsu_op == `LSUOP_STWB) | (lsu_op == `LSUOP_LWZ) | 
            (lsu_op == `LSUOP_LWZB)) & |dimx_adr_o[1:0];
assign sig_dbuserr = dimx_ack_i & dimx_err_i;


endmodule

