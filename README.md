openzcore
=========

* Introduction

zCore is designed at PWRSemi(2009-2011), a startup try to develop clean-room PowerPC processor for consumer market. Now The unfinished product is open-sourced.

* File Directories
pippo-0.9: powerpc isa

    /asm            assembly example, show how to layout powerpc program, to run on pippo simulation or fpga prototyping

    /board          currently, only xilinx xupv2p board is supported
    
    /rtl            rtl logic implementation, pippo is the code name of first zCore product.
    
    /sim            modelsim simulation files, we will support icarus simulator later
    
    /soc            soc component files, not used currently, because the pippo cbu(core-bridge-unit) is un-verified
    
    /syn            for synthesis 
    
    /testbench      testbench environment

pippo-riscv: unfinished riscv porting

* Problem to fix
    When building pippo simulation or implementation, please don't add cbu related files into project
    

* FAQ
1, Q: How to use xupv2p board
   A: There is a workable fpga bitstream file(/board/xupv2p/demo_top.bit). And there are two methods to download application programs to xupv2p board's on-chip memory:
    a), initialize in the rtl memory(/rtl/verilog/imx_uocm.v), then implement(syn/translate/map/p&r) the system and generate fpga bitstream files.
        In this flow, the uartlite/uocm work in normal mode, you should put DIP switch 3 up(on/closed/logic 0).   
    b), when change software programs, to avoid fpga implement flow , you can download software bin files from PC to system's memory space(uocm). 
        In this mode, the uartlite/uocm work in burning mode, firstly you should put DIP switch 3 down(off/open/logic 0), then download software program bin files using uart tools(115200, 8 data bits, 2 stop bits, no parity). After burning complete(LED3 turn off), put DIP switch 3 on to exit burning mode.    
    In both mode, you should put DIP switch 0/1/2 down(off/open/logic 0), LED 0/1/2 will show the system's heartbeat/imx request/imxresponse state    
    
* Acknowledge:
    Some rtl logic is directly from previous great open source processor projects, such as Sun Microsystem's MicroSPARC/OpenSPARC T1/T2, Gaisler research's LEON, OpenRISC...
    
