/*
 * File:        demo_clk.v
 * Project:     pippo
 * Designer:    kiss@pwrsemi
 * Mainteiner:  kiss@pwrsemi
 * Checker:
 * Assigner:    
 * Description:
 *      clock module for FPGA demo on XUP board
 *
 */
 

module demo_clk (
    CLK_IN,
    RST,
    CLK1X, 
    CLK2X,
    LOCK
);

input   CLK_IN;
input   RST;

output  CLK1X;
output  CLK2X;
output  LOCK;

wire    CLK_INW;
wire    CLK1X_W;
wire    CLK2X_W;
wire    GND;

assign GND = 1'b0;
 
//
// IBUFG
//  
IBUFG IBUFG_inst (
  .I(CLK_IN),           
  .O(CLK_INW)           
);
     
//
// BUFG
//
BUFG  U_BUFG (
    .I(CLK2X_W),  
    .O(CLK2X)
);

BUFG  U2_BUFG (
    .I(CLK1X_W),  
    .O(CLK1X)
);

// Attributes for functional simulation//
// synopsys translate_off
       defparam U_DCM.DLL_FREQUENCY_MODE = "LOW";
       defparam U_DCM.DUTY_CYCLE_CORRECTION = "TRUE";
       defparam U_DCM.STARTUP_WAIT = "TRUE";
// synopsys translate_on
// Instantiate the DCM primitive//
DCM U_DCM ( 
    .CLKFB(CLK1X), 
    .CLKIN(CLK_INW), 
    .DSSEN(GND), 
    .PSCLK(GND), 
    .PSEN(GND), 
    .PSINCDEC(GND), 
    .RST(RST), 
    .CLK0(CLK1X_W),
    .CLK2X(CLK2X_W),  
    .LOCKED(LOCK)
);
// synthesis attribute declarations
/* synopsys attribute 

DLL_FREQUENCY_MODE "LOW"
DUTY_CYCLE_CORRECTION "TRUE"
STARTUP_WAIT "TRUE"
*/

endmodule  