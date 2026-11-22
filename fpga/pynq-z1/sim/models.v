// Behavioural stand-ins for the Xilinx primitives used by the harness (sim only).
`timescale 1ns / 1ps
module BUFGCE (input wire I, input wire CE, output wire O);
    reg ce_l = 1'b0;
    always @(*) if (!I) ce_l = CE;     // CE latched while I is low (glitch-free gate)
    assign O = I & ce_l;
endmodule
module IOBUF (inout wire IO, input wire I, input wire T, output wire O);
    assign IO = T ? 1'bz : I;
    assign O  = IO;
endmodule
// Behavioural stand-ins for the async-line clocking and fine delay (sim only)
module BUFG (input wire I, output wire O);
    assign O = I;
endmodule
module MMCME2_BASE #(
    parameter real CLKIN1_PERIOD = 8.0, parameter integer DIVCLK_DIVIDE = 1,
    parameter real CLKFBOUT_MULT_F = 8.0, parameter real CLKOUT0_DIVIDE_F = 5.0
) (
    input wire CLKIN1, input wire CLKFBIN, output wire CLKFBOUT, output reg CLKOUT0 = 1'b0,
    output reg LOCKED = 1'b0, input wire PWRDWN, input wire RST,
    output wire CLKFBOUTB, output wire CLKOUT0B, output wire CLKOUT1, output wire CLKOUT1B,
    output wire CLKOUT2, output wire CLKOUT2B, output wire CLKOUT3, output wire CLKOUT3B,
    output wire CLKOUT4, output wire CLKOUT5, output wire CLKOUT6
);
    // free-running at the nominal output period (not phase-locked to CLKIN1)
    real half;
    initial begin
        half = CLKIN1_PERIOD * DIVCLK_DIVIDE * CLKOUT0_DIVIDE_F / CLKFBOUT_MULT_F / 2.0;
        #100 LOCKED = 1'b1;
    end
    always #(half) CLKOUT0 = ~CLKOUT0;
    assign CLKFBOUT = 1'b0;
endmodule
module IDELAYCTRL (input wire REFCLK, input wire RST, output wire RDY);
    assign RDY = ~RST;
endmodule
module IDELAYE2 #(
    parameter IDELAY_TYPE = "VAR_LOAD", parameter DELAY_SRC = "DATAIN", parameter integer IDELAY_VALUE = 0,
    parameter HIGH_PERFORMANCE_MODE = "TRUE", parameter SIGNAL_PATTERN = "DATA",
    parameter real REFCLK_FREQUENCY = 200.0, parameter CINVCTRL_SEL = "FALSE", parameter PIPE_SEL = "FALSE"
) (
    input wire C, input wire REGRST, input wire LD, input wire CE, input wire INC, input wire CINVCTRL,
    input wire [4:0] CNTVALUEIN, input wire IDATAIN, input wire DATAIN, input wire LDPIPEEN,
    output reg DATAOUT = 1'b0, output wire [4:0] CNTVALUEOUT
);
    reg [4:0] tap = 5'd0;
    always @(posedge C) if (LD) tap <= CNTVALUEIN;
    assign CNTVALUEOUT = tap;
    // 0.6 ns intrinsic + 78.125 ps per tap (transport delay)
    always @(DATAIN) DATAOUT <= #(0.6 + 0.078125 * tap) DATAIN;
endmodule
