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
