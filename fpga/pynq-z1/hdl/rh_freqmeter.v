// Frequency meter: a reference-clock divider toggles every REF_CYCLES
// reference cycles; in the measurement domain the toggle is synchronised and,
// on each toggle, `result` = increase of `count` since the previous toggle.
// With REF_CYCLES = 1 s of reference clock, result = chip clock edges per
// second. `seq` increments on every update.
`timescale 1ns / 1ps
`default_nettype none

module rh_freqmeter #(
    parameter integer REF_CYCLES = 100000000
) (
    input  wire        ref_clk,
    input  wire        clk,
    input  wire [31:0] count,
    output reg  [31:0] result = 32'd0,
    output reg  [3:0]  seq    = 4'd0
);
    reg [31:0] rcnt = 32'd0;
    reg        rtgl = 1'b0;
    always @(posedge ref_clk) begin
        if (rcnt == REF_CYCLES - 1) begin
            rcnt <= 32'd0;
            rtgl <= ~rtgl;
        end else begin
            rcnt <= rcnt + 32'd1;
        end
    end

    (* ASYNC_REG = "TRUE" *) reg sync0 = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg sync1 = 1'b0;
    reg        sync2 = 1'b0;
    reg [31:0] last  = 32'd0;
    always @(posedge clk) begin
        sync0 <= rtgl;
        sync1 <= sync0;
        sync2 <= sync1;
        if (sync1 ^ sync2) begin
            result <= count - last;
            last   <= count;
            seq    <= seq + 4'd1;
        end
    end
endmodule

`default_nettype wire
