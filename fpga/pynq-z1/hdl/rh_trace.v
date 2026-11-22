// Change trace for the Rosetta harness.
//
// At every clk60 edge where the chip is clocked (`valid`), the harness hands
// in the chip pins as the chip sees them at that edge and the number of chip
// edges delivered before it (`cycle`). An entry is stored when the pins
// differ from the last stored ones (and always for the first sample after
// `clear`), which is exactly the Verilator harness's trace: entry "cycle n" =
// outputs after chip edge n and the inputs applied for edge n+1.
//
// Entries go into a circular buffer of 2**AW entries held in three 32-bit
// block RAMs written in parallel (w0, w1, w2 at the same address):
//   w0 = cycle[31:0]
//   w1 = {flags[7:0], ui[7:0], cycle[47:32]}
//   w2 = {uio_in[7:0], uio_oe[7:0], uio_out[7:0], uo[7:0]}
// The processor reads them through AXI BRAM controllers and acknowledges by
// writing its read pointer `rp`; `wp` and `rp` count entries modulo 65536.
// If a store finds the buffer full, it is dropped and `overflow` latches
// (cleared by `clear`). `almost_full` asks the harness to pause the chip
// clock (lockstep runs), which makes the trace lossless.
`timescale 1ns / 1ps
`default_nettype none

module rh_trace #(
    parameter integer AW = 14,
    parameter integer MARGIN = 16
) (
    input  wire          clk,
    input  wire          rstn,
    input  wire          valid,
    input  wire [47:0]   cycle,
    input  wire [39:0]   pins,       // {uio_in, uio_oe, uio_out, uo, ui}
    input  wire [7:0]    flags,
    input  wire          en,
    input  wire          clear,      // one-cycle pulse
    input  wire [15:0]   rp,
    output reg  [15:0]   wp,
    output reg           overflow,
    output wire          almost_full,
    output wire          first_pending,
    output wire          bram_en,
    output wire [3:0]    bram_we,
    output wire [AW-1:0] bram_waddr,
    output wire [31:0]   din0,
    output wire [31:0]   din1,
    output wire [31:0]   din2
);
    localparam integer DEPTH = 1 << AW;

    reg         s_v;
    reg  [39:0] s_pins;
    reg  [47:0] s_cyc;
    reg  [7:0]  s_flags;
    reg  [39:0] prev;
    reg         first;

    wire [15:0] count = wp - rp;
    wire        full  = count >= DEPTH;
    assign almost_full   = count >= DEPTH - MARGIN;
    assign first_pending = first;

    wire store = s_v && (first || (s_pins != prev));
    wire wr    = store && !full;

    always @(posedge clk) begin
        if (!rstn) begin
            s_v      <= 1'b0;
            s_pins   <= 40'd0;
            s_cyc    <= 48'd0;
            s_flags  <= 8'd0;
            prev     <= 40'd0;
            first    <= 1'b1;
            wp       <= 16'd0;
            overflow <= 1'b0;
        end else begin
            s_v <= valid & en;
            if (valid) begin
                s_pins  <= pins;
                s_cyc   <= cycle;
                s_flags <= flags;
            end
            if (clear) begin
                first    <= 1'b1;
                overflow <= 1'b0;
            end else if (store) begin
                prev  <= s_pins;
                first <= 1'b0;
                if (full) overflow <= 1'b1;
                else      wp <= wp + 16'd1;
            end
        end
    end

    assign bram_en    = wr;
    assign bram_we    = {4{wr}};
    assign bram_waddr = wp[AW-1:0];
    assign din0       = s_cyc[31:0];
    assign din1       = {s_flags, s_pins[7:0], s_cyc[47:32]};
    assign din2       = s_pins[39:8];
endmodule

`default_nettype wire
