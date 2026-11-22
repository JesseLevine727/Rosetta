// Lockstep waveform player for the Rosetta harness (the Verilator harness's
// V / P commands, toolchain/tests/rtl/rosetta_tb.cpp).
//
// A queue of runs {drive mask d[7:0], value mask v[7:0], n[15:0] cycles} in a
// block RAM (written by the processor through an AXI BRAM controller; `wp`
// counts the entries written, modulo 65536). One cycle of the head run is
// played per chip edge: (cur_d, cur_v) is the channel state for the next
// chip edge, 0/0 when the queue is empty. Entries are fetched ahead; when a
// run ends and the next one is not fetched yet, `ready_n` drops so the
// harness pauses lockstep runs for a cycle or two (invisible to the chip).
`timescale 1ns / 1ps
`default_nettype none

module rh_wave #(
    parameter integer AW = 14
) (
    input  wire          clk,
    input  wire          rstn,
    input  wire          ce,          // the chip is clocked at this clk edge
    input  wire [15:0]   wp,          // entries written (mod 65536)
    output reg  [15:0]   rp,          // entries fetched (mod 65536)
    output wire [7:0]    cur_d,
    output wire [7:0]    cur_v,
    output wire          ready_n,     // after this edge: the next chip edge may come
    output wire          empty,       // nothing playing or pending
    output reg           bram_en,
    output wire [AW-1:0] bram_addr,
    input  wire [31:0]   bram_dout
);
    reg  [15:0] rem;                  // cycles left in the current run (0: none)
    reg  [15:0] cd_v;                 // {d, v} of the current run
    reg  [31:0] nxt;
    reg         nxt_v;
    reg         rd_q;                 // a read was issued last cycle

    wire        pend   = (rp != wp);
    wire        cons   = ce && rem != 16'd0;                  // one cycle of the run is used
    wire [15:0] rem_d  = rem - {15'd0, cons};
    wire        take   = (rem_d == 16'd0) && nxt_v;           // the next run becomes current
    wire [15:0] rem_n  = take ? nxt[15:0] : rem_d;
    wire        nxtv_n = (nxt_v && !take) || rd_q;
    wire [15:0] rp_n   = rd_q ? rp + 16'd1 : rp;
    // the next edge needs a run that is pending but not current yet: wait
    assign ready_n = !((rem_n == 16'd0) && (nxtv_n || (rp_n != wp) || (bram_en && !rd_q)));
    assign empty   = (rem == 16'd0) && !nxt_v && !pend && !rd_q && !bram_en;

    assign cur_d = (rem != 16'd0) ? cd_v[15:8] : 8'd0;
    assign cur_v = (rem != 16'd0) ? cd_v[7:0]  : 8'd0;
    assign bram_addr = rp[AW-1:0];

    always @(posedge clk) begin
        if (!rstn) begin
            rem     <= 16'd0;
            cd_v    <= 16'd0;
            nxt     <= 32'd0;
            nxt_v   <= 1'b0;
            rd_q    <= 1'b0;
            rp      <= 16'd0;
            bram_en <= 1'b0;
        end else begin
            rem <= rem_n;
            if (take) cd_v <= nxt[31:16];
            // fetch: one read in flight at a time, into the single prefetch slot
            rd_q <= bram_en;
            if (rd_q) begin
                nxt   <= bram_dout;
                nxt_v <= 1'b1;
                rp    <= rp + 16'd1;
            end else if (take) begin
                nxt_v <= 1'b0;
            end
            bram_en <= !bram_en && !rd_q && pend && (!nxt_v || take);
        end
    end
endmodule

`default_nettype wire
