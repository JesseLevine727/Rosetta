// Asynchronous UART far end for the Rosetta harness.
//
// A transmitter clocked by the board's independent 125 MHz oscillator
// (sysclk), so its bit edges drift against the 60 MHz chip clock like a
// real remote device. The bit rate is set by a 32-bit phase accumulator:
// one bit = 2**32 / inc sysclk cycles (rate = 125 MHz * inc / 2**32).
//
// The processor fills a 64-byte buffer (clk60 domain) while the
// transmitter is idle, then toggles `go`. Frames: start bit, 8 data bits
// LSB first, optional parity, 1 or 2 stop bits, then `gap` idle bits.
// `tx` idles high. Configuration (count, parity, stop2, gap, inc) must be
// stable while busy; `go` and `busy` cross through 2-flop synchronisers.
`timescale 1ns / 1ps
`default_nettype none

module rh_async_uart (
    input  wire        clk60,
    input  wire        rstn,          // clk60 domain
    input  wire        we,            // clk60: write wdata at waddr
    input  wire [5:0]  waddr,
    input  wire [7:0]  wdata,
    input  wire        go_tgl,        // clk60: toggle starts a burst
    input  wire        sysclk,
    input  wire [31:0] inc,
    input  wire [5:0]  count_m1,      // bytes - 1
    input  wire [1:0]  parity,        // 0 none, 1 even, 2 odd
    input  wire        stop2,
    input  wire [2:0]  gap,
    output reg         tx = 1'b1,     // sysclk domain
    output wire        busy60         // clk60 domain
);
    reg [7:0] buf_mem [0:63];
    always @(posedge clk60) if (we) buf_mem[waddr] <= wdata;

    // go: clk60 toggle -> sysclk
    (* ASYNC_REG = "TRUE" *) reg g0 = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg g1 = 1'b0;
    reg g2 = 1'b0;
    always @(posedge sysclk) begin g0 <= go_tgl; g1 <= g0; g2 <= g1; end
    wire start = g1 ^ g2;

    reg        busy = 1'b0;
    reg [31:0] acc = 32'd0;
    reg [5:0]  idx = 6'd0;
    reg [3:0]  bitn = 4'd0;      // bit index within the frame
    reg [3:0]  nbits = 4'd0;     // frame length in bits incl. gap
    reg [7:0]  cur = 8'd0;
    reg        par = 1'b0;
    wire [32:0] acc_n = {1'b0, acc} + {1'b0, inc};
    wire        tick  = acc_n[32];

    // level of frame bit b of byte d
    function lvl;
        input [3:0] b;
        input [7:0] d;
        input [1:0] p;
        begin
            if (b == 4'd0)                lvl = 1'b0;                       // start
            else if (b <= 4'd8)           lvl = d[b - 4'd1];
            else if (b == 4'd9 && p != 0) lvl = (^d) ^ (p == 2'd2);        // parity
            else                          lvl = 1'b1;                       // stop, gap
        end
    endfunction

    always @(posedge sysclk) begin
        if (!busy) begin
            tx <= 1'b1;
            if (start) begin
                busy  <= 1'b1;
                acc   <= 32'd0;
                idx   <= 6'd0;
                bitn  <= 4'd0;
                cur   <= buf_mem[0];
                nbits <= 4'd10 + (parity != 0) + stop2 + gap;
                tx    <= 1'b0;
            end
        end else begin
            acc <= acc_n[31:0];
            if (tick) begin
                if (bitn + 4'd1 == nbits) begin
                    bitn <= 4'd0;
                    if (idx == count_m1) begin
                        busy <= 1'b0;
                        tx   <= 1'b1;
                    end else begin
                        idx <= idx + 6'd1;
                        cur <= buf_mem[idx + 6'd1];
                        tx  <= 1'b0;
                    end
                end else begin
                    bitn <= bitn + 4'd1;
                    tx   <= lvl(bitn + 4'd1, cur, parity);
                end
            end
        end
    end

    (* ASYNC_REG = "TRUE" *) reg b0 = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg b1 = 1'b0;
    always @(posedge clk60) begin b0 <= busy; b1 <= b0; end
    assign busy60 = b1;

    wire _unused = &{1'b0, rstn, par};
endmodule

`default_nettype wire
