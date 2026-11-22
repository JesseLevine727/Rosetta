// Lockstep host-SPI engine for the Rosetta harness.
//
// Plays one host register frame on ui[0..2] exactly as the Verilator harness
// does (toolchain/tests/rtl/rosetta_tb.cpp, spi_frame()): mode 0, MSB first,
// every phase H = 8 chip cycles:
//
//   8 cycles  CS_n low, SCLK low, MOSI 0
//   per bit   8 cycles SCLK low (MOSI = bit), sample MISO, 8 cycles SCLK high
//   8 cycles  SCLK low (MOSI keeps the last bit), CS_n low
//   8 cycles  idle: CS_n high, SCLK low, MOSI 0
//
// A write is 40 bits {0, addr[6:0], data[31:0]} (664 cycles), a read 48 bits
// {1, addr[6:0], 40 x 0} (792 cycles); `rdata` = the last 32 MISO bits.
//
// The engine advances only on clk60 edges where the chip is clocked
// (`ce` = 1), so the chip sees the same input on the same edge as in the
// Verilator harness however long the chip clock is paused in between.
// `busy_n` is the busy state after the current edge, so the harness can
// register the chip clock enable for the next edge.
`timescale 1ns / 1ps
`default_nettype none

module rh_hostspi (
    input  wire        clk,
    input  wire        rstn,
    input  wire        ce,          // the chip is clocked at this clk edge
    input  wire        go,          // one-cycle start pulse (ignored while busy)
    input  wire        rd,          // 1: 48-bit read frame, 0: 40-bit write frame
    input  wire [6:0]  addr,
    input  wire [31:0] data,
    input  wire        miso,        // uo[0], chip output after the last edge
    output reg         sclk,
    output reg         csn,
    output reg         mosi,
    output reg         busy,
    output wire        busy_n,
    output reg  [31:0] rdata
);
    // k = chip edges delivered in this frame so far; the outputs are the
    // values the chip samples at edge k+1.
    reg  [9:0]  k;
    reg  [9:0]  total;
    reg  [5:0]  nbits;
    reg  [47:0] sr;          // frame bits, MSB first (sr[47] = bit 0 of the frame)
    reg  [47:0] rx;

    // output function f(k) for the edge after k delivered edges
    function [2:0] f;        // {mosi, csn, sclk}
        input [9:0]  kk;
        input [5:0]  n;
        input [47:0] s;
        reg   [9:0]  rel;
        reg   [5:0]  j;
        begin
            if (kk < 10'd8)
                f = 3'b000;
            else if (kk < 10'd8 + {n, 4'b0}) begin
                rel = kk - 10'd8;
                j   = rel[9:4];
                f   = {s[47 - j], 1'b0, rel[3]};
            end else if (kk < 10'd16 + {n, 4'b0})
                f = {s[48 - n], 1'b0, 1'b0};      // last bit stays on MOSI
            else
                f = 3'b010;                       // CS_n high
        end
    endfunction

    wire        adv   = busy & ce;
    wire [9:0]  k1    = k + 10'd1;
    assign busy_n = go & ~busy ? 1'b1 : (adv ? (k1 != total) : busy);

    // sample MISO after the low half of bit j: k = 16 + 16 j
    wire        smp   = adv && (k >= 10'd16) && (k[3:0] == 4'd0) && (k < 10'd16 + {nbits, 4'b0});

    always @(posedge clk) begin
        if (!rstn) begin
            busy  <= 1'b0;
            k     <= 10'd0;
            total <= 10'd0;
            nbits <= 6'd0;
            sr    <= 48'd0;
            rx    <= 48'd0;
            rdata <= 32'd0;
            sclk  <= 1'b0;
            csn   <= 1'b1;
            mosi  <= 1'b0;
        end else if (go && !busy) begin
            busy  <= 1'b1;
            k     <= 10'd0;
            nbits <= rd ? 6'd48 : 6'd40;
            total <= rd ? 10'd792 : 10'd664;
            sr    <= rd ? {1'b1, addr, 40'd0} : {1'b0, addr, data, 8'd0};
            rx    <= 48'd0;
            {mosi, csn, sclk} <= 3'b000;
        end else if (adv) begin
            k <= k1;
            if (smp) rx <= {rx[46:0], miso};
            if (k1 == total) begin
                busy  <= 1'b0;
                rdata <= smp ? {rx[30:0], miso} : rx[31:0];
                {mosi, csn, sclk} <= 3'b010;
            end else begin
                {mosi, csn, sclk} <= f(k1, nbits, sr);
            end
        end
    end
endmodule

`default_nettype wire
