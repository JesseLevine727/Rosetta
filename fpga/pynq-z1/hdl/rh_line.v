// Asynchronous far end with sub-nanosecond edge placement (board only).
//
// Two lines (for example 10BASE-T RX, USB D-/D+, a CAN node) whose every
// edge time is computed by the processor in continuous time (rate offset,
// jitter, duty-cycle distortion: the same line models as the shared tests)
// and played here from a clock that is independent of the chip's: an MMCM on
// the board's 125 MHz oscillator makes clk200 (200 MHz, also the IDELAYCTRL
// reference).
//
// Event word (block RAM, written by the processor through an AXI BRAM
// controller, read here at 200 MHz):
//   [15:0]  dt: 200 MHz cycles since the previous event's cycle (>= 4; the
//           first event's dt counts from its fetch after `go`)
//   [16]    half: the edge is at the falling clock edge of that cycle (+2.5 ns)
//   [21:17] tap: IDELAYE2 taps of 78.125 ps added to the edge (0-31)
//   [23:22] the lines this event sets (bit 22 line 0, bit 23 line 1)
//   [25:24] their new levels (line 0, line 1)
//   [27:26] their drive enables (0 = released)
// Edge time = dt-accumulated cycles x 5 ns + half x 2.5 ns + tap x 78.125 ps:
// a 78.125 ps grid. A line's value is the XOR of a posedge and a negedge
// toggle flop (glitch-free: they never toggle together), delayed by its own
// IDELAYE2 (DATAIN path, VAR_LOAD) whose tap is loaded two cycles before the
// edge, while the line is stable. Successive events must be at least 4
// cycles (20 ns) apart; an event may set both lines at once (USB D-/D+).
`timescale 1ns / 1ps
`default_nettype none

module rh_line #(
    parameter integer AW = 14
) (
    input  wire          clk60,
    input  wire          sysclk,          // 125 MHz board oscillator
    input  wire          go_tgl,          // clk60: toggle starts playback from event 0
    input  wire [15:0]   n_events,        // clk60, quasi-static: events to play
    output wire          busy60,          // clk60
    output wire          ready60,         // clk60: MMCM locked and IDELAYCTRL ready
    output wire [1:0]    val,             // line values (asynchronous to every other clock)
    output wire [1:0]    drv,             // line drive enables
    output wire          bram_clk,
    output reg           bram_en,
    output wire [AW-1:0] bram_addr,
    input  wire [31:0]   bram_dout
);
    // ---------------------------------------------------------------- clocks
    wire clkfb, clk200_mmcm, clk200, locked;
    MMCME2_BASE #(
        .CLKIN1_PERIOD(8.0), .DIVCLK_DIVIDE(1), .CLKFBOUT_MULT_F(8.0),   // VCO 1000 MHz
        .CLKOUT0_DIVIDE_F(5.0)                                            // 200 MHz
    ) u_mmcm (
        .CLKIN1(sysclk), .CLKFBIN(clkfb), .CLKFBOUT(clkfb), .CLKOUT0(clk200_mmcm),
        .LOCKED(locked), .PWRDWN(1'b0), .RST(1'b0),
        .CLKFBOUTB(), .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
        .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6());
    BUFG u_bufg200 (.I(clk200_mmcm), .O(clk200));
    assign bram_clk = clk200;

    wire dly_rdy;
    (* IODELAY_GROUP = "rh_line" *)
    IDELAYCTRL u_dlyctrl (.REFCLK(clk200), .RST(!locked), .RDY(dly_rdy));

    // ---------------------------------------------------------------- control CDC
    (* ASYNC_REG = "TRUE" *) reg g0 = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg g1 = 1'b0;
    reg g2 = 1'b0;
    always @(posedge clk200) begin g0 <= go_tgl; g1 <= g0; g2 <= g1; end
    wire go = g1 ^ g2;

    // ---------------------------------------------------------------- player
    reg         run = 1'b0;
    reg  [15:0] idx = 16'd0;          // next event to fetch
    reg  [15:0] left = 16'd0;         // events not yet applied
    reg  [31:0] nxt = 32'd0;
    reg         nxt_v = 1'b0;
    reg         rd_q = 1'b0;
    reg  [31:0] cur = 32'd0;
    reg         cur_v = 1'b0;
    reg  [15:0] cnt = 16'd0;
    reg  [1:0]  pos = 2'b00, lvl = 2'b00, drv_q = 2'b00;
    reg  [1:0]  negreq = 2'b00;
    reg  [1:0]  ld = 2'b00;
    reg  [4:0]  tapv = 5'd0;
    (* DONT_TOUCH = "TRUE" *) reg [1:0] neg = 2'b00;

    assign bram_addr = idx[AW-1:0];
    wire [1:0] c_mask = cur[23:22];
    wire [1:0] c_lvl  = cur[25:24];
    wire [1:0] c_drv  = cur[27:26];
    wire [1:0] c_tgl  = c_mask & (c_lvl ^ lvl);

    always @(posedge clk200) begin
        rd_q    <= bram_en;
        bram_en <= 1'b0;
        ld      <= 2'b00;
        negreq  <= 2'b00;
        if (go) begin
            run   <= (n_events != 16'd0);
            idx   <= 16'd0;
            left  <= n_events;
            nxt_v <= 1'b0;
            cur_v <= 1'b0;
            rd_q  <= 1'b0;
        end else if (run) begin
            // fetch into the prefetch slot
            if (rd_q) begin
                nxt   <= bram_dout;
                nxt_v <= 1'b1;
                idx   <= idx + 16'd1;
            end else if (!nxt_v && !bram_en && idx != n_events) begin
                bram_en <= 1'b1;
            end
            if (!cur_v) begin
                if (nxt_v) begin                         // the first event
                    cur   <= nxt;
                    cur_v <= 1'b1;
                    nxt_v <= 1'b0;
                    cnt   <= nxt[15:0] - 16'd1;
                end
            end else begin
                if (cnt == 16'd2) begin                  // load the tap while the lines are still
                    ld   <= c_mask;
                    tapv <= cur[21:17];
                end
                if (cnt == 16'd0) begin                  // the edge's cycle
                    if (cur[16]) negreq <= c_tgl;
                    else         pos    <= pos ^ c_tgl;
                    lvl   <= (lvl & ~c_mask) | (c_lvl & c_mask);
                    drv_q <= (drv_q & ~c_mask) | (c_drv & c_mask);
                    if (left == 16'd1) begin
                        run   <= 1'b0;
                        cur_v <= 1'b0;
                    end else if (nxt_v) begin
                        cur   <= nxt;
                        nxt_v <= 1'b0;
                        cnt   <= nxt[15:0] - 16'd1;
                    end else begin
                        cur_v <= 1'b0;                   // underrun (dt < 4): stop
                        run   <= 1'b0;
                    end
                    left <= left - 16'd1;
                end else begin
                    cnt <= cnt - 16'd1;
                end
            end
        end
    end

    always @(negedge clk200) begin
        if (negreq[0]) neg[0] <= ~neg[0];
        if (negreq[1]) neg[1] <= ~neg[1];
    end

    // ---------------------------------------------------------------- fine delay
    genvar k;
    generate
        for (k = 0; k < 2; k = k + 1) begin : g_dly
            wire raw = pos[k] ^ neg[k];
            (* IODELAY_GROUP = "rh_line" *)
            IDELAYE2 #(
                .IDELAY_TYPE("VAR_LOAD"), .DELAY_SRC("DATAIN"), .IDELAY_VALUE(0),
                .HIGH_PERFORMANCE_MODE("TRUE"), .SIGNAL_PATTERN("DATA"),
                .REFCLK_FREQUENCY(200.0), .CINVCTRL_SEL("FALSE"), .PIPE_SEL("FALSE")
            ) u_idly (
                .C(clk200), .REGRST(1'b0), .LD(ld[k]), .CE(1'b0), .INC(1'b0),
                .CINVCTRL(1'b0), .CNTVALUEIN(tapv), .IDATAIN(1'b0), .DATAIN(raw),
                .LDPIPEEN(1'b0), .DATAOUT(val[k]), .CNTVALUEOUT());
        end
    endgenerate
    assign drv = drv_q;

    // ---------------------------------------------------------------- status CDC
    (* ASYNC_REG = "TRUE" *) reg [1:0] b0 = 2'b00;
    (* ASYNC_REG = "TRUE" *) reg [1:0] b1 = 2'b00;
    always @(posedge clk60) begin b0 <= {run | go, locked & dly_rdy}; b1 <= b0; end
    assign busy60  = b1[1];
    assign ready60 = b1[0];
endmodule

`default_nettype wire
