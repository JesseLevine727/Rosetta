`default_nettype none
// ============================================================================
//  Formal properties for rosetta_crc (check unit).
//
//  Written from docs/blocks/crc.md and docs/conventions.md only; the property
//  writer has not read the RTL. Everything here is about ports.
//
//  Modules in this file:
//    rosetta_crc_props       single-copy checker: spec items 1, 2, 3 + port
//                            table, and the known answers (item 4) through
//    rosetta_crc_ka_monitor  one instance per row of the known-answer table
//    rosetta_crc_lin_props   three-instance lockstep check of linearity (item 5)
//
//  Traceability (spec "Required properties" -> assertion labels):
//    1  update rule ............ ap_shift (and ap_init, ap_hold below)
//    2  bits >= W are zero ..... ap_upper_state, ap_upper_value
//    3  init priority .......... ap_init (checked whatever en_i/bit_i are)
//       hold when idle ......... ap_hold
//       reset value ............ ap_rst_async, ap_rst_after
//    4  known answers .......... ka_value in each rosetta_crc_ka_monitor
//                                (plus ka_step: the state after every bit)
//    5  linearity .............. al_state, al_value (rosetta_crc_lin_props)
//    port table, value_o ....... ap_value
//    "outputs are functions of the registered state only" follows from
//       ap_shift/ap_init/ap_hold + ap_rst_*: state_o at cycle t+1 is pinned
//       to a function of cycle-t values, so a combinational path from
//       bit_i/en_i/init_i to state_o would let the solver change it; and
//       ap_value pins value_o to state_o and the configuration.
//
//  Environment. Inputs are free every cycle, including the configuration,
//  with one assumption: rst_n is low in the first cycle (flops start
//  arbitrary). The configuration may therefore change at any time, and the
//  checks are written so that every one of them is about behaviour the spec
//  defines:
//    - ap_value, ap_upper_*, ap_rst_* hold on every cycle (the spec says
//      "always", and defines both outputs from the state and the current
//      cfg_width_m1_i / cfg_xorout_i).
//    - The update checks need the state to be meaningful at the current width.
//      The spec defines the state as W bits and says nothing about what it
//      becomes when W changes, so a width change without init_i makes the
//      state "undefined" (tracked by `defd`) until the next init_i or reset.
//      cfg_poly_i and cfg_init_i are only sampled at the edge they act on,
//      so they may change freely.
//  Define CRC_FV_CFG_STABLE to assume instead that the four cfg_* inputs are
//  constant for the whole trace (the strictest reading of "configuration
//  inputs are assumed stable while the block is active").
//
//  Async reset: sby (multiclock off) runs async2sync, so an asynchronously
//  reset flop reads as its reset value in the same cycle rst_n is low. That
//  is what ap_rst_async checks; a synchronous reset fails it.
// ============================================================================

module rosetta_crc_props (
    input wire        clk,
    input wire        rst_n,
    input wire [31:0] cfg_poly_i,
    input wire [31:0] cfg_init_i,
    input wire [4:0]  cfg_width_m1_i,
    input wire        cfg_xorout_i,
    input wire        init_i,
    input wire        en_i,
    input wire        bit_i,
    input wire [31:0] state_o,
    input wire [31:0] value_o
);

    // ------------------------------------------------------------------
    // Reference arithmetic, deliberately NOT the spec's shift-and-xor form.
    // ------------------------------------------------------------------

    // Low W bits set, W = wm1 + 1, built bit by bit from a comparison.
    function automatic [31:0] f_mask(input [4:0] wm1);
        integer i;
        begin
            for (i = 0; i < 32; i = i + 1)
                f_mask[i] = (i <= wm1);
        end
    endfunction

    // One CRC step as polynomial arithmetic over GF(2):
    //     S'(x) = ( x * S(x)  +  b * x^W )  mod  G(x),   G(x) = x^W + P(x)
    // where P(x) is cfg_poly with only its low W coefficients (the spec's
    // "without its top term, right-aligned in the low W bits"). T has degree
    // at most W, so the reduction is a single conditional subtraction of G.
    function automatic [31:0] f_step(input [31:0] s, input b, input [31:0] p,
                                     input [4:0] wm1);
        reg   [32:0] g, t;
        reg   [5:0]  w;
        reg          lead;
        integer      i;
        begin
            w = {1'b0, wm1} + 6'd1;
            for (i = 0; i <= 32; i = i + 1) begin
                // generator, top term explicit
                g[i] = (i == w) | ((i < w) & p[i % 32]);
                // x * S(x): coefficient i comes from s[i-1] (S has degree < W)
                t[i] = (i >= 1) & (i <= w) & s[(i + 31) % 32];
                // + b * x^W
                if (i == w) t[i] = t[i] ^ b;
            end
            lead = 1'b0;
            for (i = 0; i <= 32; i = i + 1)
                if (i == w) lead = t[i];
            if (lead) t = t ^ g;
            f_step = t[31:0];
        end
    endfunction

    wire [31:0] m  = f_mask(cfg_width_m1_i);
    wire [31:0] xm = cfg_xorout_i ? m : 32'd0;

    // ------------------------------------------------------------------
    // Environment
    // ------------------------------------------------------------------
    reg past_valid = 1'b0;
    always @(posedge clk) past_valid <= 1'b1;

    // Conventions: reset is low for the first cycle; flops start arbitrary.
    always @(*) if (!past_valid) assume (!rst_n);

`ifdef CRC_FV_CFG_STABLE
    reg [69:0] cfg_q;
    always @(posedge clk) cfg_q <= {cfg_poly_i, cfg_init_i, cfg_width_m1_i, cfg_xorout_i};
    always @(*) if (past_valid)
        assume ({cfg_poly_i, cfg_init_i, cfg_width_m1_i, cfg_xorout_i} == cfg_q);
`endif

    // ------------------------------------------------------------------
    // History of the previous cycle (edge t -> t+1 is checked at t+1)
    // ------------------------------------------------------------------
    reg        rst_q, init_q, en_q, bit_q, def_q, fb_q, nz_q;
    reg  [4:0] wm1_q;
    reg [31:0] state_q, exp_init_q, exp_shift_q;
    reg        up_poly_q, up_init_q;

    // defd: the current state_o is a state the spec defines at the current W.
    // True during/just after reset (state 0), after init_i at an unchanged W,
    // and kept by shift/hold edges as long as W does not change.
    wire defd = !past_valid || !rst_n || !rst_q ||
                ((wm1_q == cfg_width_m1_i) && (init_q || def_q));

    // feedback bit, from state_o by index search (only used for covers)
    reg fb_now;
    integer k;
    always @(*) begin
        fb_now = bit_i;
        for (k = 0; k < 32; k = k + 1)
            if (k == cfg_width_m1_i) fb_now = state_o[k] ^ bit_i;
    end

    always @(posedge clk) begin
        rst_q       <= rst_n;
        init_q      <= init_i;
        en_q        <= en_i;
        bit_q       <= bit_i;
        def_q       <= defd;
        wm1_q       <= cfg_width_m1_i;
        state_q     <= state_o;
        exp_init_q  <= cfg_init_i & m;
        exp_shift_q <= f_step(state_o, bit_i, cfg_poly_i, cfg_width_m1_i);
        fb_q        <= fb_now;
        nz_q        <= (state_o != 32'd0);
        up_poly_q   <= (cfg_poly_i & ~m) != 32'd0;
        up_init_q   <= (cfg_init_i & ~m) != 32'd0;
    end

    // an update edge t -> t+1 with no reset at either end, W meaningful
    wire chk = past_valid && rst_q && rst_n && defd;

    // ------------------------------------------------------------------
    // Assertions
    // ------------------------------------------------------------------
    always @(*) begin
        // [3] reset value; asynchronous: zero while rst_n is low ...
        if (!rst_n)
            ap_rst_async: assert (state_o == 32'd0);
        // ... and zero on the first cycle after release
        if (past_valid && !rst_q && rst_n)
            ap_rst_after: assert (state_o == 32'd0);

        // [3] init_i loads the low W bits of cfg_init_i, whatever en_i/bit_i
        if (chk && init_q)
            ap_init: assert (state_o == exp_init_q);
        // [1] en_i: one CRC step
        if (chk && !init_q && en_q)
            ap_shift: assert (state_o == exp_shift_q);
        // [3] neither: hold
        if (chk && !init_q && !en_q)
            ap_hold: assert (state_o == state_q);

        // [2] bits W and above of both outputs are always zero
        ap_upper_state: assert ((state_o & ~m) == 32'd0);
        ap_upper_value: assert ((value_o & ~m) == 32'd0);

        // port table: value_o is the state, complemented in the low W bits
        // when cfg_xorout_i (also during reset: the state is 0 then)
        ap_value: assert (value_o == (state_o ^ xm));
    end

    // ------------------------------------------------------------------
    // Covers: every assumption and every feature is reachable
    // ------------------------------------------------------------------
    always @(*) begin
        // the initial-reset assumption still lets reset release
        cp_rst_release: cover (past_valid && !rst_q && rst_n);
        // reset in the middle of a computation clears a live state
        cp_rst_midrun:  cover (past_valid && rst_q && nz_q && !rst_n);
        // init alone, and init winning over a simultaneous en_i whose shift
        // would have produced something else
        cp_init:        cover (chk && init_q && !en_q && state_o != 32'd0);
        cp_init_vs_en:  cover (chk && init_q && en_q && exp_shift_q != exp_init_q);
        // init with garbage above W in cfg_init_i (ignored)
        cp_init_upper:  cover (chk && init_q && up_init_q && wm1_q != 5'd31);
        // shifts with and without feedback, at the extreme and a middle width
        cp_shift_fb0:   cover (chk && !init_q && en_q && !fb_q && state_o != 32'd0);
        cp_shift_fb1:   cover (chk && !init_q && en_q &&  fb_q && state_o != 32'd0);
        cp_shift_w1:    cover (chk && !init_q && en_q && fb_q && wm1_q == 5'd0);
        cp_shift_w5:    cover (chk && !init_q && en_q && fb_q && wm1_q == 5'd4);
        cp_shift_w32:   cover (chk && !init_q && en_q && fb_q && wm1_q == 5'd31 && state_o[31]);
        // feedback with garbage above W in cfg_poly_i (ignored)
        cp_shift_upper: cover (chk && !init_q && en_q && fb_q && up_poly_q && wm1_q != 5'd31);
        // hold of a live state while bit_i toggles
        cp_hold:        cover (chk && !init_q && !en_q && bit_q && state_o != 32'd0);
        // final XOR visible
        cp_xorout:      cover (rst_n && cfg_xorout_i && state_o != 32'd0 && value_o != 32'd0);
        // top state bit set at W = 32
        cp_top32:       cover (rst_n && cfg_width_m1_i == 5'd31 && state_o[31]);
`ifndef CRC_FV_CFG_STABLE
        // width reconfigured while idle with a live state, then re-init
        cp_w_change:    cover (past_valid && rst_q && rst_n && !init_q && nz_q &&
                               wm1_q != cfg_width_m1_i);
        cp_redefined:   cover (past_valid && rst_q && rst_n && !def_q && defd &&
                               state_o != 32'd0);
`endif
    end

    // ------------------------------------------------------------------
    // [4] Known answers, data = ASCII "123456789" (72 wire bits).
    // The value_o constants are the spec's, confirmed independently by
    // known_answers.py (a bit-serial model and the standard table-driven
    // CRC agree with every one). The per-step state tables come from the
    // same script (crc_ka_states.svh); they are checked as lemmas so the
    // unbounded engine does not have to rediscover 72 steps of state.
    // ------------------------------------------------------------------
`include "crc_ka_states.svh"

`define CRC_KA(NAME, W_, POLY_, INIT_, XO_, LSB_, EXP_, ST_) \
    rosetta_crc_ka_monitor #( \
        .W(W_), .POLY(POLY_), .INIT(INIT_), .XOROUT(XO_), \
        .LSB_FIRST(LSB_), .EXPECT(EXP_), .STATES(ST_) \
    ) NAME ( \
        .clk(clk), .rst_n(rst_n), .cfg_poly_i(cfg_poly_i), \
        .cfg_init_i(cfg_init_i), .cfg_width_m1_i(cfg_width_m1_i), \
        .cfg_xorout_i(cfg_xorout_i), .init_i(init_i), .en_i(en_i), \
        .bit_i(bit_i), .state_o(state_o), .value_o(value_o) \
    );

    //       name           W   poly          init          xo lsb value_o       per-step states
    `CRC_KA(ka_crc32,      32, 32'h04C11DB7, 32'hFFFFFFFF, 1, 1, 32'h649C2FD3, KA_ST_CRC32)
    `CRC_KA(ka_crc16_usb,  16, 32'h00008005, 32'h0000FFFF, 1, 1, 32'h0000132D, KA_ST_CRC16_USB)
    `CRC_KA(ka_crc16_ccitt,16, 32'h00001021, 32'h0000FFFF, 0, 0, 32'h000029B1, KA_ST_CRC16_CCITT)
    `CRC_KA(ka_crc15_can,  15, 32'h00004599, 32'h00000000, 0, 0, 32'h0000059E, KA_ST_CRC15_CAN)
    `CRC_KA(ka_crc8,        8, 32'h00000007, 32'h00000000, 0, 0, 32'h000000F4, KA_ST_CRC8)
    `CRC_KA(ka_crc5_usb,    5, 32'h00000005, 32'h0000001F, 1, 1, 32'h00000013, KA_ST_CRC5_USB)
    `CRC_KA(ka_parity_msb,  1, 32'h00000001, 32'h00000000, 0, 0, 32'h00000001, KA_ST_PARITY_MSB)
    `CRC_KA(ka_parity_lsb,  1, 32'h00000001, 32'h00000000, 0, 1, 32'h00000001, KA_ST_PARITY_LSB)

`undef CRC_KA

endmodule


// ============================================================================
//  One known answer. Tracks a run that starts with init_i under the row's
//  configuration and then consumes exactly the row's 72 wire bits on en_i
//  cycles (idle cycles may be interleaved: they must hold). When all 72 have
//  been consumed, value_o must equal the row's constant (ka_value). On the
//  way, state_o must equal the software CRC's state after each bit
//  (ka_step); these lemmas make the property 1-step inductive and give a
//  sharper counterexample. Bits of cfg_poly_i and cfg_init_i above W are
//  left free (the block must ignore them).
// ============================================================================
module rosetta_crc_ka_monitor #(
    parameter integer W         = 32,
    parameter [31:0]  POLY      = 32'h0,
    parameter [31:0]  INIT      = 32'h0,
    parameter         XOROUT    = 1'b0,
    parameter         LSB_FIRST = 1'b0,
    parameter [31:0]  EXPECT    = 32'h0,
    parameter [73*32-1:0] STATES = {73{32'h0}}
) (
    input wire        clk,
    input wire        rst_n,
    input wire [31:0] cfg_poly_i,
    input wire [31:0] cfg_init_i,
    input wire [4:0]  cfg_width_m1_i,
    input wire        cfg_xorout_i,
    input wire        init_i,
    input wire        en_i,
    input wire        bit_i,
    input wire [31:0] state_o,
    input wire [31:0] value_o
);
    localparam [71:0] MSG = "123456789";     // MSG[71:64] = "1"
    localparam [31:0] M   = (W == 32) ? 32'hFFFF_FFFF : ((32'd1 << W) - 32'd1);

    // wire bit j (j = 0 first on the wire): byte j/8 of the text, bit j%8
    // counted from the LSB (LSB first) or from the MSB (MSB first).
    function automatic [71:0] f_seq(input lsb);
        integer j;
        begin
            for (j = 0; j < 72; j = j + 1)
                f_seq[j] = lsb ? MSG[64 - 8 * (j / 8) + (j % 8)]
                               : MSG[71 - j];
        end
    endfunction
    // padded to the counter's range so the bit select never reads x
    localparam [127:0] SEQ = {56'd0, f_seq(LSB_FIRST)};

    wire w_ok    = (cfg_width_m1_i == W - 1);
    wire load_ok = w_ok && ((cfg_init_i & M) == INIT);
    wire step_ok = w_ok && ((cfg_poly_i & M) == POLY);
    wire out_ok  = w_ok && (cfg_xorout_i == XOROUT);

    reg       trk = 1'b0;
    reg [6:0] n   = 7'd0;

    always @(posedge clk) begin
        if (!rst_n)
            trk <= 1'b0;
        else if (init_i) begin
            trk <= load_ok;
            n   <= 7'd0;
        end else if (!w_ok)
            trk <= 1'b0;
        else if (en_i) begin
            if (trk && step_ok && n < 7'd72 && bit_i == SEQ[n])
                n <= n + 7'd1;
            else
                trk <= 1'b0;
        end
    end

    wire done = rst_n && trk && (n == 7'd72) && out_ok;

    // expected state after n bits (loop instead of a variable part-select,
    // so no x is ever produced)
    reg [31:0] st_exp;
    integer j;
    always @(*) begin
        st_exp = 32'd0;
        for (j = 0; j <= 72; j = j + 1)
            if (n == j) st_exp = STATES[32 * j +: 32];
    end

    always @(*) begin
        if (rst_n && trk && w_ok) ka_step: assert (state_o == st_exp);
        if (done) ka_value: assert (value_o == EXPECT);
        ka_reached: cover (done);
    end
endmodule


// ============================================================================
//  [5] Linearity, three copies in lockstep (A, B, C).
//
//  The spec's identity crc(a ^ b) = crc(a) ^ crc(b) contains three CRC runs,
//  so it is checked on three instances of the design that share clock,
//  reset, cfg_poly_i, cfg_width_m1_i, cfg_xorout_i, init_i and en_i, with
//      bit_C  = bit_A  ^ bit_B,      cfg_init_C = cfg_init_A ^ cfg_init_B
//  (wired that way in crc_fv.sv). Then state_C = state_A ^ state_B must hold
//  on every cycle at which the state is defined. This is the spec's item with
//  init 0 (cfg_init_A = cfg_init_B = 0, or a common reset) as the special
//  case, generalised to any pair of initial values, which the update rule
//  equally implies (it is linear in state, input bit and loaded value). No
//  reference model is involved. cp_lin_init0 covers the spec's exact case.
// ============================================================================
module rosetta_crc_lin_props (
    input wire        clk,
    input wire        rst_n,
    input wire [4:0]  cfg_width_m1_i,
    input wire        cfg_xorout_i,
    input wire        init_i,
    input wire        en_i,
    input wire [31:0] cfg_init_a,
    input wire [31:0] cfg_init_b,
    input wire        bit_a,
    input wire        bit_b,
    input wire [31:0] state_a,
    input wire [31:0] state_b,
    input wire [31:0] state_c,
    input wire [31:0] value_a,
    input wire [31:0] value_b,
    input wire [31:0] value_c
);
    wire [31:0] m  = 32'hFFFF_FFFF >> (5'd31 - cfg_width_m1_i);
    wire [31:0] xm = cfg_xorout_i ? m : 32'd0;

    reg past_valid = 1'b0;
    always @(posedge clk) past_valid <= 1'b1;

    // same "defined" notion as in rosetta_crc_props (W unchanged since the
    // last common init or reset): the spec's "fixed configuration"
    reg       rst_q, init_q, def_q;
    reg [4:0] wm1_q;
    wire defd = !past_valid || !rst_n || !rst_q ||
                ((wm1_q == cfg_width_m1_i) && (init_q || def_q));

    // for the cover: all three started from 0, and how many bits since
    reg       z_q = 1'b0;
    reg [3:0] nb_q = 4'd0;

    always @(posedge clk) begin
        rst_q  <= rst_n;
        init_q <= init_i;
        def_q  <= defd;
        wm1_q  <= cfg_width_m1_i;
        if (!rst_n) begin
            z_q <= 1'b1; nb_q <= 4'd0;
        end else if (init_i) begin
            z_q <= (cfg_init_a & m) == 32'd0 && (cfg_init_b & m) == 32'd0;
            nb_q <= 4'd0;
        end else if (en_i && nb_q != 4'hF)
            nb_q <= nb_q + 4'd1;
    end

    always @(*) begin
        if (defd) begin
            al_state: assert (state_c == (state_a ^ state_b));
            // xorout complements all three, so the XOR of three values
            // carries one complement
            al_value: assert (value_c == (value_a ^ value_b ^ xm));
        end
        // the spec's case: equal-length inputs from init 0, 32-bit CRC,
        // a and b both non-trivial and different
        cp_lin_init0: cover (rst_n && defd && z_q && nb_q >= 4'd8 &&
                             cfg_width_m1_i == 5'd31 && state_a != 32'd0 &&
                             state_b != 32'd0 && state_a != state_b);
        // the generalisation: non-zero, different initial values
        cp_lin_init:  cover (rst_n && defd && !z_q && nb_q >= 4'd4 &&
                             state_a != state_b && state_c != 32'd0);
    end
endmodule

`default_nettype wire
