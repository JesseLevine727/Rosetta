`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_cfg (lane.md: u_cfg / rosetta_lane_cfg),
//  the lane's mode-register file and its SETM write side-effect pulses.
//
//  Written from docs/blocks/lane.md sections 1, 2, 3, 5, 6, 8, 12,
//  docs/isa.md section 4, docs/decisions.md rulings 10, 21, 23 and
//  docs/conventions.md ONLY (the RTL was not read; only its port list).
//  Every property is about the block's ports.
//
//  Spec in one paragraph (lane.md 5.1, 5.2, 8). 198 stored bits, all reset
//  to 0 by the asynchronous active-low rst_n. "Low write to r" =
//  run_i && cfg_we_i && !cfg_hi_i && cfg_reg_i == r; it loads the stored
//  bits of register r from cfg_data_i at the end of the cycle (visible in the
//  next cycle). "High write to 8" = run_i && cfg_we_i && cfg_hi_i &&
//  cfg_reg_i == 8; it loads CRCPOLY[31:16]. A high write to any other number
//  and any write to 14..31 changes nothing. Registers keep their values while
//  halted. Pulses, combinational in the write cycle:
//    wr_tick  = low write to 0         wr_code  = low write to 5
//    wr_stuff = low write to 7         wr_evc[k] = low write to 10+k
//    wr_crc   = low or high write to 8, or low write to 9
//  Stored bit positions (lane.md table 5.1, rulings 10 and 23):
//    tick_o    = TICK[12:0]      period_o = PERIOD[15:0]
//    period2_o = PERIOD2[15:0]   thr0_o   = THR01[7:0]    (THR1 deleted)
//    dly_o     = THR2L[15:8]     (THR2 deleted)          code_o = CODE[11:0]
//    pulse_o   = PULSE[15:0]     stuff_o  = STUFF[5:0]
//    crcpoly_o = CRCPOLY[31:0]   crccfg_o = CRCCFG[6:0]
//    evc_o[16k +: 16] = EVCk[15:0] (raw; trigger 5-7 / B-must 3 are stored
//                       as written, the event block treats them as off,
//                       ruling 21)
//
//  Contents
//   [W] a_watch       THE main property. A symbolic stored bit w (anyconst,
//                     any of the 198) is compared in EVERY cycle, reset cycles
//                     included, with a one-flop shadow holding the last value
//                     written to it since reset (0 if none). One assertion
//                     covers every bit of every register: reset value, data
//                     bit position, register number decode, half decode,
//                     run_i / cfg_we_i gating, writes to 14..31, the
//                     one-cycle write latency, and hold while halted.
//   [R] a_rst_*       every register is 0 in a reset cycle and in the first
//                     cycle after reset release (a write in a reset cycle is
//                     lost), stated per register on the ports.
//   [N] a_nx_*        next-state relation per register: out(t) =
//                     f(out(t-1), inputs(t-1)) whenever rst_n is 1 in t-1
//                     and t (no write -> unchanged; write -> exact bits).
//                     Redundant with [W]+[R]; kept for direct diagnostics.
//   [P] a_p_*         exact value of every pulse in every cycle with
//                     rst_n = 1 (a_p_*), and in reset cycles too (a_pr_*;
//                     reading S1 below).
//   [C] a_ct_*        consumer contracts (events.md 131, stuff.md, crc.md,
//                     tick.md 349): a register with a side-effect pulse
//                     changes only at the end of a cycle in which that pulse
//                     is 1 (or by reset); PERIOD, PERIOD2, THR0, delay and
//                     PULSE change only after a low write to their number.
//   cfg_bus_props     (separate module, two DUT copies in cfg_fv.sv):
//                     lane.md property 3, shared-bus lemma: cfg_reg_i /
//                     cfg_hi_i / cfg_data_i are ignored in cycles with
//                     cfg_we_i = 0 (every output equal in every cycle).
//   Covers (c_*): every assumption; a low and a high write to each of
//   registers 0..13 and the visible effect of each low write and of the
//   CRCPOLY high write; each pulse (wr_crc from all three sources); writes
//   to 14..31 and without run_i; back-to-back writes; all registers nonzero
//   at once; hold while halted; reset in the middle of operation.
//
//  Environment (STANDALONE = 1: assumed; 0: asserted)
//   env_rst          rst_n = 0 in the first cycle (flops start arbitrary);
//                    rst_n is free afterwards (resets mid-run are checked).
//   every other input is free in every cycle. The sequencer guarantees of
//   lane.md 2.1 (no strobe while run_i = 0, cfg_reg_i <= 13) are NOT
//   assumed: lane.md 2.1 says the lane behaves as specified without them,
//   and 5.1/5.2 define the behaviour (no write, no pulse). Reading S2 lets
//   the cfg_reg_i <= 13 guarantee be assumed instead (define
//   CFG_ASSUME_REG_LE13).
//   checker-internal: w (anyconst) < 198.
//
//  Readings (also in the report; each is one define or one line here)
//   S1  Pulses are "functions of cfg_we_i, cfg_reg_i, cfg_hi_i and run_i"
//       (5.2), so they are checked in reset cycles too (a_pr_*). Define
//       CFG_PULSE_RST_FREE to leave them unchecked while rst_n = 0.
//   S2  Writes to register numbers 14..31 change nothing and raise no pulse
//       (no such register exists; 2.1 says the lane behaves as specified
//       when the sequencer guarantee cfg_reg_i <= 13 is violated). Define
//       CFG_ASSUME_REG_LE13 to assume the guarantee instead.
//   S3  SETM halves follow lane.md 5.1 / model D42 (decisions.md still lists
//       "SETM halves" as to be ruled): a high write changes only
//       CRCPOLY[31:16]; on other registers it does nothing.
//   S4  wr_crc_o is the cycle-t pulse only; the second init pulse in t+1 is
//       the lane top's crcw_q (lane.md 3, 5.4), not a port of this block.
//   S5  Reset is asynchronous (conventions.md): registers read 0 in the
//       reset cycle itself.
//
//  Diagnostic defines (for mutation runs only; the default job uses none):
//   CFG_NO_GRP_W / _R / _N / _C turn off group [W] / [R] / [N] / [C];
//   FV_NO_BUS (cfg_fv.sv) drops the second copy and cfg_bus_props.
// ===========================================================================

module cfg_props #(
    // 1: stand-alone proof (environment assumed, covers active).
    // 0: monitor inside a larger proof (environment asserted, covers off).
    parameter STANDALONE = 1
) (
    input wire         clk,
    input wire         rst_n,
    input wire         run_i,
    input wire         cfg_we_i,
    input wire [4:0]   cfg_reg_i,
    input wire         cfg_hi_i,
    input wire [15:0]  cfg_data_i,
    input wire [12:0]  tick_o,
    input wire [15:0]  period_o,
    input wire [15:0]  period2_o,
    input wire [7:0]   thr0_o,
    input wire [7:0]   dly_o,
    input wire [11:0]  code_o,
    input wire [15:0]  pulse_o,
    input wire [5:0]   stuff_o,
    input wire [31:0]  crcpoly_o,
    input wire [6:0]   crccfg_o,
    input wire [63:0]  evc_o,
    input wire         wr_tick_o,
    input wire         wr_code_o,
    input wire         wr_stuff_o,
    input wire         wr_crc_o,
    input wire [3:0]   wr_evc_o
);
    localparam integer NB = 198;          // stored bits (lane.md table 5.1)

    // =====================================================================
    // Spec decode (lane.md 5.1, 5.2)
    // =====================================================================
    wire wlo = run_i && cfg_we_i && !cfg_hi_i;     // a low-half write
    wire whi = run_i && cfg_we_i &&  cfg_hi_i;     // a high-half write
    logic [13:0] lo_r;                             // low write to r
    always @* begin
        for (int r = 0; r < 14; r++) lo_r[r] = wlo && (cfg_reg_i == 5'(r));
    end
    wire hi8 = whi && (cfg_reg_i == 5'd8);         // high write to CRCPOLY

    wire       x_wr_tick  = lo_r[0];
    wire       x_wr_code  = lo_r[5];
    wire       x_wr_stuff = lo_r[7];
    wire       x_wr_crc   = lo_r[8] || hi8 || lo_r[9];
    wire [3:0] x_wr_evc   = lo_r[13:10];

    // All stored bits as one vector, register 0 at the bottom.
    //   [12:0] TICK  [28:13] PERIOD  [44:29] PERIOD2  [52:45] THR0
    //   [60:53] delay  [72:61] CODE  [88:73] PULSE  [94:89] STUFF
    //   [126:95] CRCPOLY  [133:127] CRCCFG  [197:134] EVC0..EVC3
    wire [NB-1:0] obs = {evc_o, crccfg_o, crcpoly_o, stuff_o, pulse_o,
                         code_o, dly_o, thr0_o, period2_o, period_o, tick_o};

    // Spec next state from the current outputs and inputs.
    wire [12:0] n_tick    = lo_r[0] ? cfg_data_i[12:0]  : tick_o;
    wire [15:0] n_period  = lo_r[1] ? cfg_data_i        : period_o;
    wire [15:0] n_period2 = lo_r[2] ? cfg_data_i        : period2_o;
    wire [7:0]  n_thr0    = lo_r[3] ? cfg_data_i[7:0]   : thr0_o;
    wire [7:0]  n_dly     = lo_r[4] ? cfg_data_i[15:8]  : dly_o;
    wire [11:0] n_code    = lo_r[5] ? cfg_data_i[11:0]  : code_o;
    wire [15:0] n_pulse   = lo_r[6] ? cfg_data_i        : pulse_o;
    wire [5:0]  n_stuff   = lo_r[7] ? cfg_data_i[5:0]   : stuff_o;
    wire [31:0] n_crcpoly = {hi8     ? cfg_data_i : crcpoly_o[31:16],
                             lo_r[8] ? cfg_data_i : crcpoly_o[15:0]};
    wire [6:0]  n_crccfg  = lo_r[9] ? cfg_data_i[6:0]   : crccfg_o;
    wire [63:0] n_evc     = {lo_r[13] ? cfg_data_i : evc_o[63:48],
                             lo_r[12] ? cfg_data_i : evc_o[47:32],
                             lo_r[11] ? cfg_data_i : evc_o[31:16],
                             lo_r[10] ? cfg_data_i : evc_o[15:0]};

    // =====================================================================
    // Checker state
    // =====================================================================
    // rv1 = rst_n in this cycle and in the previous one.
    // rl  = with rst_n = 1: rst_n was 0 in the previous cycle (first cycle
    //       after release). Both are 0 / 1 respectively in reset cycles.
    logic rv1, rl;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rv1 <= 1'b0;
            rl  <= 1'b1;
        end else begin
            rv1 <= 1'b1;
            rl  <= 1'b0;
        end
    end
    wire rel = rst_n && rl;

    // Previous-cycle samples (only read when rv1 = 1, so no reset needed).
    logic [12:0] e_tick;    logic [15:0] e_period, e_period2, e_pulse;
    logic [7:0]  e_thr0, e_dly;  logic [11:0] e_code;  logic [5:0] e_stuff;
    logic [31:0] e_crcpoly; logic [6:0]  e_crccfg; logic [63:0] e_evc;
    logic [NB-1:0] p_obs;
    logic [13:0] p_lo_r;
    logic        p_hi8, p_whi, p_rst_n, p_run, p_we;
    logic [4:0]  p_reg;
    logic [15:0] p_data;
    logic        p_wr_tick, p_wr_code, p_wr_stuff, p_wr_crc;
    logic [3:0]  p_wr_evc;
    always_ff @(posedge clk) begin
        e_tick    <= n_tick;    e_period  <= n_period;  e_period2 <= n_period2;
        e_thr0    <= n_thr0;    e_dly     <= n_dly;     e_code    <= n_code;
        e_pulse   <= n_pulse;   e_stuff   <= n_stuff;   e_crcpoly <= n_crcpoly;
        e_crccfg  <= n_crccfg;  e_evc     <= n_evc;
        p_obs     <= obs;
        p_lo_r    <= lo_r;
        p_hi8     <= hi8;
        p_whi     <= whi;
        p_rst_n   <= rst_n;
        p_run     <= run_i;
        p_we      <= cfg_we_i;
        p_reg     <= cfg_reg_i;
        p_data    <= cfg_data_i;
        p_wr_tick <= wr_tick_o;  p_wr_code <= wr_code_o;
        p_wr_stuff<= wr_stuff_o; p_wr_crc  <= wr_crc_o;
        p_wr_evc  <= wr_evc_o;
    end

    // ---------------------------------------------------------------------
    // [W] symbolic watched bit
    // ---------------------------------------------------------------------
    (* anyconst *) logic [7:0] w;          // obs index of the watched bit
    logic [4:0] w_reg;                      // its register number
    logic       w_hi;                       // its half (1 = bits [31:16])
    logic [3:0] w_db;                       // the cfg_data_i bit that feeds it
    logic [7:0] w_j;
    always @* begin
        w_reg = 5'd0; w_hi = 1'b0; w_j = 8'd0;
        if      (w < 8'd13)  begin w_reg = 5'd0;  w_j = w;                end
        else if (w < 8'd29)  begin w_reg = 5'd1;  w_j = w - 8'd13;        end
        else if (w < 8'd45)  begin w_reg = 5'd2;  w_j = w - 8'd29;        end
        else if (w < 8'd53)  begin w_reg = 5'd3;  w_j = w - 8'd45;        end
        else if (w < 8'd61)  begin w_reg = 5'd4;  w_j = w - 8'd53 + 8'd8; end
        else if (w < 8'd73)  begin w_reg = 5'd5;  w_j = w - 8'd61;        end
        else if (w < 8'd89)  begin w_reg = 5'd6;  w_j = w - 8'd73;        end
        else if (w < 8'd95)  begin w_reg = 5'd7;  w_j = w - 8'd89;        end
        else if (w < 8'd111) begin w_reg = 5'd8;  w_j = w - 8'd95;        end
        else if (w < 8'd127) begin w_reg = 5'd8;  w_j = w - 8'd111; w_hi = 1'b1; end
        else if (w < 8'd134) begin w_reg = 5'd9;  w_j = w - 8'd127;       end
        else begin
            w_j   = w - 8'd134;
            w_reg = 5'd10 + {3'd0, w_j[5:4]};
        end
        w_db = w_j[3:0];
    end
    wire w_hit = run_i && cfg_we_i && (cfg_reg_i == w_reg) && (cfg_hi_i == w_hi);
    logic sh;                               // last value written since reset
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)     sh <= 1'b0;
        else if (w_hit) sh <= cfg_data_i[w_db];
    end
    // zero-extended to 256 so the variable index never leaves the vector
    // (no x bits in the model; w < 198 by env_w_range anyway)
    wire [255:0] obs_x   = {{(256-NB){1'b0}}, obs};
    wire [255:0] p_obs_x = {{(256-NB){1'b0}}, p_obs};
    wire w_obs   = obs_x[w];
    wire w_p_obs = p_obs_x[w];

    // =====================================================================
    // Environment
    // =====================================================================
    generate
        if (STANDALONE) begin : g_env_assume
            always @* begin
                if ($initstate) env_rst: assume (!rst_n);
`ifdef CFG_ASSUME_REG_LE13
                env_reg_le13: assume (cfg_reg_i <= 5'd13);
`endif
            end
        end else begin : g_env_assert
            always @* begin
                if ($initstate) env_rst: assert (!rst_n);
`ifdef CFG_ASSUME_REG_LE13
                env_reg_le13: assert (cfg_reg_i <= 5'd13);
`endif
            end
        end
    endgenerate
    always @* env_w_range: assume (w < 8'(NB));

    // =====================================================================
    // Assertions
    // =====================================================================
`ifdef CFG_ASSUME_REG_LE13
    localparam bit CHK_GT13 = 1'b0;
`else
    localparam bit CHK_GT13 = 1'b1;
`endif
`ifdef CFG_PULSE_RST_FREE
    localparam bit CHK_PRST = 1'b0;
`else
    localparam bit CHK_PRST = 1'b1;
`endif

    always @* begin
        // ------------------------------------------------ [W] main property
`ifndef CFG_NO_GRP_W
        a_watch: assert (w_obs == sh);
`endif

        // ------------------------------------------------ [R] reset values
`ifndef CFG_NO_GRP_R
        if (!rst_n || rel) begin
            a_rst_tick:    assert (tick_o    == 13'd0);
            a_rst_period:  assert (period_o  == 16'd0);
            a_rst_period2: assert (period2_o == 16'd0);
            a_rst_thr0:    assert (thr0_o    == 8'd0);
            a_rst_dly:     assert (dly_o     == 8'd0);
            a_rst_code:    assert (code_o    == 12'd0);
            a_rst_pulse:   assert (pulse_o   == 16'd0);
            a_rst_stuff:   assert (stuff_o   == 6'd0);
            a_rst_crcpoly: assert (crcpoly_o == 32'd0);
            a_rst_crccfg:  assert (crccfg_o  == 7'd0);
            a_rst_evc:     assert (evc_o     == 64'd0);
        end
`endif

        // ------------------------------------------------ [N] next state
`ifndef CFG_NO_GRP_N
        if (rv1) begin
            a_nx_tick:       assert (tick_o            == e_tick);
            a_nx_period:     assert (period_o          == e_period);
            a_nx_period2:    assert (period2_o         == e_period2);
            a_nx_thr0:       assert (thr0_o            == e_thr0);
            a_nx_dly:        assert (dly_o             == e_dly);
            a_nx_code:       assert (code_o            == e_code);
            a_nx_pulse:      assert (pulse_o           == e_pulse);
            a_nx_stuff:      assert (stuff_o           == e_stuff);
            a_nx_crcpoly_lo: assert (crcpoly_o[15:0]   == e_crcpoly[15:0]);
            a_nx_crcpoly_hi: assert (crcpoly_o[31:16]  == e_crcpoly[31:16]);
            a_nx_crccfg:     assert (crccfg_o          == e_crccfg);
            a_nx_evc0:       assert (evc_o[15:0]       == e_evc[15:0]);
            a_nx_evc1:       assert (evc_o[31:16]      == e_evc[31:16]);
            a_nx_evc2:       assert (evc_o[47:32]      == e_evc[47:32]);
            a_nx_evc3:       assert (evc_o[63:48]      == e_evc[63:48]);
            // nothing changes without a write strobe, with run_i = 0, or for
            // a register number 14..31 / a high half other than CRCPOLY
            if (!(p_run && p_we))
                a_nx_nowrite: assert (obs == p_obs);
            if (CHK_GT13 && p_reg > 5'd13)
                a_nx_unimpl:  assert (obs == p_obs);
            if (p_whi && p_reg != 5'd8)
                a_nx_hi_noop: assert (obs == p_obs);
        end
`endif

        // ------------------------------------------------ [P] pulses
        if (rst_n) begin
            a_p_wr_tick:  assert (wr_tick_o  == x_wr_tick);
            a_p_wr_code:  assert (wr_code_o  == x_wr_code);
            a_p_wr_stuff: assert (wr_stuff_o == x_wr_stuff);
            a_p_wr_crc:   assert (wr_crc_o   == x_wr_crc);
            a_p_wr_evc:   assert (wr_evc_o   == x_wr_evc);
        end else if (CHK_PRST) begin
            a_pr_wr_tick:  assert (wr_tick_o  == x_wr_tick);
            a_pr_wr_code:  assert (wr_code_o  == x_wr_code);
            a_pr_wr_stuff: assert (wr_stuff_o == x_wr_stuff);
            a_pr_wr_crc:   assert (wr_crc_o   == x_wr_crc);
            a_pr_wr_evc:   assert (wr_evc_o   == x_wr_evc);
        end

        // ------------------------------------------------ [C] contracts
`ifndef CFG_NO_GRP_C
        if (rv1) begin
            if (tick_o != p_obs[12:0])         a_ct_tick:    assert (p_wr_tick);
            if (code_o != p_obs[72:61])        a_ct_code:    assert (p_wr_code);
            if (stuff_o != p_obs[94:89])       a_ct_stuff:   assert (p_wr_stuff);
            if (crcpoly_o != p_obs[126:95])    a_ct_crcpoly: assert (p_wr_crc);
            if (crccfg_o != p_obs[133:127])    a_ct_crccfg:  assert (p_wr_crc);
            if (evc_o[15:0]  != p_obs[149:134]) a_ct_evc0:   assert (p_wr_evc[0]);
            if (evc_o[31:16] != p_obs[165:150]) a_ct_evc1:   assert (p_wr_evc[1]);
            if (evc_o[47:32] != p_obs[181:166]) a_ct_evc2:   assert (p_wr_evc[2]);
            if (evc_o[63:48] != p_obs[197:182]) a_ct_evc3:   assert (p_wr_evc[3]);
            if (period_o  != p_obs[28:13])     a_ct_period:  assert (p_lo_r[1]);
            if (period2_o != p_obs[44:29])     a_ct_period2: assert (p_lo_r[2]);
            if (thr0_o    != p_obs[52:45])     a_ct_thr0:    assert (p_lo_r[3]);
            if (dly_o     != p_obs[60:53])     a_ct_dly:     assert (p_lo_r[4]);
            if (pulse_o   != p_obs[88:73])     a_ct_pulse:   assert (p_lo_r[6]);
        end
`endif
    end

    // =====================================================================
    // Covers
    // =====================================================================
    generate
        if (STANDALONE) begin : g_cov
            // register r's output at [16r +: 16] (now and one cycle earlier)
            wire [14*16-1:0] cur = {
                evc_o[63:48], evc_o[47:32], evc_o[31:16], evc_o[15:0],
                {9'd0, crccfg_o}, crcpoly_o[15:0], {10'd0, stuff_o}, pulse_o,
                {4'd0, code_o}, {dly_o, 8'd0}, {8'd0, thr0_o}, period2_o,
                period_o, {3'd0, tick_o}};
            wire [14*16-1:0] prv = {
                p_obs[197:182], p_obs[181:166], p_obs[165:150], p_obs[149:134],
                {9'd0, p_obs[133:127]}, p_obs[110:95], {10'd0, p_obs[94:89]},
                p_obs[88:73], {4'd0, p_obs[72:61]}, {p_obs[60:53], 8'd0},
                {8'd0, p_obs[52:45]}, p_obs[44:29], p_obs[28:13],
                {3'd0, p_obs[12:0]}};
            // per register r = 0..13:
            //  c_wlo_r  a low write to r;  c_whi_r  a high write to r
            //  c_chg_r  the low write's visible effect in the next cycle
            //  c_hid_r  a high write to r whose data differs from r's
            //           (low half of) value (only CRCPOLY changes;
            //           asserted elsewhere)
            always @* begin
                if (rst_n) c_wlo_0: cover (lo_r[0]);
                if (rst_n) c_whi_0: cover (whi && cfg_reg_i == 5'd0);
                if (rv1)   c_chg_0: cover (p_lo_r[0] && cur[0 +: 16] != prv[0 +: 16]);
                if (rv1)   c_hid_0: cover (p_whi && p_reg == 5'd0 && p_data != cur[0 +: 16]);
                if (rst_n) c_wlo_1: cover (lo_r[1]);
                if (rst_n) c_whi_1: cover (whi && cfg_reg_i == 5'd1);
                if (rv1)   c_chg_1: cover (p_lo_r[1] && cur[16 +: 16] != prv[16 +: 16]);
                if (rv1)   c_hid_1: cover (p_whi && p_reg == 5'd1 && p_data != cur[16 +: 16]);
                if (rst_n) c_wlo_2: cover (lo_r[2]);
                if (rst_n) c_whi_2: cover (whi && cfg_reg_i == 5'd2);
                if (rv1)   c_chg_2: cover (p_lo_r[2] && cur[32 +: 16] != prv[32 +: 16]);
                if (rv1)   c_hid_2: cover (p_whi && p_reg == 5'd2 && p_data != cur[32 +: 16]);
                if (rst_n) c_wlo_3: cover (lo_r[3]);
                if (rst_n) c_whi_3: cover (whi && cfg_reg_i == 5'd3);
                if (rv1)   c_chg_3: cover (p_lo_r[3] && cur[48 +: 16] != prv[48 +: 16]);
                if (rv1)   c_hid_3: cover (p_whi && p_reg == 5'd3 && p_data != cur[48 +: 16]);
                if (rst_n) c_wlo_4: cover (lo_r[4]);
                if (rst_n) c_whi_4: cover (whi && cfg_reg_i == 5'd4);
                if (rv1)   c_chg_4: cover (p_lo_r[4] && cur[64 +: 16] != prv[64 +: 16]);
                if (rv1)   c_hid_4: cover (p_whi && p_reg == 5'd4 && p_data != cur[64 +: 16]);
                if (rst_n) c_wlo_5: cover (lo_r[5]);
                if (rst_n) c_whi_5: cover (whi && cfg_reg_i == 5'd5);
                if (rv1)   c_chg_5: cover (p_lo_r[5] && cur[80 +: 16] != prv[80 +: 16]);
                if (rv1)   c_hid_5: cover (p_whi && p_reg == 5'd5 && p_data != cur[80 +: 16]);
                if (rst_n) c_wlo_6: cover (lo_r[6]);
                if (rst_n) c_whi_6: cover (whi && cfg_reg_i == 5'd6);
                if (rv1)   c_chg_6: cover (p_lo_r[6] && cur[96 +: 16] != prv[96 +: 16]);
                if (rv1)   c_hid_6: cover (p_whi && p_reg == 5'd6 && p_data != cur[96 +: 16]);
                if (rst_n) c_wlo_7: cover (lo_r[7]);
                if (rst_n) c_whi_7: cover (whi && cfg_reg_i == 5'd7);
                if (rv1)   c_chg_7: cover (p_lo_r[7] && cur[112 +: 16] != prv[112 +: 16]);
                if (rv1)   c_hid_7: cover (p_whi && p_reg == 5'd7 && p_data != cur[112 +: 16]);
                if (rst_n) c_wlo_8: cover (lo_r[8]);
                if (rst_n) c_whi_8: cover (whi && cfg_reg_i == 5'd8);
                if (rv1)   c_chg_8: cover (p_lo_r[8] && cur[128 +: 16] != prv[128 +: 16]);
                if (rv1)   c_hid_8: cover (p_whi && p_reg == 5'd8 && p_data != cur[128 +: 16]);
                if (rst_n) c_wlo_9: cover (lo_r[9]);
                if (rst_n) c_whi_9: cover (whi && cfg_reg_i == 5'd9);
                if (rv1)   c_chg_9: cover (p_lo_r[9] && cur[144 +: 16] != prv[144 +: 16]);
                if (rv1)   c_hid_9: cover (p_whi && p_reg == 5'd9 && p_data != cur[144 +: 16]);
                if (rst_n) c_wlo_10: cover (lo_r[10]);
                if (rst_n) c_whi_10: cover (whi && cfg_reg_i == 5'd10);
                if (rv1)   c_chg_10: cover (p_lo_r[10] && cur[160 +: 16] != prv[160 +: 16]);
                if (rv1)   c_hid_10: cover (p_whi && p_reg == 5'd10 && p_data != cur[160 +: 16]);
                if (rst_n) c_wlo_11: cover (lo_r[11]);
                if (rst_n) c_whi_11: cover (whi && cfg_reg_i == 5'd11);
                if (rv1)   c_chg_11: cover (p_lo_r[11] && cur[176 +: 16] != prv[176 +: 16]);
                if (rv1)   c_hid_11: cover (p_whi && p_reg == 5'd11 && p_data != cur[176 +: 16]);
                if (rst_n) c_wlo_12: cover (lo_r[12]);
                if (rst_n) c_whi_12: cover (whi && cfg_reg_i == 5'd12);
                if (rv1)   c_chg_12: cover (p_lo_r[12] && cur[192 +: 16] != prv[192 +: 16]);
                if (rv1)   c_hid_12: cover (p_whi && p_reg == 5'd12 && p_data != cur[192 +: 16]);
                if (rst_n) c_wlo_13: cover (lo_r[13]);
                if (rst_n) c_whi_13: cover (whi && cfg_reg_i == 5'd13);
                if (rv1)   c_chg_13: cover (p_lo_r[13] && cur[208 +: 16] != prv[208 +: 16]);
                if (rv1)   c_hid_13: cover (p_whi && p_reg == 5'd13 && p_data != cur[208 +: 16]);
            end

            always @* begin
                // environment: reset release reached, mid-run reset
                c_rst_release: cover (rel);
                // (p_* are plain flops: valid from step 1, and p_rst_n = 1
                //  needs step >= 2 because rst_n = 0 in step 0)
                if (!$initstate)
                    c_mid_reset: cover (!rst_n && p_rst_n && p_obs != '0);
                // watched bit: set by a write, at both ends of its range
                if (rst_n) c_watch_set: cover (sh);
                if (rst_n) c_watch_lo:  cover (sh && w == 8'd0);
                if (rst_n) c_watch_hi:  cover (sh && w == 8'(NB - 1));
                if (rv1)   c_watch_clr: cover (!sh && w_obs == 1'b0 && w_p_obs);
                // CRCPOLY upper half changed by a high write
                if (rv1)   c_chg_crcpoly_hi: cover (p_hi8 && crcpoly_o[31:16] != p_obs[126:111]);
                // every pulse, wr_crc from each of its three sources
                if (rst_n) begin
                    c_wr_tick:   cover (wr_tick_o);
                    c_wr_code:   cover (wr_code_o);
                    c_wr_stuff:  cover (wr_stuff_o);
                    c_wr_crc_l8: cover (wr_crc_o && cfg_reg_i == 5'd8 && !cfg_hi_i);
                    c_wr_crc_h8: cover (wr_crc_o && cfg_reg_i == 5'd8 &&  cfg_hi_i);
                    c_wr_crc_9:  cover (wr_crc_o && cfg_reg_i == 5'd9);
                    c_wr_evc0:   cover (wr_evc_o[0]);
                    c_wr_evc1:   cover (wr_evc_o[1]);
                    c_wr_evc2:   cover (wr_evc_o[2]);
                    c_wr_evc3:   cover (wr_evc_o[3]);
                    // writes that must do nothing
`ifndef CFG_ASSUME_REG_LE13
                    c_unimpl:    cover (run_i && cfg_we_i && cfg_reg_i > 5'd13);
`endif
                    c_norun:     cover (!run_i && cfg_we_i && cfg_reg_i <= 5'd13);
                    c_reg13:     cover (lo_r[13]);
                end
                if (!rst_n) c_wr_in_rst: cover (run_i && cfg_we_i);
                // back-to-back writes to one register, both visible
                if (rv1) c_b2b: cover (p_lo_r[1] && lo_r[1] && cfg_data_i != p_data &&
                                       period_o == p_data);
                // registers held while halted
                if (rv1) c_halt_hold: cover (!run_i && !p_run && obs != '0);
                // every register nonzero at the same time (no clobbering)
                if (rst_n) c_all_set: cover (tick_o != 0 && period_o != 0 &&
                    period2_o != 0 && thr0_o != 0 && dly_o != 0 && code_o != 0 &&
                    pulse_o != 0 && stuff_o != 0 && crcpoly_o[15:0] != 0 &&
                    crcpoly_o[31:16] != 0 && crccfg_o != 0 && evc_o[15:0] != 0 &&
                    evc_o[31:16] != 0 && evc_o[47:32] != 0 && evc_o[63:48] != 0);
            end
        end
    endgenerate
endmodule

// ===========================================================================
//  lane.md section 9 property 3 (shared-bus lemma) for this block: two
//  copies with equal clk, rst_n, run_i, cfg_we_i, whose cfg_reg_i /
//  cfg_hi_i / cfg_data_i are equal in every cycle with cfg_we_i = 1 and free
//  otherwise (the driving is in cfg_fv.sv), have equal outputs in every
//  cycle.
// ===========================================================================
module cfg_bus_props (
    input wire [197:0] a_obs,
    input wire [7:0]   a_pulse,
    input wire [197:0] b_obs,
    input wire [7:0]   b_pulse
);
    always @* begin
        a_bus_regs:   assert (a_obs   == b_obs);
        a_bus_pulses: assert (a_pulse == b_pulse);
    end
endmodule
