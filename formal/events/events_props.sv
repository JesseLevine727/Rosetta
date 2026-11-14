`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_events (the lane's four event detectors).
//
//  Written from docs/blocks/events.md ONLY (the RTL was not read), with
//  docs/decisions.md rulings 10 and 21 and the notation of lane.md 1.
//  Every property is about the block's ports.
//
//  Contents
//   * Executable model of section 3 (per detector win, cnt, done, lv; the
//     pending bits), driven from the input ports. fire_o and ev_pend_o are
//     compared with it in every cycle (a_m_fire, a_m_pend).
//   * The required properties of section 4 stated again on the ports,
//     without the model, from their own text (a trigger / episode start
//     t0, every cycle of t0+1..u checked, u = t0 (+) K found by counting
//     the strobes still needed):
//       1 a_edge_k0, a_edge_exact      2 a_lvl_exact     3 a_lvl_once
//       4 a_off                        5 a_pend_*        6 a_indep_* (in
//       7 a_stuff_e3, a_stuff_cfg        events_fv.sv: two copies)
//     plus a_wr_edge, a_wr_lvl (3.3 restart on a write), a_halt_fire,
//     a_rst_* and the worked I2C case of section 5 (a_i2c_true_start,
//     a_i2c_false_start).
//   * Environment rules (env_*): reset in the first cycle; cfg_evc_i[k]
//     changes only at the end of a cycle with evc_wr_i[k], run_i = 0 or
//     reset (section 4, "Assumptions"). Assumed (STANDALONE = 1) or asserted
//     (STANDALONE = 0, monitor inside a lane-level proof). Everything else
//     is free: in_q_i is NOT tied to the previous in_i, pt_i is any
//     pattern, stuff_viol_i need not be a one-cycle pulse, ev_clr_i and
//     evc_wr_i any value in any cycle. This is stronger than the lane.
//   * Covers (STANDALONE = 1 and COVERS = 1).
//
//  Assertions inside generate loops carry their name as a trailing
//  `// @name` comment (Yosys does not scope labels per loop iteration);
//  ref/run_mutants.sh reports failures by that name.
//
//  Readings (see the report):
//   * fire_o is 0 in every cycle with run_i = 0 (3.1/3.2 apply "with
//     run_i"; 3.4 holds the detectors at 0 while halted).
//   * fire_o is not checked while rst_n = 0 (the spec defines no fire in
//     reset; ev_pend_o is 0 in reset and is checked).
//   * "pend 0 while halted" (4.5) is checked as the 3.4 formula: 0 in every
//     cycle that follows a halted cycle (the first halted cycle still shows
//     the bit registered at the end of the last running cycle: ev_pend_o is
//     a register).
//   * A configuration write in cycle t does not stop a fire in t (3.3) and
//     does not change the pending bit; win/done are cleared at the end of t
//     even if the old configuration would have opened a window in t.
//   * Required properties 1 and 2 are stated with the write and halt rules
//     added (their text omits them): a window needs run_i in t0..u and no
//     write in t0..u-1; an episode also starts in the first running cycle
//     after a write, a halt or a reset.
//   * The configuration rule is assumed as: cfg_evc_i[k] may change at the
//     end of any cycle with evc_wr_i[k], run_i = 0 or rst_n = 0.
//   * Required property 7 is checked for cycles with run_i = 1 (the 3.4
//     formula gates every pending bit with run_i).
//   * Independence (events_fv.sv) is checked against changes of both
//     cfg_evc_i[j] and evc_wr_i[j], j != k (the spec names cfg only).
// ===========================================================================

module events_props #(
    // 1: stand-alone proof (environment rules are assumed).
    // 0: monitor inside a larger proof (environment rules are asserted,
    //    covers off).
    parameter STANDALONE = 1,
    // 1: covers active (STANDALONE only). 0 in the second copy of the
    //    independence proof.
    parameter COVERS     = 1
) (
    input wire        clk,
    input wire        rst_n,
    input wire        run_i,
    input wire [3:0]  in_i,
    input wire [3:0]  in_q_i,
    input wire        pt_i,
    input wire [63:0] cfg_evc_i,
    input wire [3:0]  evc_wr_i,
    input wire        cfg_stuffev_i,
    input wire        stuff_viol_i,
    input wire [3:0]  ev_clr_i,
    input wire [3:0]  ev_pend_o,
    input wire [3:0]  fire_o
);
    genvar g;

    // stuffing violation term of event 3 (3.4)
    wire viol = cfg_stuffev_i && stuff_viol_i;

    // ---------------------------------------------------------------------
    // Environment
    // ---------------------------------------------------------------------
    generate
        if (STANDALONE) begin : g_env_assume
            always @* begin
                if ($initstate) begin
                    env_rst: assume (!rst_n);
                end
            end
            for (g = 0; g < 4; g = g + 1) begin : g_env_cfg
                always @(posedge clk) begin
                    if (!$initstate && $past(rst_n && run_i && !evc_wr_i[g])) begin
                        assume (cfg_evc_i[16*g +: 16] == $past(cfg_evc_i[16*g +: 16]));  // @env_cfg
                    end
                end
            end
        end else begin : g_env_assert
            always @* begin
                if ($initstate) begin
                    env_rst: assert (!rst_n);
                end
            end
            for (g = 0; g < 4; g = g + 1) begin : g_env_cfg
                always @(posedge clk) begin
                    if (!$initstate && $past(rst_n && run_i && !evc_wr_i[g])) begin
                        assert (cfg_evc_i[16*g +: 16] == $past(cfg_evc_i[16*g +: 16]));  // @env_cfg
                    end
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Per-detector decode (section 2) and model of section 3
    // ---------------------------------------------------------------------
    wire [3:0] m_fire;    // model: detector k fires in this cycle
    wire [3:0] d_edge;    // trigger 2 or 3
    wire [3:0] d_lvl;     // trigger 1 or 4
    wire [3:0] d_k0;      // K == 0
    wire [3:0] d_trg;     // edge trigger condition (0 for other kinds)
    wire [3:0] d_cond;    // level condition (0 for other kinds)
    wire [3:0] m_winv;    // model window open (for covers)
    wire [3:0] m_hcv;     // model hold condition (for covers)
    wire [3:0] m_lvv;     // model lv (for covers)
    wire [3:0] d_a, d_aq, d_b, d_L, d_bok;   // per-detector signals (covers)
    wire [3:0] o_firedv, fs_onv, fs_cutv;    // monitor state (covers)

    generate
        for (g = 0; g < 4; g = g + 1) begin : g_det
            wire [15:0] c    = cfg_evc_i[16*g +: 16];
            wire [1:0]  fA   = c[1:0];
            wire        fL   = c[2];
            wire [2:0]  trig = c[5:3];
            wire [1:0]  fB   = c[7:6];
            wire [1:0]  bm   = c[9:8];
            wire [5:0]  K    = c[15:10];

            wire a   = in_i[fA];
            wire aq  = in_q_i[fA];
            wire b   = in_i[fB];
            wire bok = (bm == 2'd0) || (bm == 2'd1 && b) || (bm == 2'd2 && !b);

            wire is_edge = (trig == 3'd2) || (trig == 3'd3);
            wire is_lvl  = (trig == 3'd1) || (trig == 3'd4);
            wire is_off  = !is_edge && !is_lvl;           // 0, 5, 6, 7 (ruling 21)

            // 3.1: trg (trigger 2: to level L; trigger 3: any edge)
            wire trg  = is_edge && (a != aq) && bok && ((trig == 3'd3) || (a == fL));
            // 3.2: cond (trigger 4 ignores L and bm)
            wire cond = (trig == 3'd1) ? ((a == fL) && bok) :
                        (trig == 3'd4) ? (!a && !b) : 1'b0;

            assign d_edge[g] = is_edge;
            assign d_lvl[g]  = is_lvl;
            assign d_k0[g]   = (K == 6'd0);
            assign d_trg[g]  = trg;
            assign d_cond[g] = cond;

            // ---- model state ----
            logic       m_win, m_done, m_lv;
            logic [5:0] m_cnt;
            logic       n_win, n_done, n_lv, f;
            logic [5:0] n_cnt;
            wire hc = ((trig == 3'd3) ? (a == m_lv) : (a == fL)) && bok;

            always @* begin
                f      = 1'b0;
                n_win  = m_win;
                n_done = m_done;
                n_lv   = m_lv;
                n_cnt  = m_cnt;
                if (!run_i) begin
                    // 3.4: held at 0 while halted, no fire
                    n_win  = 1'b0;
                    n_done = 1'b0;
                    n_lv   = 1'b0;
                    n_cnt  = 6'd0;
                end else begin
                    if (is_edge) begin
                        // 3.1
                        if (trg) begin
                            n_lv = a;
                            if (K == 6'd0) begin
                                f     = 1'b1;
                                n_win = 1'b0;
                            end else begin
                                n_win = 1'b1;
                                n_cnt = K;
                            end
                        end else if (m_win) begin
                            if (!hc) begin
                                n_win = 1'b0;
                            end else if (pt_i) begin
                                if (m_cnt == 6'd1) begin
                                    f     = 1'b1;
                                    n_win = 1'b0;
                                end else begin
                                    n_cnt = m_cnt - 6'd1;
                                end
                            end
                        end
                    end else if (is_lvl) begin
                        // 3.2
                        if (!cond) begin
                            n_win  = 1'b0;
                            n_done = 1'b0;
                        end else if (!m_win && !m_done) begin
                            if (K == 6'd0) begin
                                f      = 1'b1;
                                n_done = 1'b1;
                            end else begin
                                n_win = 1'b1;
                                n_cnt = K;
                            end
                        end else if (m_win && pt_i) begin
                            if (m_cnt == 6'd1) begin
                                f      = 1'b1;
                                n_win  = 1'b0;
                                n_done = 1'b1;
                            end else begin
                                n_cnt = m_cnt - 6'd1;
                            end
                        end
                    end
                    // 3.3: a write restarts the detector at the end of t
                    if (evc_wr_i[g]) begin
                        n_win  = 1'b0;
                        n_done = 1'b0;
                    end
                end
            end

            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    m_win  <= 1'b0;
                    m_done <= 1'b0;
                    m_lv   <= 1'b0;
                    m_cnt  <= 6'd0;
                end else begin
                    m_win  <= n_win;
                    m_done <= n_done;
                    m_lv   <= n_lv;
                    m_cnt  <= n_cnt;
                end
            end
            assign m_fire[g] = f;
            assign m_winv[g] = m_win;
            assign m_hcv[g]  = hc;
            assign m_lvv[g]  = m_lv;
            assign d_a[g]    = a;
            assign d_aq[g]   = aq;
            assign d_b[g]    = b;
            assign d_L[g]    = fL;
            assign d_bok[g]  = bok;

            // =============================================================
            // Port-level statements (no model)
            // =============================================================

            // ---- Required property 1, edge triggers ----
            // e_ok: there is a trigger t0 < t with run_i in t0..t-1, no write
            // in t0..t-1, and in t0+1..t-1 no other trigger and hc held.
            // e_r: strobes still needed, K minus the number of cycles with
            // pt_i in t0+1..t-1 (saturating at 0). So u = t0 (+) K is the
            // cycle with pt_i and e_r == 1.
            // e_lv: in_i[A] at t0 (trigger 3's lv).
            logic       e_ok, e_lv;
            logic [5:0] e_r;
            wire        e_hc   = ((trig == 3'd3) ? (a == e_lv) : (a == fL)) && bok;
            wire        e_cand = e_ok && run_i && !trg && e_hc && pt_i && (e_r == 6'd1);
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    e_ok <= 1'b0;
                    e_lv <= 1'b0;
                    e_r  <= 6'd0;
                end else if (!run_i || evc_wr_i[g]) begin
                    e_ok <= 1'b0;
                end else if (trg) begin
                    e_ok <= 1'b1;
                    e_lv <= a;
                    e_r  <= K;
                end else if (e_ok) begin
                    if (!e_hc) e_ok <= 1'b0;
                    if (pt_i && e_r != 6'd0) e_r <= e_r - 6'd1;
                end
            end
            always @* begin
                if (rst_n && is_edge && K == 6'd0) begin
                    assert (fire_o[g] == (run_i && trg));  // @a_edge_k0
                end
                if (rst_n && is_edge && K != 6'd0) begin
                    assert (fire_o[g] == e_cand);  // @a_edge_exact
                end
            end

            // ---- Required property 2, level triggers ----
            // l_ep: cycle t-1 had run_i, cond and no write, so t (with cond)
            // continues that episode; otherwise a cond in t starts one.
            // l_r: K minus the pt_i count in t0+1..t-1 of the current
            // episode (saturating at 0).
            logic       l_ep;
            logic [5:0] l_r;
            wire        l_start = run_i && cond && !l_ep;
            wire        l_cand  = run_i && cond &&
                                  ((K == 6'd0) ? l_start
                                               : (l_ep && pt_i && l_r == 6'd1));
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    l_ep <= 1'b0;
                    l_r  <= 6'd0;
                end else begin
                    l_ep <= run_i && cond && !evc_wr_i[g];
                    if (l_start)                  l_r <= K;
                    else if (pt_i && l_r != 6'd0) l_r <= l_r - 6'd1;
                end
            end
            always @* begin
                if (rst_n && is_lvl) begin
                    assert (fire_o[g] == l_cand);  // @a_lvl_exact
                end
            end

            // ---- Required property 3, once per episode ----
            logic o_fired;   // fired since the last !cond / write / halt
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n)                              o_fired <= 1'b0;
                else if (!run_i || !cond || evc_wr_i[g]) o_fired <= 1'b0;
                else if (fire_o[g])                      o_fired <= 1'b1;
            end
            assign o_firedv[g] = o_fired;
            always @* begin
                if (rst_n && is_lvl && o_fired) begin
                    assert (!fire_o[g]);  // @a_lvl_once
                end
            end

            // ---- Required property 4, off codes (ruling 21) ----
            always @* begin
                if (rst_n && (is_off || (bm == 2'd3 && trig != 3'd4))) begin
                    assert (!fire_o[g]);  // @a_off
                end
            end

            // ---- 3.4: no fire while halted (reading) ----
            always @* begin
                if (rst_n && !run_i) begin
                    assert (!fire_o[g]);  // @a_halt_fire
                end
            end

            // ---- 3.3: restart on a configuration write ----
            always @(posedge clk) begin
                if (!$initstate && rst_n && $past(rst_n && run_i && evc_wr_i[g])) begin
                    // an edge window never survives a write
                    if (is_edge && K != 6'd0) begin
                        assert (!fire_o[g]);  // @a_wr_edge
                    end
                    // a level condition true right after a write starts an
                    // episode there (K = 0: fires at once, even if the old
                    // episode had already fired)
                    if (run_i && is_lvl && K == 6'd0 && cond) begin
                        assert (fire_o[g]);  // @a_wr_lvl
                    end
                end
            end

            // ---- Required property 5, pending bit k ----
            wire fk = (g == 3) ? (fire_o[g] || viol) : fire_o[g];
            always @(posedge clk) begin
                if (!$initstate && rst_n) begin
                    // exact next-state formula of 3.4 on the ports
                    assert (ev_pend_o[g] == ($past(rst_n) && $past(run_i) &&  // @a_pend_next
                                             $past(fk || (ev_pend_o[g] && !ev_clr_i[g]))));
                    // a fire is visible on ev_pend_o exactly one cycle later,
                    // a clear in the same cycle notwithstanding
                    if ($past(rst_n && run_i && fire_o[g])) begin
                        assert (ev_pend_o[g]);  // @a_pend_fire
                    end
                    if ($past(rst_n && run_i && fire_o[g] && ev_clr_i[g])) begin
                        assert (ev_pend_o[g]);  // @a_pend_set_wins
                    end
                    // a pending bit rises only after a fire (or, for E3, a
                    // qualified violation)
                    if (ev_pend_o[g] && !$past(ev_pend_o[g])) begin
                        assert ($past(fk));  // @a_pend_rise
                    end
                    // a clear without a fire drops the bit
                    if ($past(rst_n && ev_clr_i[g] && !fk)) begin
                        assert (!ev_pend_o[g]);  // @a_pend_clr
                    end
                    // 0 in the cycle after a halted cycle
                    if (!$past(run_i)) begin
                        assert (!ev_pend_o[g]);  // @a_pend_halt
                    end
                    // the bit is held otherwise (a write does not change it)
                    if ($past(rst_n && run_i && !fk && !ev_clr_i[g])) begin
                        assert (ev_pend_o[g] == $past(ev_pend_o[g]));  // @a_pend_hold
                    end
                end
            end

            // ---- Section 5: the I2C START, on any detector set up as in
            //      the worked example (trigger 2 to low, B must be high) ----
            wire i2c    = (trig == 3'd2) && !fL && (bm == 2'd1);
            wire sda_lo = !in_i[fA] && !in_q_i[fA];   // SDA stays low
            wire scl_hi = in_i[fB];
            // true START: SDA fell with SCL high at t0 (= trg); since then
            // SDA low, SCL high, running, no write. ts_r: K minus the pt
            // count in t0+1..t-1 (saturating at 0).
            logic       ts_ok;
            logic [5:0] ts_r;
            wire        ts_hold = run_i && sda_lo && scl_hi;
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    ts_ok <= 1'b0;
                    ts_r  <= 6'd0;
                end else if (i2c && run_i && trg && !evc_wr_i[g]) begin
                    ts_ok <= 1'b1;
                    ts_r  <= K;
                end else if (!ts_hold || evc_wr_i[g]) begin
                    ts_ok <= 1'b0;
                end else if (pt_i && ts_r != 6'd0) begin
                    ts_r  <= ts_r - 6'd1;
                end
            end
            always @* begin
                if (rst_n && i2c && run_i && trg && K == 6'd0) begin
                    assert (fire_o[g]);  // @a_i2c_true_start
                end
                // fires exactly at t0 (+) K, not before, not again
                if (rst_n && i2c && K != 6'd0 && ts_ok && ts_hold) begin
                    assert (fire_o[g] == (pt_i && ts_r == 6'd1));  // @a_i2c_true_start
                end
            end
            // false START: SDA fell with SCL high at t0 (K > 0), and SCL was
            // low in some cycle s of the window (fewer than K strobes in
            // t0+1..s-1, so s <= t0 (+) K). From s on, until SDA falls again,
            // a write or a halt, the detector never fires.
            // fs_r: K minus the pt count in t0+1..t-1 (saturating at 0):
            // a cycle s is inside the window iff fs_r(s) != 0.
            logic       fs_on, fs_cut;
            logic [5:0] fs_r;
            wire        fs_cut_now = fs_cut || (!scl_hi && fs_r != 6'd0);
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    fs_on  <= 1'b0;
                    fs_cut <= 1'b0;
                    fs_r   <= 6'd0;
                end else if (!run_i || evc_wr_i[g]) begin
                    fs_on  <= 1'b0;
                end else if (trg) begin
                    fs_on  <= i2c && K != 6'd0;
                    fs_cut <= 1'b0;
                    fs_r   <= K;
                end else if (fs_on) begin
                    fs_cut <= fs_cut_now;
                    if (pt_i && fs_r != 6'd0) fs_r <= fs_r - 6'd1;
                end
            end
            assign fs_onv[g]  = fs_on;
            assign fs_cutv[g] = fs_cut;
            always @* begin
                if (rst_n && i2c && fs_on && fs_cut_now) begin
                    assert (!fire_o[g]);  // @a_i2c_false_start
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Model of the pending bits (3.4)
    // ---------------------------------------------------------------------
    wire [3:0] m_f = m_fire | {viol, 3'b000};
    logic [3:0] m_pend;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) m_pend <= 4'h0;
        else        m_pend <= run_i ? (m_f | (m_pend & ~ev_clr_i)) : 4'h0;
    end

    // ---------------------------------------------------------------------
    // Exact behaviour: both outputs equal the model in every cycle
    // (ev_pend_o in reset cycles too).
    // ---------------------------------------------------------------------
`ifndef EVENTS_NO_MODEL_CHECK
    always @* begin
        a_m_pend: assert (ev_pend_o == m_pend);
        if (rst_n) begin
            a_m_fire: assert (fire_o == m_fire);
        end
    end
`endif

    // ---------------------------------------------------------------------
    // Reset (asynchronous) and release; stuffing source (property 7)
    // ---------------------------------------------------------------------
    always @* begin
        if (!rst_n) begin
            a_rst_pend: assert (ev_pend_o == 4'h0);
        end
    end
    always @(posedge clk) begin
        if (!$initstate && rst_n) begin
            if (!$past(rst_n)) begin
                a_rel_pend: assert (ev_pend_o == 4'h0);
            end
            // E3 rises after a qualified violation, whatever EVC3 holds
            if ($past(rst_n && run_i && viol)) begin
                a_stuff_e3: assert (ev_pend_o[3]);
            end
            // never from stuff_viol_i without cfg_stuffev_i; E0-E2 never
            // from a violation
            if (ev_pend_o[3] && !$past(ev_pend_o[3])) begin
                a_stuff_cfg: assert ($past(fire_o[3] || viol));
            end
            if ((ev_pend_o[2:0] & ~$past(ev_pend_o[2:0])) != 3'b000) begin
                a_stuff_only3: assert ((ev_pend_o[2:0] & ~$past(ev_pend_o[2:0]) &
                                        ~$past(fire_o[2:0])) == 3'b000);
            end
        end
    end

    // ---------------------------------------------------------------------
    // Cover helpers
    // ---------------------------------------------------------------------
    // h_*: per detector 0, history used to describe a scenario
    logic [16:0] h_pt;        // pt_i over the last 17 cycles (bit 0 = t-1)
    logic [5:0]  h_age;       // cycles since the last trigger of detector 0
    logic        h_wr_ep;     // detector 0 level: fired, then a write while
                              // cond stayed true (no !cond since the fire)
    logic        h_cut;       // detector 0: a window was cut short
    logic        h_halted;    // a pending bit was set, then run_i dropped
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_pt     <= 17'h0;
            h_age    <= 6'd0;
            h_wr_ep  <= 1'b0;
            h_cut    <= 1'b0;
            h_halted <= 1'b0;
        end else begin
            h_pt  <= {h_pt[15:0], pt_i};
            h_age <= d_trg[0] ? 6'd1 : (h_age == 6'd63 ? 6'd63 : h_age + 6'd1);
            if (!run_i || !d_cond[0])                             h_wr_ep <= 1'b0;
            else if (fire_o[0] && evc_wr_i[0] && d_lvl[0])        h_wr_ep <= 1'b1;
            else if (o_firedv[0] && evc_wr_i[0] && d_lvl[0]) h_wr_ep <= 1'b1;
            if (m_winv[0] && !m_hcv[0] && !d_trg[0])              h_cut <= 1'b1;
            if (!run_i && ev_pend_o != 4'h0)                      h_halted <= 1'b1;
        end
    end

    generate
        if (STANDALONE && COVERS) begin : g_cover
            // detector 0 configuration fields
            wire [2:0] t0  = cfg_evc_i[5:3];
            wire [5:0] K0  = cfg_evc_i[15:10];
            wire [1:0] bm0 = cfg_evc_i[9:8];
            wire       go  = rst_n && run_i;
            always @(posedge clk) begin
                if (!$initstate) begin
                    // environment: configuration changes are reachable both
                    // ways the assumption allows
                    c_env_cfg_wr:   cover (rst_n && $past(rst_n && run_i && evc_wr_i[0]) &&
                                           cfg_evc_i[15:0] != $past(cfg_evc_i[15:0]));
                    c_env_cfg_halt: cover (rst_n && $past(rst_n && !run_i) &&
                                           cfg_evc_i[15:0] != $past(cfg_evc_i[15:0]));
                    // a window cut short (A, B, or the level dropping) and a
                    // fire later in the same trace
                    c_cut_then_fire: cover (go && h_cut && fire_o[0] && !d_k0[0]);
                    // write while a level condition is true: the detector
                    // fires again without the condition ever dropping
                    c_wr_lvl_refire: cover (go && h_wr_ep && fire_o[0] && t0 == 3'd1);
                    // a write cuts an open edge window
                    c_wr_edge_cut:  cover (rst_n && $past(go && evc_wr_i[0] && m_winv[0] &&
                                                         d_edge[0]) && !m_winv[0]);
                    // a write in a fire cycle: the fire still counts
                    c_wr_fire:      cover (go && fire_o[0] && evc_wr_i[0]);
                    // fire racing a clear: the bit stays set
                    c_fire_clr:     cover (rst_n && $past(go && fire_o[0] && ev_clr_i[0] &&
                                                         ev_pend_o[0]) && ev_pend_o[0]);
                    // clear without a fire
                    c_clr:          cover (rst_n && $past(go && ev_clr_i[0] && ev_pend_o[0] &&
                                                         !fire_o[0]) && !ev_pend_o[0]);
                    // back-to-back fires
                    c_b2b:          cover (go && fire_o[0] && $past(go && fire_o[0]));
                    // (K > 0: fires are at least two cycles apart, as a
                    // trigger in the fire cycle restarts the window)
                    c_b2b_k:        cover (go && fire_o[0] && !d_k0[0] && $past(go && !fire_o[0]) &&
                                           $past(go && fire_o[0] && !d_k0[0], 2));
                    // violation on event 3 with EVC3 off; with EVC3 firing
                    c_viol_e3:      cover (rst_n && $past(go && viol && !fire_o[3] && !ev_pend_o[3] &&
                                                         cfg_evc_i[53:51] == 3'd0) && ev_pend_o[3]);
                    c_viol_fire3:   cover (go && viol && fire_o[3]);
                    c_viol_nocfg:   cover (go && stuff_viol_i && !cfg_stuffev_i && !ev_pend_o[3]);
                    // halt clears pending bits; reset in mid-trace
                    c_halt_clear:   cover (rst_n && run_i && h_halted && ev_pend_o == 4'h0);
                    c_rst_mid:      cover (!rst_n && $past(rst_n && ev_pend_o == 4'hF));
                    // prescale-8 window: K = 2, strobes 8 cycles apart, the
                    // trigger 16 cycles before the fire (aligned to a strobe)
                    c_ps8:          cover (go && fire_o[0] && d_edge[0] && K0 == 6'd2 && pt_i &&
                                           h_pt == 17'b0_1000_0000_1000_0000 && h_age == 6'd16);
                    c_ps8_lvl:      cover (go && fire_o[0] && t0 == 3'd1 && K0 == 6'd2 && pt_i &&
                                           h_pt[15:0] == 16'b1000_0000_1000_0000);
                    // all four detectors in one cycle
                    c_all4:         cover (rst_n && fire_o == 4'hF);
                    // all four pending
                    c_pend_all:     cover (rst_n && ev_pend_o == 4'hF);
                    // the worked I2C table: detector 0 as in section 5
                    // (A = 0, L = 0, trig 2, B = 1, bm = 1, K = 4),
                    // prescale 1: fire 4 cycles after the trigger; and a
                    // false START cut one cycle after its trigger
                    c_i2c_true:     cover (go && fire_o[0] && cfg_evc_i[15:0] == {6'd4, 2'd1, 2'd1, 3'd2, 1'b0, 2'd0} &&
                                           h_pt[3:0] == 4'hF && pt_i && h_age == 6'd4 &&
                                           $past(in_i[1:0], 4) == 2'b10 && $past(in_q_i[0], 4));
                    c_i2c_false:    cover (go && cfg_evc_i[15:0] == {6'd4, 2'd1, 2'd1, 3'd2, 1'b0, 2'd0} &&
                                           $past(go && d_trg[0]) && in_i[1:0] == 2'b00 &&
                                           fs_onv[0] && !fs_cutv[0]);
                end
            end
            // per detector: fires, kinds, K extremes, B qualifier, off codes
            for (g = 0; g < 4; g = g + 1) begin : g_cov_det
                wire [15:0] c    = cfg_evc_i[16*g +: 16];
                wire [2:0]  trig = c[5:3];
                wire [1:0]  bm   = c[9:8];
                wire [5:0]  K    = c[15:10];
                wire        gok  = rst_n && run_i;
                always @(posedge clk) begin
                    cover (gok && fire_o[g]);  // c_fire
                    cover (gok && fire_o[g] && trig == 3'd3 && K != 6'd0 && in_i[c[1:0]] == 1'b0);  // c_fire_t3_fall
                end
            end
            // detector 0: each kind with K = 0 and K = 63, B qualifiers,
            // cancellation by A and by B, undefined codes reached
            always @(posedge clk) begin
                c_t1_k0:   cover (go && fire_o[0] && t0 == 3'd1 && K0 == 6'd0);
                c_t2_k0:   cover (go && fire_o[0] && t0 == 3'd2 && K0 == 6'd0);
                c_t3_k0:   cover (go && fire_o[0] && t0 == 3'd3 && K0 == 6'd0);
                c_t4_k0:   cover (go && fire_o[0] && t0 == 3'd4 && K0 == 6'd0);
                c_t1_k63:  cover (go && fire_o[0] && t0 == 3'd1 && K0 == 6'd63);
                c_t2_k63:  cover (go && fire_o[0] && t0 == 3'd2 && K0 == 6'd63);
                c_t3_k63:  cover (go && fire_o[0] && t0 == 3'd3 && K0 == 6'd63);
                c_t4_k63:  cover (go && fire_o[0] && t0 == 3'd4 && K0 == 6'd63);
                c_bm1:     cover (go && fire_o[0] && t0 == 3'd2 && bm0 == 2'd1 && !d_k0[0]);
                c_bm2:     cover (go && fire_o[0] && t0 == 3'd2 && bm0 == 2'd2 && !d_k0[0]);
                c_bm0:     cover (go && fire_o[0] && t0 == 3'd1 && bm0 == 2'd0 && !d_k0[0]);
                c_t4_bm3:  cover (go && fire_o[0] && t0 == 3'd4 && bm0 == 2'd3);
                // window cancelled by A (A off level / off lv, B fine) and
                // by B (A fine, B wrong)
                c_cancel_a_edge: cover (go && m_winv[0] && d_edge[0] && !d_trg[0] && !m_hcv[0] &&
                                        d_bok[0]);
                c_cancel_b_edge: cover (go && m_winv[0] && d_edge[0] && !d_trg[0] && !d_bok[0] &&
                                        ((t0 == 3'd3) ? (d_a[0] == m_lvv[0]) : (d_a[0] == d_L[0])));
                c_cancel_a_t3:   cover (go && m_winv[0] && t0 == 3'd3 && !d_trg[0] && d_bok[0] &&
                                        d_a[0] != m_lvv[0]);
                c_cancel_lvl:    cover (go && m_winv[0] && d_lvl[0] && !d_cond[0]);
                c_cancel_t4:     cover (go && m_winv[0] && t0 == 3'd4 && !d_cond[0]);
                // retrigger restarts an open window, and the restarted
                // window fires
                c_retrig:        cover (go && m_winv[0] && d_trg[0] && !d_k0[0]);
                // undefined codes and bm = 3 with conditions that would
                // fire under some other reading
                c_off5:    cover (go && t0 == 3'd5 && d_a[0] == d_L[0] && d_bok[0]);
                c_off6:    cover (go && t0 == 3'd6 && d_a[0] != d_aq[0]);
                c_off7:    cover (go && t0 == 3'd7 && !d_a[0] && !d_b[0]);
                c_bm3_t2:  cover (go && t0 == 3'd2 && bm0 == 2'd3 && d_a[0] != d_aq[0] &&
                                  d_a[0] == d_L[0]);
            end
        end
    endgenerate

endmodule
