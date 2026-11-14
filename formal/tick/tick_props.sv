`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_tick (tick source).
//
//  Written from docs/blocks/tick.md sections 1-8 and 10-11, docs/blocks/
//  lane.md 1-6 and 12.2, and docs/decisions.md (rulings 2-7, 18, 19, 22, 23,
//  26-29) ONLY. The RTL was not read. Every property is about the ports.
//
//  Contents
//   * Executable model of tick.md 3-7 (prescaler, BAUD timer, EDGE with
//     stretch_hold, RECOVER, PULSE, sample countdown, edge interval counter,
//     operation timer, clock requests) driven from the input ports. Every
//     output is compared with it in EVERY cycle, reset cycles included
//     (a_m_*). cpay_o, creq_dly_o, creq_cap_o are compared only in cycles
//     with creq = 1 (they qualify the request; reading R-A in the report).
//   * The required properties of tick.md 10 restated on the ports with
//     checker-side counters and trackers that see only port values
//     (a_p<N>_*, N = the item number of tick.md 10). No unrolling: spacing
//     is stated as "the strobe count since the last launch/capture tick
//     reaches Pd/P2d in this cycle", which is exact for every prescale.
//   * Environment rules (env_*): assumed (STANDALONE = 1) or asserted
//     (STANDALONE = 0, monitor inside a lane proof).
//   * Covers (STANDALONE = 1 only).
//
//  Proof range parameters (assumed, so a proof states its own range):
//     PBITS  cfg_period_i, cfg_period2_i < 2**PBITS   (16 = full range)
//     OBITS  ot_n_i < 2**OBITS                         (16 = full range)
//
//  Also in this file: tick_worked (the two "Worked timing" tables of
//  tick.md 11 as exact assertions on a scripted instance) and
//  tick_ecnt_props (edge interval counter from a free initial state, to
//  reach the 16-bit saturation).
// ===========================================================================

module tick_props #(
    parameter STANDALONE = 1,
    parameter PBITS      = 16,
    parameter OBITS      = 16
) (
    input wire        clk,
    input wire        rst_n,
    input wire        run_i,
    input wire        abort_i,
    input wire        wr_tick_i,
    input wire [1:0]  cfg_mode_i,
    input wire [1:0]  cfg_ps_i,
    input wire [1:0]  cfg_clkin_i,
    input wire        cfg_capfall_i,
    input wire        cfg_clkout_i,
    input wire        cfg_idle_i,
    input wire        cfg_clkod_i,
    input wire        cfg_rest_i,
    input wire        cfg_honour_i,
    input wire        cfg_hold_i,
    input wire [15:0] cfg_period_i,
    input wire [15:0] cfg_period2_i,
    input wire [7:0]  cfg_thr0_i,
    input wire [7:0]  cfg_pw0_i,
    input wire [7:0]  cfg_pw1_i,
    input wire        cfg_dlyclk_i,
    input wire [7:0]  cfg_dly_i,
    input wire [3:0]  in_i,
    input wire [3:0]  in_q_i,
    input wire        dline_i,
    input wire        dedge_i,
    input wire        pw_bit_i,
    input wire        cu_v_i,
    input wire        cu_fld_i,
    input wire        cu_tx_i,
    input wire        cu_smp_i,
    input wire        pk_v_i,
    input wire        launch_req_i,
    input wire        fld_next_i,
    input wire        fld_first_i,
    input wire        data_wait_i,
    input wire        ot_start_i,
    input wire        ot_raw_i,
    input wire [15:0] ot_n_i,
    input wire        dcap_i,
    // outputs of the block
    input wire        pt_o,
    input wire        lt_o,
    input wire        ct_o,
    input wire        st_o,
    input wire        cte_o,
    input wire        hte_o,
    input wire        spend_o,
    input wire        bt_run_o,
    input wire        bt_stop_o,
    input wire        ot_done_o,
    input wire [15:0] d16_o,
    input wire        ge0_o,
    input wire        creq_o,
    input wire [1:0]  cpay_o,
    input wire        creq_dly_o,
    input wire        creq_cap_o,
    input wire        stretch_o
);
    localparam [1:0] M_BAUD = 2'd0, M_EDGE = 2'd1, M_REC = 2'd2, M_PULSE = 2'd3;
    localparam RTRIP = 4;

    // ---------------------------------------------------------------------
    // Spec functions
    // ---------------------------------------------------------------------
    function automatic ptf(input [8:0] p, input [1:0] ps);
        case (ps)
            2'd0:    ptf = 1'b1;
            2'd1:    ptf = (p[2:0] == 3'd7);
            2'd2:    ptf = (p[5:0] == 6'd63);
            default: ptf = (p == 9'd511);
        endcase
    endfunction
    // checker-side phase counters: TW bits, saturating; exact for every
    // comparison with Pd, P2d, Pr, Hr under env_range (2**TW - 1 > Pd)
    localparam TW = (PBITS < 16) ? PBITS + 1 : 16;
    localparam [15:0] TMAX = (16'd1 << TW) - 16'd1;
    function automatic [15:0] satTp(input [15:0] a, input b);
        satTp = (b && a < TMAX) ? a + 16'd1 : a;
    endfunction
    function automatic [15:0] sat16p(input [15:0] a, input b);
        sat16p = (b && a != 16'hFFFF) ? a + 16'd1 : a;
    endfunction
    // clock mapping of a level (tick.md 2): cpay = {v3, e3}
    function automatic [1:0] cmap(input c, input od);
        cmap = od ? {1'b0, !c} : {c, 1'b1};
    endfunction

    // ---------------------------------------------------------------------
    // Definitions (tick.md 2, lane.md 1)
    // ---------------------------------------------------------------------
    wire        CL   = !cfg_capfall_i;
    wire        LL   =  cfg_capfall_i;
    wire        IL   =  cfg_idle_i;
    wire        cin  = in_i[cfg_clkin_i];
    wire        cinq = in_q_i[cfg_clkin_i];
    wire [15:0] Pd   = (cfg_period_i  == 16'd0) ? 16'd1 : cfg_period_i;
    wire [15:0] P2d  = (cfg_period2_i == 16'd0) ? 16'd1 : cfg_period2_i;
    wire [15:0] Pr   = (cfg_period_i  <  16'd2) ? 16'd2 : cfg_period_i;
    wire [15:0] Hr   = Pr >> 1;
    wire        G    = run_i && !abort_i;
    wire        BM   = (cfg_mode_i == M_BAUD) || (cfg_mode_i == M_PULSE);  // BAUD machinery runs
    wire        BAUDT = (cfg_mode_i == M_BAUD) || (cfg_mode_i == M_PULSE && cu_tx_i);
    wire        dlycfg = cfg_dlyclk_i && (cfg_dly_i != 8'd0) && cfg_clkout_i;
    wire [15:0] Hpw  = {8'd0, pw_bit_i ? cfg_pw1_i : cfg_pw0_i};
    wire        el   = (cin != cinq) && (cin == LL);
    wire        ec   = (cin != cinq) && (cin == CL);
    wire [7:0]  d8   = (d16_o > 16'd255) ? 8'd255 : d16_o[7:0];

    // ---------------------------------------------------------------------
    // Executable model
    // ---------------------------------------------------------------------
    logic [8:0]  m_pre;
    logic        m_run, m_ph, m_lr, m_hw;
    logic [15:0] m_cnt;
    logic        m_spend, m_sarm, m_rw;
    logic [1:0]  m_scnt;
    logic        m_str, m_srel, m_twq;
    logic [15:0] m_ecnt;
    logic        m_ota;
    logic [11:0] m_oiv;       // intervals left after the current one
    logic [15:0] m_osc;       // strobes left in the current interval

    wire        m_pt   = ptf(m_pre, cfg_ps_i);
    wire        m_ptn  = ptf(m_pre + 9'd1, cfg_ps_i);
    wire [15:0] m_inc  = sat16p(m_cnt, m_pt);

    // BAUD (4.1); state decode
    wire b_stp  = !m_run;
    wire b_p0   =  m_run && !m_ph && !m_hw;
    wire b_hws  =  m_run && !m_ph &&  m_hw;
    wire b_p1   =  m_run &&  m_ph;
    wire b_cte  = b_p0 && (m_inc >= Pd) && !m_spend;
    wire b_ct   = (b_cte && !cfg_honour_i) || (b_hws && cin == CL);
    wire b_eob  = m_lr || (m_inc >= P2d);
    wire b_lt   = (b_stp && launch_req_i) || (b_p1 && b_eob && launch_req_i);
    wire b_stop = b_p1 && b_eob && !launch_req_i && !fld_next_i;
    wire b_hte  = b_p0 && m_pt && (m_inc == Hpw) && (Hpw != 16'd0);

    // EDGE (4.2)
    wire e_on    = G && (cfg_mode_i == M_EDGE);
    wire e_start = e_on && cfg_hold_i && cfg_clkout_i && !m_str && el &&
                   !cu_v_i && !pk_v_i && data_wait_i;
    wire e_first = e_on && m_str && fld_first_i && !m_srel;     // ruling 41
    wire e_rel   = e_on && m_srel && (m_inc >= Pd);
    wire e_lt    = el || (m_str && fld_first_i);

    // RECOVER (4.3)
    wire r_lt    = dedge_i || (m_pt && m_inc >= Pr);
    wire r_ct    = !dedge_i && m_pt && (m_inc == Hr);
    wire [15:0] r_next = (dedge_i || (m_pt && m_inc >= Pr)) ? 16'd0 : m_inc;

    // Sample (5)
    wire m_samp  = m_sarm && (m_scnt == 2'd3) && m_spend;
    wire mode_ct = (cfg_mode_i == M_EDGE) ? ec : (cfg_mode_i == M_REC) ? r_ct : dedge_i;

    // Operation timer (7)
    // N = raw ? max(ot_n, 1) : W * Pd strobes, kept as W intervals of Pd
    // strobes (the spec's implementation note; no N = W * Pd product)
    wire [15:0] ot_sc0 = ot_raw_i ? ((ot_n_i == 16'd0) ? 16'd1 : ot_n_i) : Pd;
    wire [11:0] ot_iv0 = ot_raw_i ? 12'd0 : (ot_n_i[11:0] - 12'd1);
    wire        ot_n1 = ot_raw_i ? (ot_n_i <= 16'd1) : (ot_n_i[11:0] == 12'd1 && Pd == 16'd1);
    wire        ot_w0 = !ot_raw_i && ot_n_i[11:0] == 12'd0;    // ruling 43: no start
    wire        m_olast = m_ota && m_osc == 16'd1 && m_oiv == 12'd0;
    // ruling 43: a new start suppresses the old timer's ot_done in its cycle
    wire        m_otd = G && (ot_start_i ? (ot_n1 && m_ptn) : (m_olast && m_ptn));

    // Model outputs
    wire mo_lt   = G && (BAUDT ? b_lt : (cfg_mode_i == M_EDGE) ? e_lt :
                         (cfg_mode_i == M_REC) ? r_lt : 1'b0);
    wire mo_ct   = G && (BAUDT ? b_ct : mode_ct);
    wire mo_cte  = G && (BAUDT ? b_cte : (cfg_mode_i == M_EDGE || cfg_mode_i == M_REC) ? mode_ct : 1'b0);
    wire mo_hte  = G && BAUDT && b_hte;
    wire mo_st   = G && (BAUDT ? (cfg_honour_i ? (b_ct && cu_smp_i) : m_samp) : mode_ct);
    wire mo_stop = G && BM && b_stop;
    wire b_req   = G && BM && (b_lt || b_cte || b_stop);
    wire t_req   = G && m_twq;
    wire a_req   = abort_i && m_str;
    // ruling 39: the idle request of a TICK write wins (a launch in that
    // cycle keeps its lt but makes no request)
    wire b_reqv  = b_req && !t_req;
    wire mo_creq = cfg_clkout_i && (b_req || e_start || e_rel || t_req || a_req);
    wire mo_lvl  = b_reqv ? (b_cte ? CL : b_lt ? LL : (cfg_rest_i ? LL : IL)) :
                   (e_start && !t_req) ? 1'b0 : IL;
    wire mo_dly  = b_reqv;
    wire mo_cap  = b_reqv && b_cte;
    wire mo_ge0  = (((m_ecnt > 16'd255) ? 8'd255 : m_ecnt[7:0]) >= cfg_thr0_i);
    wire samp_set = G && BAUDT && !cfg_honour_i && b_ct && cu_smp_i;
    wire arm_ev   = G && BM && b_ct;                  // ruling 42: every BAUD capture

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_pre <= 9'd0; m_run <= 1'b0; m_ph <= 1'b0; m_lr <= 1'b0; m_hw <= 1'b0;
            m_cnt <= 16'd0; m_spend <= 1'b0; m_sarm <= 1'b0; m_rw <= 1'b0;
            m_scnt <= 2'd0; m_str <= 1'b0; m_srel <= 1'b0; m_twq <= 1'b0;
            m_ecnt <= 16'd0; m_ota <= 1'b0; m_oiv <= 12'd0; m_osc <= 16'd0;
        end else if (!run_i) begin
            // 3.2 halt: every flop takes its reset value
            m_pre <= 9'd0; m_run <= 1'b0; m_ph <= 1'b0; m_lr <= 1'b0; m_hw <= 1'b0;
            m_cnt <= 16'd0; m_spend <= 1'b0; m_sarm <= 1'b0; m_rw <= 1'b0;
            m_scnt <= 2'd0; m_str <= 1'b0; m_srel <= 1'b0; m_twq <= 1'b0;
            m_ecnt <= 16'd0; m_ota <= 1'b0; m_oiv <= 12'd0; m_osc <= 16'd0;
        end else begin
            m_pre  <= m_pre + 9'd1;
            m_ecnt <= dedge_i ? {15'd0, m_pt} : sat16p(m_ecnt, m_pt);
            m_twq  <= wr_tick_i;
            // operation timer: cancelled by abort (and halt, above)
            if (abort_i) begin
                m_ota <= 1'b0;
            end else if (ot_start_i) begin
                m_ota <= !(ot_n1 && m_ptn) && !ot_w0;
                if (!m_ptn)                    begin m_osc <= ot_sc0; m_oiv <= ot_iv0; end
                else if (ot_sc0 == 16'd1)      begin m_osc <= Pd; m_oiv <= ot_iv0 - 12'd1; end
                else                           begin m_osc <= ot_sc0 - 16'd1; m_oiv <= ot_iv0; end
            end else if (m_ota) begin
                if (m_ptn && m_olast)          m_ota <= 1'b0;
                else if (m_ptn && m_osc == 16'd1) begin m_osc <= Pd; m_oiv <= m_oiv - 12'd1; end
                else if (m_ptn)                m_osc <= m_osc - 16'd1;
            end
            if (wr_tick_i || abort_i) begin
                // 3.3 / 3.2
                m_run <= 1'b0; m_ph <= 1'b0; m_lr <= 1'b0; m_hw <= 1'b0;
                m_spend <= 1'b0; m_sarm <= 1'b0; m_rw <= 1'b0; m_scnt <= 2'd0;
                m_str <= 1'b0; m_srel <= 1'b0;
                m_cnt <= (abort_i && cfg_mode_i == M_REC) ? r_next : 16'd0;
            end else begin
                // ---- BAUD timer (G holds here)
                if (BM) begin
                    if (b_stp) begin
                        if (launch_req_i) begin
                            m_run <= 1'b1; m_ph <= 1'b0; m_cnt <= 16'd0; m_lr <= 1'b0; m_hw <= 1'b0;
                        end
                    end else if (b_p0) begin
                        if (b_cte) begin
                            if (!cfg_honour_i) begin m_ph <= 1'b1; m_cnt <= 16'd0; end
                            else m_hw <= 1'b1;
                        end else begin
                            m_cnt <= m_inc;
                        end
                    end else if (b_hws) begin
                        if (cin == CL) begin m_ph <= 1'b1; m_cnt <= 16'd0; m_hw <= 1'b0; end
                    end else begin
                        if (!b_eob)             m_cnt <= m_inc;
                        else if (launch_req_i)  begin m_ph <= 1'b0; m_cnt <= 16'd0; m_lr <= 1'b0; end
                        else if (fld_next_i)    m_lr <= 1'b1;
                        else                    begin m_run <= 1'b0; m_lr <= 1'b0; end
                    end
                end else if (cfg_mode_i == M_REC) begin
                    m_cnt <= r_next;
                end else begin
                    // EDGE: stretch_hold
                    if (e_first) begin
                        m_srel <= 1'b1; m_cnt <= 16'd0;
                    end else if (m_srel) begin
                        if (m_inc >= Pd) begin m_str <= 1'b0; m_srel <= 1'b0; end
                        else m_cnt <= m_inc;
                    end
                    if (e_start) m_str <= 1'b1;
                end
                // ---- sample countdown (5)
                // A capture restarts the countdown (an older count or wait
                // is discarded: reading R-T5).
                if (samp_set)    m_spend <= 1'b1;
                else if (m_samp) m_spend <= 1'b0;    // ruling 42
                if (arm_ev) begin
                    if (!dlycfg || dcap_i) begin m_sarm <= 1'b1; m_scnt <= 2'd0; m_rw <= 1'b0; end
                    else begin m_rw <= 1'b1; m_sarm <= 1'b0; end
                end else begin
                    if (m_rw && dcap_i) begin
                        m_sarm <= 1'b1; m_scnt <= 2'd0; m_rw <= 1'b0;
                    end else if (m_sarm) begin
                        if (m_scnt == 2'd3) m_sarm <= 1'b0;
                        else m_scnt <= m_scnt + 2'd1;
                    end
                end
            end
        end
    end

    // ---------------------------------------------------------------------
    // Environment (tick.md 10 preamble, lane.md 2.1 and 6.1)
    //  env_rst        rst_n low in the first cycle
    //  env_cfg_o      non-TICK cfg_ inputs change only after a cycle with
    //                 cu_v = pk_v = 0 and bt_run = 0, or with run_i = 0;
    //                 also not while a BAUD sample is pending or a timer
    //                 runs (a timed op is current to its end; R-T6)
    //  env_cfg_t      TICK fields change only at the end of a wr_tick cycle
    //  env_wrtick     wr_tick only with run_i, no abort, idle lane
    //                 (lane.md 2.1: SETM of registers 0-9 only when
    //                 busy_o = park_full_o = 0; at most one strobe)
    //  env_after_wr   the cycle after a TICK write: data_wait 0 (the lane's
    //                 next instruction executes >= 5 cycles later). Ruling 39
    //                 says the TICK-write request wins over a stretch start
    //                 but not whether the stretch state starts (R-T2).
    //  env_abort      abort only with run_i (sequencer: strobes 0 while halted)
    //  env_fld        fld_first, launch_req only with cu_fld; ot_start only
    //                 with cu_v; cu_fld, cu_tx, cu_smp describe the current op
    //  env_otrun      no W = 0 non-raw start while a timer runs (ruling 43
    //                 says such a start starts nothing, not whether it
    //                 cancels the running timer: R-T7)
    //  env_pulse      in PULSE a receiving field never asserts launch_req
    //                 (4.4), and the transmitting field stays current while
    //                 its sample is pending (otherwise st would be both the
    //                 receive strobe and the BAUD sample: R-T4)
    //  env_range      proof range (PBITS, OBITS)
    // ---------------------------------------------------------------------
    wire [12:0] cfgT = {cfg_mode_i, cfg_ps_i, cfg_clkin_i, cfg_capfall_i, cfg_clkout_i,
                        cfg_idle_i, cfg_clkod_i, cfg_rest_i, cfg_honour_i, cfg_hold_i};
    wire [72:0] cfgO = {cfg_period_i, cfg_period2_i, cfg_thr0_i, cfg_pw0_i, cfg_pw1_i,
                        cfg_dlyclk_i, cfg_dly_i};
    wire idle_now = !run_i || (!cu_v_i && !pk_v_i && !m_run && !m_spend && !m_ota);

    wire env_wrtick_w = !wr_tick_i || (run_i && !abort_i && !cu_v_i && !pk_v_i && !m_run);
    wire env_abort_w  = !abort_i || run_i;
    wire env_fld_w    = (!fld_first_i || cu_fld_i) && (!launch_req_i || cu_fld_i) &&
                      (!ot_start_i || cu_v_i) && (!cu_fld_i || cu_v_i) &&
                      (!cu_tx_i || cu_fld_i) && (!cu_smp_i || cu_fld_i);
    wire env_otrun_w  = !(ot_start_i && ot_w0 && m_ota);
    wire env_pulse_w  = (cfg_mode_i != M_PULSE) || ((!launch_req_i || cu_tx_i) && (!m_spend || cu_tx_i));
    wire env_range_w  = ((cfg_period_i >> PBITS) == 16'd0) && ((cfg_period2_i >> PBITS) == 16'd0) &&
                      ((ot_n_i >> OBITS) == 16'd0);

    generate
        if (STANDALONE) begin : g_env_assume
            always @* begin
                if ($initstate) begin
                    env_rst: assume (!rst_n);
                end
                if (rst_n) begin
                    env_wrtick: assume (env_wrtick_w);
                    env_abort:  assume (env_abort_w);
                    env_fld:    assume (env_fld_w);
                    env_otrun:  assume (env_otrun_w);
                    env_pulse:  assume (env_pulse_w);
                end
                env_range: assume (env_range_w);
            end
            always @(posedge clk) begin
                if (!$initstate && rst_n && $past(rst_n)) begin
                    env_cfg_o:    assume (cfgO == $past(cfgO) || $past(idle_now));
                    env_cfg_t:    assume (cfgT == $past(cfgT) || $past(wr_tick_i));
                    env_after_wr: assume (!$past(wr_tick_i) || !data_wait_i);
                end
            end
        end else begin : g_env_assert
            always @* begin
                if ($initstate) begin
                    env_rst: assert (!rst_n);
                end
                if (rst_n) begin
                    env_wrtick: assert (env_wrtick_w);
                    env_abort:  assert (env_abort_w);
                    env_fld:    assert (env_fld_w);
                    env_otrun:  assert (env_otrun_w);
                    env_pulse:  assert (env_pulse_w);
                end
            end
            always @(posedge clk) begin
                if (!$initstate && rst_n && $past(rst_n)) begin
                    env_cfg_o:    assert (cfgO == $past(cfgO) || $past(idle_now));
                    env_cfg_t:    assert (cfgT == $past(cfgT) || $past(wr_tick_i));
                    env_after_wr: assert (!$past(wr_tick_i) || !data_wait_i);
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Exact behaviour: every output equals the model in every cycle.
    // (`define TICK_NO_MODEL_CHECK removes these, leaving the port-level
    // statements below to stand on their own.)
    // ---------------------------------------------------------------------
`ifndef TICK_NO_MODEL_CHECK
    always @* begin
        a_m_pt:     assert (pt_o      == m_pt);
        if (!wr_tick_i) begin
            a_m_lt:     assert (lt_o      == mo_lt);
        end else begin
            // ruling 44: a TICK write produces no lt (R-T3: an lt from the
            // old mode's own rules in that cycle may or may not be gated)
            a_m_lt_wr:  assert (!lt_o || mo_lt);
        end
        a_m_ct:     assert (ct_o      == mo_ct);
        a_m_st:     assert (st_o      == mo_st);
        a_m_cte:    assert (cte_o     == mo_cte);
        a_m_hte:    assert (hte_o     == mo_hte);
        a_m_spend:  assert (spend_o   == m_spend);
        a_m_btrun:  assert (bt_run_o  == m_run);
        a_m_btstop: assert (bt_stop_o == mo_stop);
        a_m_otdone: assert (ot_done_o == m_otd);
        a_m_d16:    assert (d16_o     == m_ecnt);
        a_m_ge0:    assert (ge0_o     == mo_ge0);
        a_m_creq:   assert (creq_o    == mo_creq);
        a_m_str:    assert (stretch_o == m_str);
        if (mo_creq) begin
            a_m_cpay: assert (cpay_o     == cmap(mo_lvl, cfg_clkod_i));
            a_m_cdly: assert (creq_dly_o == mo_dly);
            a_m_ccap: assert (creq_cap_o == mo_cap);
        end
    end
`endif

    // =====================================================================
    // Port-level statements (tick.md 10). Trackers below read only ports.
    // =====================================================================
    wire clr_ev = !run_i || abort_i || wr_tick_i;
    wire BV     = BAUDT;      // BAUD strobes are visible on the outputs

    // previous-cycle TICK write (port view of twq)
    logic pw_tick;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) pw_tick <= 1'b0; else pw_tick <= run_i && wr_tick_i;

    // ---- 1. Prescaler. The spec counter (3.1) is m_pre.
    always @* begin
        if (rst_n) begin
            a_p1_pt:  assert (pt_o == ptf(m_pre, cfg_ps_i));
            if (cfg_ps_i == 2'd0) begin
                a_p1_ps1: assert (pt_o);
            end
        end
    end
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n)) begin
            a_p1_pre: assert (m_pre == ($past(run_i) ? $past(m_pre) + 9'd1 : 9'd0));
        end
    end

    // ---- BAUD phase tracker: q0 launch->capture, qh honour wait,
    //      q1 capture->launch; qk = strobes since entering the phase,
    //      counted in the cycles after the entering tick.
    //      qv = the tracker knows the timer's phase (lost while the BAUD
    //      timer runs in PULSE with no transmitting field: its strobes are
    //      hidden, ruling 40; regained at the next launch or stop).
    logic        q0, qh, q1, qv;
    logic [15:0] qk;
    wire  [15:0] qinc = satTp(qk, pt_o);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                          qv <= 1'b1;
        else if (clr_ev || lt_o || bt_stop_o) qv <= 1'b1;
        else if (BM && !BV && bt_run_o)      qv <= 1'b0;
    end
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q0 <= 1'b0; qh <= 1'b0; q1 <= 1'b0; qk <= 16'd0;
        end else if (clr_ev || !BV) begin
            q0 <= 1'b0; qh <= 1'b0; q1 <= 1'b0; qk <= 16'd0;
        end else if (lt_o) begin
            q0 <= 1'b1; qh <= 1'b0; q1 <= 1'b0; qk <= 16'd0;
        end else if (bt_stop_o) begin
            q0 <= 1'b0; qh <= 1'b0; q1 <= 1'b0; qk <= 16'd0;
        end else if (q0 && cte_o && cfg_honour_i) begin
            q0 <= 1'b0; qh <= 1'b1;
        end else if ((q0 || qh) && ct_o) begin
            q0 <= 1'b0; qh <= 1'b0; q1 <= 1'b1; qk <= 16'd0;
        end else begin
            qk <= qinc;
        end
    end

    always @* begin
        if (rst_n && G && BV) begin
            // 2. launch -> capture: cte exactly when the strobe count since
            //    the launch tick reaches Pd (postponed while spend);
            //    no lt, no stop in between
            if (q0) begin
                a_p2_ph0_cte: assert (cte_o == (qinc >= Pd && !spend_o));
                a_p2_ph0_ct:  assert (ct_o  == (cte_o && !cfg_honour_i));
                a_p2_ph0_lt:  assert (!lt_o && !bt_stop_o);
                a_p2_ph0_req: assert (creq_o == (cte_o && cfg_clkout_i));
                if (Hpw < TMAX) begin
                    a_p2_ph0_hte: assert (hte_o == (pt_o && qinc == Hpw && Hpw != 16'd0));
                end
            end
            // 7. honour wait: ct exactly in the first cycle with cin == CL,
            //    nothing else happens (the timer pauses)
            if (qh) begin
                a_p7_wait_ct:    assert (ct_o == (cin == CL));
                a_p7_wait_quiet: assert (!cte_o && !lt_o && !bt_stop_o && !creq_o && !hte_o);
            end
            // 2. capture -> launch: nothing before the count since the
            //    capture tick reaches P2d; from then on (end of bit, also
            //    while holding) lt = launch_req, stop = no field coming
            if (q1) begin
                a_p2_ph1_noct: assert (!cte_o && !ct_o && !hte_o);
                if (qinc < P2d) begin
                    a_p2_ph1_early: assert (!lt_o && !bt_stop_o && !creq_o);
                end else begin
                    a_p2_ph1_lt:   assert (lt_o == launch_req_i);
                    a_p2_ph1_stop: assert (bt_stop_o == (!launch_req_i && !fld_next_i));
                    a_p2_ph1_req:  assert (creq_o == (cfg_clkout_i && (launch_req_i || !fld_next_i)));
                end
            end
            if (q0 || qh || q1) begin
                a_p2_running: assert (bt_run_o);
            end
        end
        // 3. start from stopped
        if (rst_n && G && BM && !bt_run_o) begin
            a_p3_start: assert (lt_o == launch_req_i);
            if (launch_req_i && cfg_clkout_i && !pw_tick) begin
                a_p3_req: assert (creq_o && cpay_o == cmap(LL, cfg_clkod_i) && creq_dly_o && !creq_cap_o);
            end
            a_p3_quiet: assert (!cte_o && !bt_stop_o && !hte_o && (cfg_mode_i == M_PULSE || !ct_o) &&
                                (!creq_o || launch_req_i || pw_tick));
        end
        // 4. hold never makes a clock edge or a stop
        if (rst_n && G && BM && bt_run_o && !launch_req_i && fld_next_i) begin
            a_p4_hold: assert (!lt_o && !bt_stop_o && (!creq_o || creq_cap_o));
        end
        // 5. stop
        if (rst_n && bt_stop_o) begin
            a_p5_cond: assert (G && BM && bt_run_o && !launch_req_i && !fld_next_i);
            if (cfg_clkout_i) begin
                a_p5_req: assert (creq_o && cpay_o == cmap(cfg_rest_i ? LL : IL, cfg_clkod_i) &&
                                  creq_dly_o && !creq_cap_o);
            end
        end
    end
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n && bt_stop_o)) begin
            a_p5_next: assert (!bt_run_o);
        end
    end

    // ---- 6. Clock alternation of the timer requests
    //      alt: 0 none / after stop, 1 after launch (LL), 2 after capture (CL)
    //      A launch in the cycle after a TICK write requests nothing
    //      (ruling 39: the idle request wins); it still counts as a launch.
    logic [1:0] alt;
    wire tq   = creq_o && creq_dly_o && G && BM;
    wire tlch = G && BV && lt_o && cfg_clkout_i;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                 alt <= 2'd0;
        else if (clr_ev)            alt <= 2'd0;
        else if (tlch)              alt <= 2'd1;
        else if (tq && creq_cap_o)  alt <= 2'd2;
        else if (tq && bt_stop_o)   alt <= 2'd0;
    end
    always @* begin
        if (rst_n && tlch) begin
            a_p6_launch: assert (alt != 2'd1);
        end
        if (rst_n && tq) begin
            a_p6_cause: assert (creq_cap_o || lt_o || bt_stop_o);
            if (creq_cap_o) begin
                a_p6_cap:     assert (alt == 2'd1 && cpay_o == cmap(CL, cfg_clkod_i));
            end else if (bt_stop_o) begin
                a_p6_stop:    assert (alt == 2'd2 && cpay_o == cmap(cfg_rest_i ? LL : IL, cfg_clkod_i));
            end else begin
                a_p6_lvl_lch: assert (cpay_o == cmap(LL, cfg_clkod_i) && !pw_tick);
            end
        end
    end

    // ---- 7. Honour (outside the wait: see a_p7_wait_*)
    always @* begin
        if (rst_n && G && BV && cfg_honour_i) begin
            if (qv) begin
                a_p7_ct_cl:  assert (!ct_o || (qh && cin == CL));
            end
            a_p7_st:     assert (st_o == (ct_o && cu_smp_i));
        end
        if (rst_n && cfg_honour_i) begin
            a_p7_nospend: assert (!spend_o);
        end
    end

    // ---- 8. Sample without honour: sp = pending, sw = waiting for the
    //      delayed capture-level clock change, rsh = reference cycles
    //      (shift register, rsh[3] = reference 4 cycles ago = RTRIP),
    //      ssh = capture ticks with cu_smp (same, for the undelayed case)
    logic       sp, sw;
    logic [3:0] rsh, ssh;
    wire s_set   = G && BV && !cfg_honour_i && ct_o && cu_smp_i;
    wire ref_now = (s_set && (!dlycfg || dcap_i)) || (!s_set && sw && dcap_i);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sp <= 1'b0; sw <= 1'b0; rsh <= 4'd0; ssh <= 4'd0;
        end else if (clr_ev) begin
            sp <= 1'b0; sw <= 1'b0; rsh <= 4'd0; ssh <= 4'd0;
        end else begin
            rsh <= {rsh[2:0], ref_now};
            ssh <= {ssh[2:0], s_set};
            if (s_set) begin
                sp <= 1'b1; sw <= dlycfg && !dcap_i;
            end else begin
                if (st_o)          sp <= 1'b0;
                if (sw && dcap_i)  sw <= 1'b0;
            end
        end
    end
    always @* begin
        if (rst_n) begin
            a_p8_spend: assert (spend_o == sp);
        end
        if (rst_n && G && BV && !cfg_honour_i) begin
            a_p8_st:  assert (st_o == (sp && rsh[3]));
            a_p8_one: assert (!s_set || !sp);
            if (!dlycfg) begin
                // the 4-cycle round trip, undelayed clock: st exactly 4
                // cycles after its capture tick
                a_p8_rtrip: assert (st_o == (sp && ssh[3]));
            end
        end
    end

    // ---- 9. EDGE
    always @* begin
        if (rst_n && G && cfg_mode_i == M_EDGE) begin
            a_p9_ct:  assert (ct_o == ec && st_o == ec && cte_o == ec);
            if (!wr_tick_i) begin
                a_p9_lt:  assert (lt_o == (el || (stretch_o && fld_first_i)));
            end else begin
                a_p9_lt_wr: assert (!lt_o || el);
            end
            a_p9_oth: assert (!hte_o && !bt_stop_o);
        end
    end

    // ---- 10. stretch_hold: rl = release count running since a FIELD
    //      start while stretched; rk = strobes since then
    logic        rl, rl_fire;
    logic [15:0] rk;
    wire  [15:0] rinc = satTp(rk, pt_o);
    wire  s_start_ok = G && cfg_mode_i == M_EDGE && el && !stretch_o && !cu_v_i && !pk_v_i &&
                       data_wait_i && cfg_clkout_i && cfg_hold_i;
    wire  s_start_rq = s_start_ok && creq_o && cpay_o == cmap(1'b0, cfg_clkod_i) && !creq_dly_o;
    wire  s_rel_now  = G && rl && (rinc >= Pd);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rl <= 1'b0; rk <= 16'd0; rl_fire <= 1'b0;
        end else if (clr_ev) begin
            rl <= 1'b0; rk <= 16'd0; rl_fire <= 1'b0;
        end else begin
            rl_fire <= s_rel_now;
            if (G && cfg_mode_i == M_EDGE && stretch_o && fld_first_i && !rl) begin
                rl <= 1'b1; rk <= 16'd0;          // ruling 41: no restart
            end else if (rl) begin
                if (rinc >= Pd) rl <= 1'b0;
                else rk <= rinc;
            end
        end
    end
    always @* begin
        if (rst_n && G && rl) begin
            a_p10_rel: assert (creq_o == (cfg_clkout_i && rinc >= Pd));
            if (creq_o) begin
                a_p10_rel_lvl: assert (cpay_o == cmap(IL, cfg_clkod_i) && !creq_dly_o);
            end
        end
        if (rst_n && abort_i && stretch_o) begin
            a_p10_abort: assert (creq_o == cfg_clkout_i);
            if (creq_o) begin
                a_p10_abort_lvl: assert (cpay_o == cmap(IL, cfg_clkod_i) && !creq_dly_o);
            end
        end
        if (rst_n && cfg_mode_i == M_EDGE && creq_o) begin
            a_p10_only: assert ((s_start_ok && !rl) || s_rel_now || (abort_i && stretch_o) ||
                                (G && pw_tick));
        end
    end
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n)) begin
            if (stretch_o && !$past(stretch_o)) begin
                a_p10_rise: assert ($past(s_start_rq));
            end
            if (!stretch_o && $past(stretch_o)) begin
                a_p10_fall: assert ($past(clr_ev) || rl_fire);
            end
            if (rl_fire) begin
                a_p10_released: assert (!stretch_o);
            end
        end
    end

    // ---- 11. RECOVER: rv_ok = the count is known from the ports (lost by
    //      abort, where cnt keeps running with the strobes hidden)
    //      The count is a function of ports (dedge_i, pt_o) and Pr, so it
    //      is followed through aborts too (cnt keeps counting there, 3.2).
    //      Only meaningful in RECOVER (the mode changes only at a TICK
    //      write, which restarts it).
    logic        rv_ok;
    logic [15:0] rv_k;
    wire  [15:0] rv_inc = satTp(rv_k, pt_o);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rv_k <= 16'd0;
        end else if (!run_i || wr_tick_i) begin
            rv_k <= 16'd0;
        end else if (dedge_i || (pt_o && rv_inc >= Pr)) begin
            rv_k <= 16'd0;
        end else begin
            rv_k <= rv_inc;
        end
    end
    always @* rv_ok = 1'b1;
    always @* begin
        if (rst_n && G && cfg_mode_i == M_REC) begin
            a_p11_strobes: assert (cte_o == ct_o && st_o == ct_o && !hte_o && !bt_stop_o);
            if (dedge_i) begin
                a_p11_edge: assert ((lt_o || wr_tick_i) && !ct_o);
            end else if (rv_ok) begin
                a_p11_ct: assert (ct_o == (pt_o && rv_inc == Hr));
                if (!wr_tick_i) begin
                    a_p11_lt: assert (lt_o == (pt_o && rv_inc >= Pr));
                end else begin
                    a_p11_lt_wr: assert (!lt_o || (pt_o && rv_inc >= Pr));
                end
            end
        end
    end

    // ---- 12. PULSE receive
    always @* begin
        if (rst_n && G && cfg_mode_i == M_PULSE && !cu_tx_i) begin
            a_p12_rx: assert (ct_o == dedge_i && st_o == dedge_i && !lt_o && !cte_o && !hte_o);
        end
    end

    // ---- 13. Edge interval: reference counter on the ports
    logic [15:0] e_ref;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)        e_ref <= 16'd0;
        else if (!run_i)   e_ref <= 16'd0;
        else if (dedge_i)  e_ref <= {15'd0, pt_o};
        else               e_ref <= sat16p(e_ref, pt_o);
    end
    always @* begin
        if (rst_n) begin
            a_p13_d16: assert (d16_o == e_ref);
            a_p13_ge0: assert (ge0_o == (d8 >= cfg_thr0_i));
        end
    end

    // ---- 14. Operation timer, cycle-aligned form (used by prove): a
    //      counter of the strobes still to come, decremented by pt_next
    //      (3.1: pt evaluated on pre + 1, with pre the spec prescaler that
    //      property 1 ties to pt_o). ot_done(t) <=> the timer is live in t,
    //      pt_next(t) and this strobe is the N-th after e: (e (+) N) - 1.
    //      Kept as W intervals of Pd strobes (no N = W * Pd product).
    logic        ta;
    logic [11:0] tiv;
    logic [15:0] tsc;
    wire  t_live = (G && ot_start_i && !ot_w0) || (G && !ot_start_i && ta);
    wire [15:0] t_sc = ot_start_i ? ot_sc0 : tsc;
    wire [11:0] t_iv = ot_start_i ? ot_iv0 : tiv;
    wire  t_hit  = t_live && m_ptn && t_sc == 16'd1 && t_iv == 12'd0;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ta <= 1'b0; tiv <= 12'd0; tsc <= 16'd0;
        end else if (!t_live || t_hit) begin
            ta <= 1'b0;
        end else begin
            ta <= 1'b1;
            if (m_ptn && t_sc == 16'd1) begin tsc <= Pd; tiv <= t_iv - 12'd1; end
            else if (m_ptn)             begin tsc <= t_sc - 16'd1; tiv <= t_iv; end
            else                        begin tsc <= t_sc; tiv <= t_iv; end
        end
    end
    always @* begin
        if (rst_n) begin
            a_p14_once: assert (ot_done_o == t_hit);
        end
    end

`ifndef TICK_NO_P14_LOOKAHEAD
    // ---- 14 (look-ahead form, bmc only: `define TICK_NO_P14_LOOKAHEAD
    //      removes it from prove). Independent of the spec prescaler. State for cycle t-1, checked in t once the
    //      strobe of t is visible: ot_done(t-1) <=> pt(t) and the strobes
    //      in (e, t] reach N. oa1 = a timer was live in t-1.
    //      oinv: the port view was lost because a TICK write changed the
    //      prescale under a running timer (ruling 43 lets it run on; pt(t)
    //      then no longer shows pt_next(t-1)); only the model check applies
    //      until the next start (R-T7).
    logic        oa1, od1, oinv;
    logic [11:0] oiv1;
    logic [15:0] osc1;
    logic [1:0]  p_ps;
    wire  ps_chg = (cfg_ps_i != p_ps);
    wire  o_last = osc1 == 16'd1 && oiv1 == 12'd0;
    wire  o_fin  = oa1 && pt_o && o_last;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            oa1 <= 1'b0; od1 <= 1'b0; oiv1 <= 12'd0; osc1 <= 16'd0; oinv <= 1'b0; p_ps <= 2'd0;
        end else begin
            od1  <= ot_done_o;
            p_ps <= cfg_ps_i;
            if (!G)                      begin oa1 <= 1'b0; oinv <= 1'b0; end
            else if (ot_start_i)         begin oa1 <= !ot_w0; osc1 <= ot_sc0; oiv1 <= ot_iv0; oinv <= 1'b0; end
            else if (oa1 && ps_chg)      begin oa1 <= 1'b0; oinv <= 1'b1; end
            else if (oa1 && !o_fin) begin
                if (pt_o && osc1 == 16'd1) begin osc1 <= Pd; oiv1 <= oiv1 - 12'd1; end
                else if (pt_o)             osc1 <= osc1 - 16'd1;
            end
            else                         oa1 <= 1'b0;
        end
    end
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n)) begin
            if (oa1 && !ps_chg) begin
                a_p14_done: assert (od1 == (pt_o && o_last));
            end else if (!oa1 && !oinv) begin
                a_p14_none: assert (!od1);
            end
        end
    end
`endif
    always @* begin
        if (rst_n && G && ot_start_i && ot_n1 && m_ptn) begin
            a_p14_first: assert (ot_done_o);
        end
    end

    // ---- 15. Abort and halt
    always @* begin
        if (rst_n && !G) begin
            a_p15_gate: assert (!lt_o && !ct_o && !st_o && !cte_o && !hte_o && !bt_stop_o && !ot_done_o);
            a_p15_creq: assert (!creq_o || (abort_i && stretch_o));
        end
        if (!rst_n) begin
            a_rst_regs: assert (!bt_run_o && !spend_o && !stretch_o && d16_o == 16'd0);
        end
    end
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n && !G)) begin
            a_p15_after: assert (!bt_run_o && !spend_o && !stretch_o);
        end
        if (!$initstate && rst_n && $past(rst_n && !run_i)) begin
            a_p15_halt: assert (d16_o == 16'd0);
        end
        if (!$initstate && rst_n && !$past(rst_n)) begin
            a_rst_rel: assert (!bt_run_o && !spend_o && !stretch_o && d16_o == 16'd0);
        end
    end

    // ---- 16. TICK write: the idle request in the next cycle
    always @* begin
        if (rst_n && G && pw_tick) begin
            a_p16_req: assert (creq_o == cfg_clkout_i);
            if (creq_o) begin
                a_p16_lvl: assert (cpay_o == cmap(IL, cfg_clkod_i) && !creq_dly_o);
            end
        end
    end

    // =====================================================================
    // Ruling 50: tick properties the field proofs assume (a_r50_*).
    //  1. lt and ct never in one cycle. FALSE in EDGE for the spec as
    //     written: lt = el || (stretch && fld_first), ct = ec, so a FIELD
    //     start while stretched in a capture-edge cycle gives both
    //     (c_r50_ltct_edge reaches it; with cfg_capfall_i = 1 the stretch's
    //     own low request makes that capture edge 4 cycles later). Asserted
    //     in every other mode, and in EDGE without the stretch term.
    //  2. In BAUD timing lt only with launch_req.
    //  3. st == ct outside BAUD timing; st == ct && cu_smp in BAUD timing
    //     with stretch_honour.
    //  4. In BAUD timing a ct needs an lt (in BAUD timing) since the last
    //     ct in BAUD timing.
    //  spend: rises the cycle after ct && cu_smp (BAUD timing, no honour)
    //  and only then; falls only after st, abort, halt or a TICK write, and
    //  always after st; st only while spend (BAUD timing, no honour; with
    //  honour or outside BAUD timing st comes with ct and spend stays 0);
    //  no ct while spend.
    // =====================================================================
    logic l_since;     // an lt in BAUD timing since the last ct in BAUD timing
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                      l_since <= 1'b0;
        else if (G && BV && lt_o)        l_since <= 1'b1;
        else if (G && BV && ct_o)        l_since <= 1'b0;
    end
    wire sp_cause = G && BV && !cfg_honour_i && ct_o && cu_smp_i;
    always @* begin
        if (rst_n) begin
            if (cfg_mode_i != M_EDGE) begin
                a_r50_ltct: assert (!(lt_o && ct_o));
            end else if (!(stretch_o && fld_first_i)) begin
                a_r50_ltct_edge: assert (!(lt_o && ct_o));
            end
            if (G && BV && lt_o) begin
                a_r50_lt_req: assert (launch_req_i);
            end
            if (!BV) begin
                a_r50_st_nb: assert (st_o == ct_o);
            end else if (cfg_honour_i) begin
                a_r50_st_hon: assert (st_o == (ct_o && cu_smp_i));
            end
            if (G && BV && ct_o) begin
                a_r50_ct_lt: assert (l_since);
            end
            if (G && BV && !cfg_honour_i && st_o) begin
                a_r50_st_sp: assert (spend_o);
            end
            if (spend_o) begin
                a_r50_ct_sp: assert (!ct_o);
            end
        end
    end
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n)) begin
            if ($past(sp_cause)) begin
                a_r50_sp_rise: assert (spend_o);
            end
            if (spend_o && !$past(spend_o)) begin
                a_r50_sp_why: assert ($past(sp_cause));
            end
            if (!spend_o && $past(spend_o)) begin
                a_r50_sp_fall: assert ($past(st_o || abort_i || !run_i || wr_tick_i));
            end
            if ($past(st_o && spend_o)) begin
                a_r50_sp_st: assert (!spend_o);
            end
        end
    end

    // =====================================================================
    // Covers (17 and the features of sections 3-8)
    // =====================================================================
    logic [3:0] hw_len;        // honour wait length so far (sat 15)
    logic       f_long_ct;     // a ct after an honour wait > 10 cycles
    logic       f_held;        // BAUD hold seen in the current phase 1
    logic       f_edge_lt;     // EDGE launch tick seen
    logic       f_rec_ct;      // RECOVER capture since the last launch
    logic       f_ot_w2;       // timer started with W = 2 at prescale 8
    logic       f_ot_r3;       // raw timer started with N = 3
    logic       f_strel;       // stretch released (by the count)
    logic       f_dly_ref;     // the pending sample took a delayed reference
    logic       f_otwr;        // a TICK write while the timer ran
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) f_otwr <= 1'b0;
        else if (G && ot_start_i) f_otwr <= 1'b0;
        else if (wr_tick_i && ta) f_otwr <= 1'b1;
    end
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hw_len <= 4'd0; f_long_ct <= 1'b0; f_held <= 1'b0; f_edge_lt <= 1'b0;
            f_rec_ct <= 1'b0; f_ot_w2 <= 1'b0; f_ot_r3 <= 1'b0; f_strel <= 1'b0;
            f_dly_ref <= 1'b0;
        end else begin
            hw_len <= (qh && !ct_o) ? ((hw_len == 4'd15) ? hw_len : hw_len + 4'd1) : 4'd0;
            if (qh && ct_o && hw_len > 4'd10) f_long_ct <= 1'b1;
            if (q1 && G && qinc >= P2d && !launch_req_i && fld_next_i) f_held <= 1'b1;
            else if (lt_o || bt_stop_o || clr_ev) f_held <= 1'b0;
            if (G && cfg_mode_i == M_EDGE && lt_o) f_edge_lt <= 1'b1;
            if (G && cfg_mode_i == M_REC && ct_o) f_rec_ct <= 1'b1;
            else if (lt_o) f_rec_ct <= 1'b0;
            if (G && ot_start_i) begin
                f_ot_w2 <= !ot_raw_i && ot_n_i[11:0] == 12'd2 && cfg_ps_i == 2'd1;
                f_ot_r3 <= ot_raw_i && ot_n_i == 16'd3;
            end
            if (s_rel_now) f_strel <= 1'b1;
            if (sw && dcap_i && !s_set) f_dly_ref <= 1'b1;
            else if (s_set) f_dly_ref <= 1'b0;
        end
    end

    generate
        if (STANDALONE) begin : g_cover
            always @(posedge clk) begin
                if (!$initstate && rst_n) begin
                    // reset release
                    c_rst_rel:    cover (!$past(rst_n));
                    // environment rules are satisfiable with activity
                    c_env_wrtick: cover (wr_tick_i && $past(wr_tick_i, 2));
                    c_env_cfg_o:  cover ($past(rst_n) && cfgO != $past(cfgO) && $past(run_i));
                    c_env_cfg_t:  cover ($past(rst_n) && cfgT != $past(cfgT));
                    c_env_abort:  cover (abort_i && bt_run_o);
                    c_env_spend:  cover (spend_o && cu_v_i && cu_smp_i && cfg_mode_i == M_PULSE);
                    c_env_ot0:    cover (G && ot_start_i && ot_raw_i && ot_n_i == 16'd0);
                    // 1. prescaler at 8: a strobe after 8 cycles of run
                    c_ps8:        cover (cfg_ps_i == 2'd1 && pt_o && $past(run_i, 7) && !$past(run_i, 8));
                    // BAUD full bit (launch, capture, stop) with Pd, P2d >= 2,
                    // and two back-to-back bits; at prescale 8
                    c_baud_bit:   cover (G && cfg_mode_i == M_BAUD && bt_stop_o && q1 &&
                                         cfg_period_i >= 16'd2 && cfg_period2_i >= 16'd2 && cfg_clkout_i);
                    c_baud_2bits: cover (G && cfg_mode_i == M_BAUD && lt_o && q1 && cfg_period_i >= 16'd2);
                    c_baud_ps8:   cover (G && cfg_mode_i == M_BAUD && bt_stop_o && q1 && cfg_ps_i == 2'd1);
                    c_baud_hte:   cover (G && cfg_mode_i == M_BAUD && hte_o && cfg_pw0_i != cfg_pw1_i);
                    // a BAUD hold, then the parked field's launch
                    c_hold:       cover (G && cfg_mode_i == M_BAUD && lt_o && f_held);
                    // postponed capture, bit time 4 (Pd + P2d = 4)
                    c_postpone:   cover (G && q0 && cte_o && qinc > Pd && Pd + P2d == 16'd4);
                    // sample in the same cycle as the next launch (3, 4)
                    c_samp_lt:    cover (G && st_o && lt_o && Pd == 16'd3 && P2d == 16'd4 && !cfg_honour_i);
                    // SPI bit at 3 + 3 with its delayed sample (11.2)
                    c_spi33:      cover (G && cfg_mode_i == M_BAUD && st_o && Pd == 16'd3 && P2d == 16'd3 &&
                                         !cfg_honour_i && cfg_clkout_i && !cfg_clkod_i);
                    // sample referenced to a delayed capture-level clock change
                    c_dly_samp:   cover (G && st_o && dlycfg && f_dly_ref);
                    // honour wait of more than 10 cycles; stretched I2C bit
                    // (open drain, rest, capture rising) to its stop
                    c_honour_10:  cover (G && ct_o && qh && hw_len > 4'd10);
                    c_i2c_strch:  cover (G && bt_stop_o && f_long_ct && cfg_honour_i && cfg_clkod_i &&
                                         cfg_rest_i && !cfg_capfall_i && creq_o);
                    // EDGE bit (launch edge, then capture edge), both polarities
                    c_edge_bit:   cover (G && cfg_mode_i == M_EDGE && ct_o && f_edge_lt && !cfg_capfall_i);
                    c_edge_fall:  cover (G && cfg_mode_i == M_EDGE && ct_o && f_edge_lt && cfg_capfall_i);
                    // stretch_hold start, release by the count (Pd >= 2), abort
                    c_str_start:  cover (stretch_o && !$past(stretch_o));
                    c_str_rel:    cover (s_rel_now && creq_o && Pd >= 16'd2);
                    c_str_abort:  cover (abort_i && stretch_o && creq_o);
                    c_str_bit:    cover (G && cfg_mode_i == M_EDGE && ct_o && f_strel);
                    // RECOVER: full bit by the timer (launch, capture, wrap);
                    // resync by a data edge in the first and in the second half
                    c_rec_bit:    cover (G && cfg_mode_i == M_REC && lt_o && !dedge_i && f_rec_ct && Pr >= 16'd4);
                    c_rec_sync1:  cover (G && cfg_mode_i == M_REC && dedge_i && rv_ok && rv_inc < Hr && rv_inc > 16'd0);
                    c_rec_sync2:  cover (G && cfg_mode_i == M_REC && dedge_i && rv_ok && rv_inc > Hr);
                    // PULSE receive: a short and a long pulse; transmit bit
                    c_pulse_shrt: cover (G && cfg_mode_i == M_PULSE && !cu_tx_i && ct_o && !ge0_o && d16_o > 16'd1);
                    c_pulse_long: cover (G && cfg_mode_i == M_PULSE && !cu_tx_i && ct_o && ge0_o && cfg_thr0_i > 8'd2);
                    c_pulse_tx:   cover (G && cfg_mode_i == M_PULSE && cu_tx_i && hte_o);
                    c_pulse_stop: cover (G && cfg_mode_i == M_PULSE && !cu_tx_i && bt_stop_o);
                    // operation timer: W = 2 at prescale 8; done in its start
                    // cycle; raw N = 3; cancelled by abort
                    c_ot_w2_ps8:  cover (ot_done_o && f_ot_w2 && !ot_start_i);
                    c_ot_first:   cover (ot_done_o && ot_start_i);
                    c_ot_raw3:    cover (ot_done_o && f_ot_r3 && !ot_start_i);
                    c_ot_cancel:  cover (abort_i && ta && !ot_start_i);
                    // TICK write: idle request next cycle
                    c_twq:        cover (pw_tick && creq_o);
                    // abort and halt while the timer runs
                    c_abort_run:  cover ($past(abort_i && bt_run_o) && !bt_run_o);
                    c_halt_run:   cover ($past(!run_i && bt_run_o && rst_n));
                    // rulings 39-43
                    c_twq_launch: cover (pw_tick && G && lt_o && creq_o && !creq_dly_o);
                    c_srel_2nd:   cover (G && rl && stretch_o && fld_first_i && cfg_mode_i == M_EDGE);
                    c_ot_restart: cover (G && ot_start_i && m_olast && m_ptn);
                    c_ot_wrtick:  cover (ot_done_o && f_otwr && !ot_start_i);
                    c_ot_w0:      cover (G && ot_start_i && ot_w0 && ot_n_i[15:12] != 4'd0);
                    c_pulse_hid:  cover (G && cfg_mode_i == M_PULSE && !cu_tx_i && creq_o && creq_cap_o);
                    c_samp_noff:  cover (G && cfg_mode_i == M_BAUD && st_o && !cu_v_i);
                    // ruling 50 item 1 is false in EDGE: lt and ct together
                    c_r50_ltct_edge: cover (G && cfg_mode_i == M_EDGE && lt_o && ct_o);
                end
            end
        end
    endgenerate

endmodule

// ===========================================================================
//  tick_worked: tick.md 11 as exact assertions. A scripted rosetta_tick
//  instance (own reset in the first cycle, inputs fixed functions of the
//  cycle index) must produce the tables' strobes and requests in every
//  cycle. Table cycle t = k - 1, k = 0 is the reset cycle.
//   SCEN 1  11.1 I2C controller bit with clock stretching
//   SCEN 2  11.2 SPI controller read at 3 + 3 (mode 0)
// ===========================================================================
module tick_worked #(
    parameter SCEN = 1
) (
    input wire         clk,
    output wire        w_rst_n,
    output wire        run_i,
    output wire        abort_i,
    output wire        wr_tick_i,
    output wire [1:0]  cfg_mode_i,
    output wire [1:0]  cfg_ps_i,
    output wire [1:0]  cfg_clkin_i,
    output wire        cfg_capfall_i,
    output wire        cfg_clkout_i,
    output wire        cfg_idle_i,
    output wire        cfg_clkod_i,
    output wire        cfg_rest_i,
    output wire        cfg_honour_i,
    output wire        cfg_hold_i,
    output wire [15:0] cfg_period_i,
    output wire [15:0] cfg_period2_i,
    output wire [7:0]  cfg_thr0_i,
    output wire [7:0]  cfg_pw0_i,
    output wire [7:0]  cfg_pw1_i,
    output wire        cfg_dlyclk_i,
    output wire [7:0]  cfg_dly_i,
    output wire [3:0]  in_i,
    output wire [3:0]  in_q_i,
    output wire        dline_i,
    output wire        dedge_i,
    output wire        pw_bit_i,
    output wire        cu_v_i,
    output wire        cu_fld_i,
    output wire        cu_tx_i,
    output wire        cu_smp_i,
    output wire        pk_v_i,
    output wire        launch_req_i,
    output wire        fld_next_i,
    output wire        fld_first_i,
    output wire        data_wait_i,
    output wire        ot_start_i,
    output wire        ot_raw_i,
    output wire [15:0] ot_n_i,
    output wire        dcap_i,
    input wire         pt_o,
    input wire         lt_o,
    input wire         ct_o,
    input wire         st_o,
    input wire         cte_o,
    input wire         hte_o,
    input wire         spend_o,
    input wire         bt_run_o,
    input wire         bt_stop_o,
    input wire         ot_done_o,
    input wire [15:0]  d16_o,
    input wire         ge0_o,
    input wire         creq_o,
    input wire [1:0]   cpay_o,
    input wire         creq_dly_o,
    input wire         creq_cap_o,
    input wire         stretch_o
);
    assign w_rst_n = !$initstate;
    logic [5:0] k;
    always_ff @(posedge clk or negedge w_rst_n)
        if (!w_rst_n) k <= 6'd0; else if (k != 6'd63) k <= k + 6'd1;
    wire        on = (k != 6'd0);
    wire [5:0]  t  = k - 6'd1;
    function automatic in_rng(input [5:0] x, input [5:0] lo, input [5:0] hi);
        in_rng = (x >= lo) && (x <= hi);
    endfunction

    localparam I2C = (SCEN == 1);
    // configuration (constant, also in reset)
    assign cfg_mode_i    = 2'd0;                 // BAUD
    assign cfg_ps_i      = 2'd0;                 // prescale 1
    assign cfg_clkin_i   = 2'd3;
    assign cfg_capfall_i = 1'b0;                 // capture rising: CL 1, LL 0
    assign cfg_clkout_i  = 1'b1;
    assign cfg_idle_i    = I2C ? 1'b1 : 1'b0;
    assign cfg_clkod_i   = I2C ? 1'b1 : 1'b0;
    assign cfg_rest_i    = I2C ? 1'b1 : 1'b0;
    assign cfg_honour_i  = I2C ? 1'b1 : 1'b0;
    assign cfg_hold_i    = 1'b0;
    assign cfg_period_i  = I2C ? 16'd6 : 16'd3;
    assign cfg_period2_i = I2C ? 16'd5 : 16'd3;
    assign cfg_thr0_i    = 8'd0;
    assign cfg_pw0_i     = 8'd0;
    assign cfg_pw1_i     = 8'd0;
    assign cfg_dlyclk_i  = 1'b0;
    assign cfg_dly_i     = 8'd0;
    // field and lane
    wire   fld     = on && (I2C ? in_rng(t, 0, 16) : in_rng(t, 0, 13));
    assign run_i        = on;
    assign abort_i      = 1'b0;
    assign wr_tick_i    = 1'b0;
    assign cu_v_i       = fld;
    assign cu_fld_i     = fld;
    assign cu_tx_i      = fld;                   // DRIVE / EXCHANGE
    assign cu_smp_i     = I2C ? 1'b0 : fld;      // DRIVE: no sample; EXCHANGE: sample
    assign pk_v_i       = 1'b0;
    assign launch_req_i = on && (I2C ? (t == 6'd0) : in_rng(t, 0, 6));
    assign fld_next_i   = 1'b0;
    assign fld_first_i  = on && (t == 6'd0);
    assign data_wait_i  = 1'b0;
    assign ot_start_i   = 1'b0;
    assign ot_raw_i     = 1'b0;
    assign ot_n_i       = 16'd0;
    assign dcap_i       = 1'b0;
    assign dline_i      = 1'b0;
    assign dedge_i      = 1'b0;
    assign pw_bit_i     = 1'b0;
    // SCL seen by the lane (11.1): the target releases the pin at 13, cin
    // rises 3 cycles later at 16; low again after the stop (pin 22 -> 25)
    wire   cin_t  = I2C && on && in_rng(t, 6'd16, 6'd24);
    wire   cin_tq = I2C && (k >= 6'd2) && in_rng(t - 6'd1, 6'd16, 6'd24);
    assign in_i   = {cin_t, 3'b000};
    assign in_q_i = {cin_tq, 3'b000};

    // expected outputs
    wire x_lt   = on && (I2C ? (t == 6'd0) : (t == 6'd0 || t == 6'd6));
    wire x_cte  = on && (I2C ? (t == 6'd6) : (t == 6'd3 || t == 6'd9));
    wire x_ct   = on && (I2C ? (t == 6'd16) : (t == 6'd3 || t == 6'd9));
    wire x_st   = on && (I2C ? 1'b0 : (t == 6'd7 || t == 6'd13));
    wire x_stop = on && (I2C ? (t == 6'd21) : (t == 6'd12));
    wire x_run  = on && (I2C ? in_rng(t, 1, 21) : in_rng(t, 1, 12));
    wire x_sp   = on && !I2C && (in_rng(t, 4, 7) || in_rng(t, 10, 13));
    wire x_creq = x_lt || x_cte || x_stop;
    // levels: launch LL = 0, capture CL = 1, stop rest ? LL : IL = 0
    wire x_lvl  = x_cte;
    wire [1:0] x_cpay = I2C ? {1'b0, !x_lvl} : {x_lvl, 1'b1};

    always @* begin
        if (on) begin
            w_lt:    assert (lt_o == x_lt);
            w_cte:   assert (cte_o == x_cte);
            w_ct:    assert (ct_o == x_ct);
            w_st:    assert (st_o == x_st);
            w_stop:  assert (bt_stop_o == x_stop);
            w_run:   assert (bt_run_o == x_run);
            w_spend: assert (spend_o == x_sp);
            w_creq:  assert (creq_o == x_creq);
            w_other: assert (!hte_o && !ot_done_o && !stretch_o && pt_o);
            if (x_creq) begin
                w_cpay: assert (cpay_o == x_cpay && creq_dly_o && creq_cap_o == x_cte);
            end
        end
    end
    // the table's end reached (k = 24 covers cycle 22 of 11.1)
    always @* begin
        w_end: cover (k == 6'd25);
    end
endmodule

// ===========================================================================
//  tick_ecnt_props: edge interval counter (tick.md 6) from a FREE initial
//  state (rst_n free from the first cycle; the reference starts equal to
//  d16_o). Exists to make the 16-bit saturation and the THR0 clamp at 255
//  reachable in cover; asserts the same counter rule from every d16 value.
//  Uses pt_o (property 1 ties pt_o to the spec prescaler in tick_props).
// ===========================================================================
module tick_ecnt_props (
    input wire        clk,
    input wire        rst_n,
    input wire        run_i,
    input wire        dedge_i,
    input wire [7:0]  cfg_thr0_i,
    input wire        pt_o,
    input wire [15:0] d16_o,
    input wire        ge0_o
);
    logic [15:0] e_ref;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)        e_ref <= 16'd0;
        else if (!run_i)   e_ref <= 16'd0;
        else if (dedge_i)  e_ref <= {15'd0, pt_o};
        else               e_ref <= (pt_o && e_ref != 16'hFFFF) ? e_ref + 16'd1 : e_ref;
    end
    wire [7:0] d8 = (d16_o > 16'd255) ? 8'd255 : d16_o[7:0];
    always @* begin
        if ($initstate) begin
            fi_ecnt: assume (e_ref == d16_o);
        end
        x_d16: assert (d16_o == e_ref);
        x_ge0: assert (ge0_o == (d8 >= cfg_thr0_i));
    end
    always @(posedge clk) begin
        if (!$initstate) begin
            // saturation: FFFF stays FFFF over a strobe without an edge
            x_c_sat:   cover (rst_n && $past(rst_n && run_i && !dedge_i && pt_o && d16_o == 16'hFFFF) &&
                              d16_o == 16'hFFFF);
            // the step into saturation, then an edge restarts the count
            x_c_top:   cover (rst_n && $past(rst_n && d16_o == 16'hFFFE && run_i && !dedge_i && pt_o) &&
                              d16_o == 16'hFFFF);
            x_c_reedge: cover (rst_n && $past(rst_n && d16_o == 16'hFFFF && run_i && dedge_i) &&
                              d16_o <= 16'd1);
        end
        // THR0 clamp: d16 > 255 classifies as 255 (long for THR0 = 255)
        x_c_clamp: cover (rst_n && d16_o > 16'd300 && cfg_thr0_i == 8'd255 && ge0_o);
        x_c_short: cover (rst_n && d16_o == 16'd254 && cfg_thr0_i == 8'd255 && !ge0_o);
    end
endmodule
