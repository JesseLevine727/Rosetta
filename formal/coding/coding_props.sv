`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_coding (line coding, receive and transmit).
//
//  Written from docs/blocks/coding.md, docs/blocks/lane.md 1-6 and 12.2 and
//  docs/decisions.md (rulings 20, 21, 27, 46, 47, 48 win over the text)
//  ONLY. The RTL was not read. Every property is about the block's ports.
//
//  Contents
//   * Executable model of sections 2-4 (the seven flops dl_q, nz_q, mph,
//     tl_q, hp, hv, pwb, as shadow copies that the ports determine
//     completely, section 5). EVERY output is compared with it in EVERY
//     cycle, reset cycles included: dline_o, dedge_o, rxb_o, rxv_o (not
//     gated by st_i, ruling 46), pw_bit_o, dreq_o, dreq_dly_o (ruling 48:
//     1 except for a RAW DRIVE request, also when idle) and dpay_o whenever
//     dreq_o = 1 (unspecified otherwise, ruling 48).
//   * Required properties 1-3, 7-9 of section 5 restated on the ports
//     (a_p1_* .. a_p9_*), using port-derived shadows only where the spec
//     says the checker keeps shadows (nz_q, mph, tl_q, hp); the NRZI
//     continuity shadow s_tl is rebuilt from the design's own payloads.
//     Required properties 4-6 (round trips) are in coding_rt.sv.
//   * Invert, open-drain (never drives high), differential complement,
//     differential-input hold, NRZI continuity across PIN writes, reset.
//   * Environment (env_*): section 5 assumptions, assumed (STANDALONE = 1)
//     or asserted (STANDALONE = 0, monitor inside a lane proof). ENV_SEQ = 0
//     keeps only the reset rule: since rulings 47 and 48 define every
//     same-cycle collision, the model is total and the proof then holds for
//     free inputs.
//   * Covers (required property 10 and one per assumption).
//
//  Assertions inside generate loops / unlabeled ones carry their name as a
//  trailing comment `// @name` (ref/run_mutants.sh reports by it).
// ===========================================================================

module coding_props #(
    parameter STANDALONE = 1,
    parameter ENV_SEQ    = 1
) (
    input wire       clk,
    input wire       rst_n,
    input wire       run_i,
    input wire       abort_i,
    input wire       wr_code_i,
    input wire       wr_tick_i,
    input wire [1:0] cfg_mode_i,
    input wire [1:0] cfg_rx_i,
    input wire [1:0] cfg_tx_i,
    input wire [1:0] cfg_din_i,
    input wire       cfg_inv_i,
    input wire       cfg_od_i,
    input wire       cfg_dif_i,
    input wire       cfg_dfi_i,
    input wire [7:0] cfg_pw0_i,
    input wire [7:0] cfg_pw1_i,
    input wire [3:0] in_i,
    input wire       st_i,
    input wire       cte_i,
    input wire       hte_i,
    input wire       ge0_i,
    input wire       tx_go_i,
    input wire       tx_bit_i,
    input wire       tx_rel_i,
    input wire       raw_go_i,
    input wire       raw_lvl_i,
    input wire       pin_we_i,
    input wire       pin_mask0_i,
    input wire       pin_val0_i,
    input wire       pin_oe0_i,
    input wire [1:0] out_val_i,
    input wire       dline_o,
    input wire       dedge_o,
    input wire       rxb_o,
    input wire       rxv_o,
    input wire       pw_bit_o,
    input wire       dreq_o,
    input wire [4:0] dpay_o,
    input wire       dreq_dly_o
);
    localparam [1:0] M_PULSE = 2'd3;   // TICK[1:0] = 3 (isa.md register 0)

    // payload fields {v0, e0, w1, v1, e1}
    wire v0 = dpay_o[4], e0 = dpay_o[3], w1 = dpay_o[2], v1 = dpay_o[1], e1 = dpay_o[0];
    wire lv0 = e0 ? v0 : 1'b1;              // wire level of out0 (pull-up)
    wire lv1 = e1 ? v1 : 1'b1;              // wire level of out1
    wire is_rel_pay = !e0 && !e1;           // a release payload (every mapped
                                            // payload enables out0 or out1)
    wire pay_l = (cfg_od_i ? !e0 : v0) ^ cfg_inv_i;  // logical level of a
                                                     // mapped payload

    // ---------------------------------------------------------------------
    // Environment (section 5)
    //  env_rst     rst_n low in the first cycle (conventions.md)
    //  env_cfg_*   CODE fields change only at the end of a cycle with
    //              wr_code_i, TICK (mode) only with wr_tick_i, or while
    //              halted or in reset. PULSE needs none (section 5).
    //  env_one     at most one of tx_go_i, tx_rel_i, raw_go_i
    //  env_gate    tx_go_i, tx_rel_i, raw_go_i, st_i, cte_i, hte_i are 0 in
    //              a cycle with abort_i or !run_i
    // ---------------------------------------------------------------------
    wire        f_past_ok = !$initstate;   // the previous cycle exists
    logic [9:0] p_code;
    logic [1:0] p_mode;
    logic       p_wrc, p_wrt, p_free;
    always @(posedge clk) begin
        p_code <= {cfg_rx_i, cfg_tx_i, cfg_din_i, cfg_inv_i, cfg_od_i, cfg_dif_i, cfg_dfi_i};
        p_mode <= cfg_mode_i;
        p_wrc  <= wr_code_i;
        p_wrt  <= wr_tick_i;
        p_free <= !run_i || !rst_n;
    end

    wire [9:0] code_now = {cfg_rx_i, cfg_tx_i, cfg_din_i, cfg_inv_i, cfg_od_i, cfg_dif_i, cfg_dfi_i};
    wire env_cfg_code_ok = !f_past_ok || p_wrc || p_free || (code_now == p_code);
    wire env_cfg_mode_ok = !f_past_ok || p_wrt || p_free || (cfg_mode_i == p_mode);
    wire env_one_ok  = !((tx_go_i && tx_rel_i) || (tx_go_i && raw_go_i) || (tx_rel_i && raw_go_i));
    wire env_gate_ok = !((abort_i || !run_i) &&
                         (tx_go_i || tx_rel_i || raw_go_i || st_i || cte_i || hte_i));

    generate
        if (STANDALONE) begin : g_env_assume
            always @* begin
                if ($initstate) begin
                    env_rst: assume (!rst_n);
                end
                if (ENV_SEQ) begin
                    env_cfg_code: assume (env_cfg_code_ok);
                    env_cfg_mode: assume (env_cfg_mode_ok);
                    if (rst_n) begin
                        env_one:  assume (env_one_ok);
                        env_gate: assume (env_gate_ok);
                    end
                end
            end
        end else begin : g_env_assert
            always @* begin
                if (ENV_SEQ) begin
                    env_cfg_code: assert (env_cfg_code_ok);
                    env_cfg_mode: assert (env_cfg_mode_ok);
                    if (rst_n) begin
                        env_one:  assert (env_one_ok);
                        env_gate: assert (env_gate_ok);
                    end
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Executable model (sections 2, 3, 4; rulings 46-48)
    // ---------------------------------------------------------------------
    logic m_dl, m_nz, m_mph, m_tl, m_hp, m_hv, m_pwb;

    wire P   = in_i[cfg_din_i];
    wire N   = in_i[cfg_din_i ^ 2'd1];
    wire ok  = !cfg_dfi_i || (P != N);
    wire m_dline = ok ? P : m_dl;
    wire m_dedge = ok && (P != m_dl);
    wire m_lvl   = m_dline ^ cfg_inv_i;
    wire pm      = (cfg_mode_i == M_PULSE);
    wire rx_man  = pm && (cfg_rx_i == 2'd2);
    wire rx_sym  = pm && (cfg_rx_i == 2'd3);

    logic m_rxb, m_rxv_t;
    always @* begin
        if (cfg_rx_i == 2'd1) begin          // NRZI, any mode
            m_rxb = (m_lvl == m_nz);  m_rxv_t = 1'b1;
        end else if (rx_man) begin           // Manchester (PULSE)
            m_rxb = m_lvl;            m_rxv_t = ge0_i || !m_mph;
        end else if (rx_sym) begin           // pulse symbol (PULSE)
            m_rxb = ge0_i;            m_rxv_t = !m_lvl;
        end else begin                       // level (and 2/3 outside PULSE)
            m_rxb = m_lvl;            m_rxv_t = 1'b1;
        end
    end
    wire m_rxv = ok && m_rxv_t;              // ruling 46

    function automatic [4:0] MAP(input logic l, input logic inv,
                                 input logic od, input logic dif);
        logic p, q;
        begin
            p = l ^ inv;
            q = !p;
            MAP = {od ? 1'b0 : p, od ? !p : 1'b1, dif, od ? 1'b0 : q, od ? !q : 1'b1};
        end
    endfunction

    wire [7:0] hsel  = tx_bit_i ? cfg_pw1_i : cfg_pw0_i;
    wire       hnz   = (hsel != 8'd0);
    logic m_go_l;
    always @* begin
        case (cfg_tx_i)
            2'd0:    m_go_l = tx_bit_i;
            2'd1:    m_go_l = tx_bit_i ? m_tl : !m_tl;
            2'd2:    m_go_l = !tx_bit_i;
            default: m_go_l = hnz;
        endcase
    end
    wire m_scc  = m_hp && ((cfg_tx_i == 2'd3) ? (hte_i || cte_i) : cte_i);
    wire r_go   = run_i && tx_go_i;
    wire r_rel  = run_i && !tx_go_i && tx_rel_i;
    wire r_raw  = run_i && !tx_go_i && !tx_rel_i && raw_go_i;
    wire r_sc   = run_i && !tx_go_i && !tx_rel_i && !raw_go_i && m_scc;
    wire m_dreq = r_go || r_rel || r_raw || r_sc;
    wire m_dly  = !r_raw;
    wire [4:0] pay_rel = {out_val_i[0], 1'b0, cfg_dif_i, out_val_i[1], 1'b0};
    wire [4:0] m_dpay  = r_go  ? MAP(m_go_l,    cfg_inv_i, cfg_od_i, cfg_dif_i) :
                         r_rel ? pay_rel :
                         r_raw ? MAP(raw_lvl_i, cfg_inv_i, cfg_od_i, cfg_dif_i) :
                                 MAP(m_hv,      cfg_inv_i, cfg_od_i, cfg_dif_i);
    wire pin0  = pin_we_i && pin_mask0_i;
    wire wr    = wr_code_i || wr_tick_i;
    wire pin_l = (pin_oe0_i ? pin_val0_i : 1'b1) ^ cfg_inv_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_dl <= 1'b0; m_nz <= 1'b1; m_mph <= 1'b1; m_tl <= 1'b0;
            m_hp <= 1'b0; m_hv <= 1'b0; m_pwb <= 1'b0;
        end else begin
            m_dl <= m_dline;                         // also while halted
            if (!run_i) begin
                m_nz <= 1'b1; m_mph <= 1'b1; m_tl <= 1'b0;
                m_hp <= 1'b0; m_hv <= 1'b0; m_pwb <= 1'b0;
            end else begin
                if (st_i && ok) m_nz <= m_lvl;       // every coding (3.2)
                if (wr) m_mph <= 1'b1;               // ruling 47: write wins
                else if (st_i && rx_man) m_mph <= ge0_i ? 1'b1 : !m_mph;
                if (pin0)       m_tl <= pin_l;       // ruling 47: PIN wins
                else if (r_go)  m_tl <= m_go_l;
                else if (r_raw) m_tl <= raw_lvl_i;   // release: unchanged
                else if (r_sc)  m_tl <= m_hv;
                if (r_go) begin
                    if (cfg_tx_i == 2'd2) begin m_hp <= 1'b1; m_hv <= tx_bit_i; end
                    else if (cfg_tx_i == 2'd3) begin m_hp <= hnz; m_hv <= 1'b0; m_pwb <= tx_bit_i; end
                    else m_hp <= 1'b0;
                end else if (r_rel || r_raw || r_sc) begin
                    m_hp <= 1'b0;
                end
                if (abort_i || wr) m_hp <= 1'b0;     // ruling 47: clear wins
            end
        end
    end

    // ---------------------------------------------------------------------
    // Exact behaviour, every cycle (reset cycles included)
    // ---------------------------------------------------------------------
`ifndef CODING_NO_MODEL_CHECK
    always @* begin
        a_m_dline:  assert (dline_o    == m_dline);
        a_m_dedge:  assert (dedge_o    == m_dedge);
        a_m_rxb:    assert (rxb_o      == m_rxb);
        a_m_rxv:    assert (rxv_o      == m_rxv);
        a_m_pwbit:  assert (pw_bit_o   == m_pwb);
        a_m_dreq:   assert (dreq_o     == m_dreq);
        a_m_dly:    assert (dreq_dly_o == m_dly);
        if (m_dreq) begin
            a_m_dpay: assert (dpay_o == m_dpay);
        end
    end
`endif

    // ---------------------------------------------------------------------
    // Port-derived shadows (section 5: the checker keeps shadow copies)
    //  s_dl   previous dline_o (0 after a reset cycle)            [dl_q]
    //  s_nz   logical level at the last valid sample: dline_o ^ inv  [nz_q]
    //  s_mph  Manchester phase from st_i, ge0_i and writes          [mph]
    //  s_tl   last logical level written to out0, decoded from the
    //         design's own mapped payloads, or a PIN write         [tl_q]
    //  s_hp   second change pending (Manchester / non-zero pulse)  [hp]
    // ---------------------------------------------------------------------
    logic s_dl, s_nz, s_mph, s_tl, s_hp, s_hv;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_dl <= 1'b0; s_nz <= 1'b1; s_mph <= 1'b1; s_tl <= 1'b0; s_hp <= 1'b0; s_hv <= 1'b0;
        end else begin
            s_dl <= dline_o;
            if (!run_i) begin
                s_nz <= 1'b1; s_mph <= 1'b1; s_tl <= 1'b0; s_hp <= 1'b0; s_hv <= 1'b0;
            end else begin
                if (st_i && ok) s_nz <= dline_o ^ cfg_inv_i;
                if (wr) s_mph <= 1'b1;
                else if (st_i && rx_man) s_mph <= ge0_i ? 1'b1 : !s_mph;
                if (pin0) s_tl <= pin_l;
                else if (dreq_o && !is_rel_pay) s_tl <= pay_l;
                if (tx_go_i && (cfg_tx_i == 2'd2)) begin s_hp <= 1'b1; s_hv <= tx_bit_i; end
                else if (tx_go_i && (cfg_tx_i == 2'd3)) begin s_hp <= hnz; s_hv <= 1'b0; end
                else if (dreq_o) s_hp <= 1'b0;
                if (abort_i || wr) s_hp <= 1'b0;
            end
        end
    end
    wire qual = (cfg_tx_i == 2'd3) ? (hte_i || cte_i) : cte_i;
    wire req_in = tx_go_i || tx_rel_i || raw_go_i;   // a request input

    // 1. Mapping, invert, open-drain, differential complement
    always @* begin
        if (dreq_o) begin
            a_p1_w1:      assert (w1 == cfg_dif_i);
            // open-drain never drives high (also not on a release)
            if (cfg_od_i) begin
                a_p1_od_high: assert (!(e0 && v0) && !(e1 && v1));
                if (!is_rel_pay) begin
                    a_p1_od_v0: assert (!v0 && !v1);
                end
            end else if (!r_rel) begin
                a_p1_pp_en:   assert (e0 && e1);
            end
            // differential: out1 is the exact complement of out0 on the
            // wire for every mapped request (exception: a release keeps the
            // values and clears both enables)
            if (!is_rel_pay) begin
                a_p1_compl:   assert (lv1 == !lv0);
            end
        end
        // each request input makes its request with its payload
        if (run_i && tx_go_i) begin
            a_p1_go:      assert (dreq_o && dpay_o == MAP(m_go_l, cfg_inv_i, cfg_od_i, cfg_dif_i));
        end
        if (run_i && tx_rel_i && !tx_go_i) begin
            a_p1_rel:     assert (dreq_o && dpay_o == pay_rel);
        end
        if (run_i && raw_go_i && !tx_go_i && !tx_rel_i) begin
            a_p1_raw:     assert (dreq_o && dpay_o == MAP(raw_lvl_i, cfg_inv_i, cfg_od_i, cfg_dif_i));
            // invert: the out0 wire carries raw_lvl ^ inv
            a_p1_raw_inv: assert (lv0 == (raw_lvl_i ^ cfg_inv_i));
        end
        if (run_i && tx_go_i && cfg_tx_i == 2'd0) begin
            a_p1_lvl_inv: assert (lv0 == (tx_bit_i ^ cfg_inv_i));
        end
        // halted: no request
        if (!run_i) begin
            a_p1_halt:    assert (!dreq_o);
        end
    end

    // 2. Line cleaning (ruling 20)
    always @* begin
        if (!cfg_dfi_i) begin
            a_p2_plain:   assert (dline_o == P);
        end
        if (cfg_dfi_i && (P == N)) begin
            a_p2_hold:    assert (dline_o == s_dl);
            a_p2_noedge:  assert (!dedge_o);
            a_p2_norxv:   assert (!rxv_o);
        end
        if (ok) begin
            a_p2_valid:   assert (dline_o == P);
            a_p2_edge:    assert (dedge_o == (P != s_dl));
        end
    end

    // 3. Decoding, table 3.2 (not gated by st_i, ruling 46)
    wire lvl_o = dline_o ^ cfg_inv_i;
    always @* begin
        if (cfg_rx_i == 2'd1) begin
            a_p3_nrzi:    assert (rxb_o == (lvl_o == s_nz) && rxv_o == ok);
        end
        if (cfg_rx_i == 2'd0 || (!pm && cfg_rx_i[1])) begin
            a_p3_level:   assert (rxb_o == lvl_o && rxv_o == ok);
        end
        if (rx_man) begin
            a_p3_man:     assert (rxb_o == lvl_o && rxv_o == (ok && (ge0_i || !s_mph)));
        end
        if (rx_sym) begin
            a_p3_sym:     assert (rxb_o == ge0_i && rxv_o == (ok && !lvl_o));
        end
    end

    // 7. Second change; 8. delay class; 9. NRZI continuity
    always @* begin
        // a request without a request input is a second change: one was
        // pending (Manchester or non-zero pulse width) and it qualifies
        if (dreq_o && !req_in) begin
            a_p7_only_pend: assert (s_hp && qual);
            a_p7_level:     assert (pay_l == s_hv);
        end
        // a pending second change is made at the first qualifying tick
        if (run_i && s_hp && qual && !req_in) begin
            a_p7_made:      assert (dreq_o);
        end
        // 8. dreq_dly = 0 exactly for RAW DRIVE requests (ruling 48)
        a_p8_dly:       assert (dreq_dly_o == !(run_i && raw_go_i && !tx_go_i && !tx_rel_i));
        // 9. NRZI: level after a bit = s_tl for a 1, !s_tl for a 0
        if (run_i && tx_go_i && cfg_tx_i == 2'd1) begin
            a_p9_nrzi:      assert (pay_l == (tx_bit_i ? s_tl : !s_tl));
        end
        // Manchester first half = !x; pulse width: high iff H(x) != 0
        if (run_i && tx_go_i && cfg_tx_i == 2'd2) begin
            a_p7_man_first: assert (pay_l == !tx_bit_i);
        end
        if (run_i && tx_go_i && cfg_tx_i == 2'd3) begin
            a_p7_pw_first:  assert (pay_l == hnz);
        end
    end

    // Registered output and reset
    always @(posedge clk) begin
        if (f_past_ok && rst_n && $past(rst_n)) begin
            if ($past(run_i && tx_go_i && cfg_tx_i == 2'd3)) begin
                a_pw_bit:   assert (pw_bit_o == $past(tx_bit_i));
            end else if ($past(run_i)) begin
                a_pw_hold:  assert (pw_bit_o == $past(pw_bit_o));
            end
            if (!$past(run_i)) begin
                a_pw_halt:  assert (!pw_bit_o);
            end
        end
    end
    always @* begin
        if (!rst_n) begin
            a_rst_pw:     assert (!pw_bit_o);
            a_rst_dline:  assert (dline_o == (ok ? P : 1'b0));
        end
    end

    // ---------------------------------------------------------------------
    // Cover helpers
    // ---------------------------------------------------------------------
    logic f_pin_nrzi;      // a PIN wrote out0 since the last request
    logic f_pin_then;      // an NRZI bit was sent after a PIN (the bit
                           // whose level depends on the PIN)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_pin_nrzi <= 1'b0; f_pin_then <= 1'b0;
        end else begin
            if (run_i && pin0) f_pin_nrzi <= 1'b1;
            else if (dreq_o) f_pin_nrzi <= 1'b0;
            if (dreq_o && tx_go_i && cfg_tx_i == 2'd1 && f_pin_nrzi) f_pin_then <= 1'b1;
        end
    end

    genvar g;
    generate
        if (STANDALONE) begin : g_cover
            always @(posedge clk) begin
                if (rst_n) begin
                    // receive codings
                    c_rx_level:   cover (st_i && cfg_rx_i == 2'd0 && rxv_o && rxb_o);
                    c_rx_nrzi0:   cover (st_i && cfg_rx_i == 2'd1 && rxv_o && !rxb_o);
                    c_rx_nrzi1:   cover (st_i && cfg_rx_i == 2'd1 && rxv_o &&  rxb_o && !s_nz);
                    c_rx_man_mid: cover (st_i && rx_man && rxv_o && ge0_i);
                    c_rx_man_shortmid: cover (st_i && rx_man && rxv_o && !ge0_i);
                    c_rx_man_bnd: cover (st_i && rx_man && !rxv_o && ok);
                    c_rx_sym0:    cover (st_i && rx_sym && rxv_o && !rxb_o);
                    c_rx_sym1:    cover (st_i && rx_sym && rxv_o &&  rxb_o);
                    c_rx_sym_rise:cover (st_i && rx_sym && !rxv_o && ok);
                    c_rx_man_nopm:cover (st_i && !pm && cfg_rx_i == 2'd2 && rxv_o);
                    c_rx_inv:     cover (st_i && cfg_inv_i && cfg_rx_i == 2'd0 && rxv_o && rxb_o != dline_o);
                    // invalid differential pair during a sample (SE0, SE1)
                    c_dfi_se0:    cover (st_i && cfg_dfi_i && !P && !N && !rxv_o && dline_o);
                    c_dfi_se1:    cover (st_i && cfg_dfi_i &&  P &&  N && !rxv_o && !dline_o);
                    c_dfi_edge:   cover (cfg_dfi_i && dedge_o);
                    // transmit codings
                    c_tx_level:   cover (tx_go_i && cfg_tx_i == 2'd0 && dreq_o);
                    c_tx_nrzi1:   cover (tx_go_i && cfg_tx_i == 2'd1 && tx_bit_i  && dreq_o && s_tl);
                    c_tx_nrzi0:   cover (tx_go_i && cfg_tx_i == 2'd1 && !tx_bit_i && dreq_o && s_tl);
                    c_tx_man:     cover (dreq_o && !req_in && cfg_tx_i == 2'd2);
                    c_tx_pw_hte:  cover (dreq_o && !req_in && cfg_tx_i == 2'd3 && hte_i && !cte_i);
                    c_tx_pw_cte:  cover (dreq_o && !req_in && cfg_tx_i == 2'd3 && cte_i && !hte_i);
                    c_tx_pw_zero: cover (tx_go_i && cfg_tx_i == 2'd3 && !hnz && dreq_o);
                    c_tx_od:      cover (dreq_o && cfg_od_i && cfg_dif_i && !is_rel_pay);
                    c_tx_dif:     cover (dreq_o && !cfg_od_i && cfg_dif_i && !is_rel_pay);
                    c_tx_rel:     cover (dreq_o && tx_rel_i && out_val_i == 2'b10);
                    c_tx_raw:     cover (dreq_o && raw_go_i && !dreq_dly_o);
                    c_tx_inv:     cover (dreq_o && tx_go_i && cfg_inv_i && cfg_tx_i == 2'd0);
                    // NRZI after a PIN
                    c_nrzi_pin:   cover (f_pin_then);
                    c_pin_rel:    cover (pin0 && !pin_oe0_i && run_i);
                    // collisions defined by rulings 47/48
                    c_wr_st_man:  cover (wr_code_i && st_i && rx_man && !ge0_i && !s_mph);
                    c_pin_req:    cover (pin0 && dreq_o);
                    c_abort_pend: cover (abort_i && s_hp && run_i);
                    c_wr_pend:    cover (wr_tick_i && s_hp && run_i);
                    c_man_go_over:cover (tx_go_i && s_hp && cfg_tx_i == 2'd2);
                    // environment assumptions satisfiable with activity
                    c_env_cfg:    cover (f_past_ok && $past(rst_n && run_i && wr_code_i) &&
                                         code_now != p_code);
                    c_env_mode:   cover (f_past_ok && $past(rst_n && run_i && wr_tick_i) &&
                                         cfg_mode_i != p_mode);
                    c_env_halt:   cover (f_past_ok && $past(rst_n && !run_i) && code_now != p_code);
                end
            end
        end
    endgenerate
endmodule
