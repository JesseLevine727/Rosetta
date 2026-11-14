`default_nettype none
// ===========================================================================
//  Round-trip proofs for rosetta_coding (coding.md 5, required properties
//  4, 5, 6), unbounded.
//
//  Two instances of the design: u_tx transmits, u_rx receives, through a
//  wire model of the lane output registers (pins.md 2: PIN beats a data
//  request; out1 written only with w1) and pull-ups: wire level
//  = oe ? val : 1. u_rx reads out0 at in_i[din] and out1 at in_i[din ^ 1]
//  (the other two inputs are free). Both share one CODE (and TICK mode),
//  a symbolic constant: invert, open-drain, data input, and differential
//  out/in (together: out1 feeding N) all take every value.
//
//  KIND 0  level and NRZI (property 4). The environment launches a bit
//          (tx_go) only after the previous bit has been sampled once
//          (st at u_rx, at least one cycle after the launch); RAW DRIVE,
//          release and PIN writes (out0 and out1, raw) may come between
//          bits. Watched-bit technique: a free input picks any launched
//          bit; its value is held until its sample, where u_rx must
//          return it (rxv = 1, rxb = bit). Every bit can be the watched
//          one, so every bit of every sequence of any length is returned.
//          NRZI precondition (property 4: receiver's nz_q equals the
//          transmitter's tl_q) is evaluated on port-derived shadows at the
//          watched launch; a_rt_ln_keep proves it then persists.
//  KIND 1  Manchester (property 5): u_rx in PULSE mode with the spec model
//          of rosetta_tick's PULSE receive path (tick.md 4.4, 6; prescale
//          1): st = u_rx.dedge_o, ge0 = min(ecnt, 255) >= THR0. u_tx sends
//          a continuous stream (tx_go, cte, tx_go, cte, ...); every line
//          edge interval after the first edge is within 1 cycle of its
//          nominal value n*h (n = half-bit boundaries since the previous
//          edge, 1 or 2), h >= 3 symbolic, h + 1 < THR0 <= 2h - 1 (h = 2 has
//          no legal THR0). After the first true mid-bit edge with a long
//          interval (ge0), every mid-bit edge carries exactly its bit and
//          every boundary edge carries none.
//  KIND 2  pulse width (property 6): high times 1 <= H0 < THR0 <= H1.
//          hte is the spec model of tick.md 4.1 (phase-0 count == H(pw_bit)
//          with pw_bit = u_tx.pw_bit_o), cte any time at or after it. For
//          every pulse launched while the line is logically low, the
//          falling edge carries exactly the bit; rising edges carry none.
//
//  In KIND 1 and 2 one bit is in flight at a time (fixed one-cycle wire
//  latency), so the in-flight bit register IS the watched bit and every
//  bit is checked. a_rt_*_edge proves u_rx sees exactly one edge per line
//  change, one cycle later, so no bit is lost or duplicated.
// ===========================================================================

module coding_rt #(
    parameter KIND = 0,
    parameter STANDALONE = 1
) (
    input wire clk
);
    // ---------------- free inputs ----------------
    (* anyseq *)   wire       rst_n;
    (* anyconst *) wire [1:0] c_mode, c_rx, c_tx, c_din;
    (* anyconst *) wire       c_inv, c_od, c_dif, c_dfi;
    (* anyconst *) wire [7:0] c_pw0, c_pw1, c_thr0;
    (* anyconst *) wire [3:0] c_h;
    (* anyseq *)   wire       a_go, a_bit, a_rel, a_raw, a_rawl, a_cte, a_hte, a_st, a_ge0, a_watch;
    (* anyseq *)   wire       a_pin, a_pm0, a_pv0, a_po0, a_pm1, a_pv1, a_po1;
    (* anyseq *)   wire [3:0] a_junk_rx, a_junk_tx;

    always @* begin
        if ($initstate) begin
            rt_rst:  assume (!rst_n);
        end else begin
            rt_run:  assume (rst_n);
        end
        rt_dif:      assume (c_dif == c_dfi);
    end

    // ---------------- per-kind strobes ----------------
    logic tx_go, tx_rel, raw_go, pin_we, tx_cte, tx_hte, rx_st, rx_ge0;
    wire  tx_dreq, tx_dly, tx_pwb, rx_dline, rx_dedge, rx_rxb, rx_rxv;
    wire [4:0] tx_dpay;
    wire  tx_dline_u, tx_dedge_u, tx_rxb_u, tx_rxv_u;
    wire  rx_pwb_u, rx_dreq_u, rx_dly_u;
    wire [4:0] rx_dpay_u;

    // ---------------- wire model ----------------
    logic [1:0] w_val, w_oe;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_val <= 2'b00; w_oe <= 2'b00;
        end else begin
            if (pin_we && a_pm0)  begin w_val[0] <= a_pv0; w_oe[0] <= a_po0; end
            else if (tx_dreq)     begin w_val[0] <= tx_dpay[4]; w_oe[0] <= tx_dpay[3]; end
            if (pin_we && a_pm1)  begin w_val[1] <= a_pv1; w_oe[1] <= a_po1; end
            else if (tx_dreq && tx_dpay[2]) begin w_val[1] <= tx_dpay[1]; w_oe[1] <= tx_dpay[0]; end
        end
    end
    wire wl0 = w_oe[0] ? w_val[0] : 1'b1;
    wire wl1 = w_oe[1] ? w_val[1] : 1'b1;
    logic [3:0] rx_in;
    always @* begin
        rx_in = a_junk_rx;
        rx_in[c_din] = wl0;
        rx_in[c_din ^ 2'd1] = wl1;
    end
    wire rx_ok = !c_dfi || (wl0 != wl1);

    // ---------------- the two instances ----------------
    rosetta_coding u_tx (
        .clk(clk), .rst_n(rst_n), .run_i(1'b1), .abort_i(1'b0),
        .wr_code_i(1'b0), .wr_tick_i(1'b0),
        .cfg_mode_i(c_mode), .cfg_rx_i(c_rx), .cfg_tx_i(c_tx), .cfg_din_i(c_din),
        .cfg_inv_i(c_inv), .cfg_od_i(c_od), .cfg_dif_i(c_dif), .cfg_dfi_i(c_dfi),
        .cfg_pw0_i(c_pw0), .cfg_pw1_i(c_pw1),
        .in_i(a_junk_tx), .st_i(1'b0), .cte_i(tx_cte), .hte_i(tx_hte), .ge0_i(1'b0),
        .tx_go_i(tx_go), .tx_bit_i(a_bit), .tx_rel_i(tx_rel), .raw_go_i(raw_go), .raw_lvl_i(a_rawl),
        .pin_we_i(pin_we), .pin_mask0_i(a_pm0), .pin_val0_i(a_pv0), .pin_oe0_i(a_po0),
        .out_val_i(w_val),
        .dline_o(tx_dline_u), .dedge_o(tx_dedge_u), .rxb_o(tx_rxb_u), .rxv_o(tx_rxv_u),
        .pw_bit_o(tx_pwb), .dreq_o(tx_dreq), .dpay_o(tx_dpay), .dreq_dly_o(tx_dly)
    );
    rosetta_coding u_rx (
        .clk(clk), .rst_n(rst_n), .run_i(1'b1), .abort_i(1'b0),
        .wr_code_i(1'b0), .wr_tick_i(1'b0),
        .cfg_mode_i(c_mode), .cfg_rx_i(c_rx), .cfg_tx_i(c_tx), .cfg_din_i(c_din),
        .cfg_inv_i(c_inv), .cfg_od_i(c_od), .cfg_dif_i(c_dif), .cfg_dfi_i(c_dfi),
        .cfg_pw0_i(c_pw0), .cfg_pw1_i(c_pw1),
        .in_i(rx_in), .st_i(rx_st), .cte_i(1'b0), .hte_i(1'b0), .ge0_i(rx_ge0),
        .tx_go_i(1'b0), .tx_bit_i(1'b0), .tx_rel_i(1'b0), .raw_go_i(1'b0), .raw_lvl_i(1'b0),
        .pin_we_i(1'b0), .pin_mask0_i(1'b0), .pin_val0_i(1'b0), .pin_oe0_i(1'b0),
        .out_val_i(2'b00),
        .dline_o(rx_dline), .dedge_o(rx_dedge), .rxb_o(rx_rxb), .rxv_o(rx_rxv),
        .pw_bit_o(rx_pwb_u), .dreq_o(rx_dreq_u), .dpay_o(rx_dpay_u), .dreq_dly_o(rx_dly_u)
    );

    // tick model for PULSE receive (tick.md 6, prescale 1, run_i = 1)
    logic [7:0] ecnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ecnt <= 8'd0;
        else ecnt <= rx_dedge ? 8'd1 : ((ecnt == 8'hFF) ? 8'hFF : ecnt + 8'd1);
    end
    wire ge0_model = (ecnt >= c_thr0);

    // logical level of a mapped payload / a PIN write (for the shadows)
    wire tx_pay_l   = (c_od ? !tx_dpay[3] : tx_dpay[4]) ^ c_inv;
    wire tx_pay_rel = !tx_dpay[3] && !tx_dpay[0];

    generate
    // =====================================================================
    if (KIND == 0) begin : g_ln
        always @* begin
            tx_go = a_go; tx_rel = a_rel; raw_go = a_raw; pin_we = a_pin;
            tx_cte = a_cte; tx_hte = a_hte; rx_st = a_st; rx_ge0 = a_ge0;
        end
        logic armed, w_pend, w_x, w_sync, s_tl, s_nz;
        logic [1:0] n_ok;    // cover: synced bits returned in a row
        logic f_pin, w_pin;  // cover: a PIN wrote out0 before the watched bit
        always @* begin
            ln_code: assume (c_tx == c_rx && !c_tx[1]);
            ln_one:  assume (!((a_go && (a_rel || a_raw || a_pin)) || (a_rel && (a_raw || a_pin)) ||
                               (a_raw && a_pin)));
            if (armed) begin
                ln_wait: assume (!a_go && !a_rel && !a_raw && !a_pin);
            end
        end
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                armed <= 1'b0; w_pend <= 1'b0; w_x <= 1'b0; w_sync <= 1'b0;
                s_tl <= 1'b0; s_nz <= 1'b1; n_ok <= 2'd0; f_pin <= 1'b0; w_pin <= 1'b0;
            end else begin
                if (a_pin && a_pm0) f_pin <= 1'b1;
                else if (a_go) f_pin <= 1'b0;
                if (a_go && a_watch) w_pin <= f_pin;
                if (a_go) armed <= 1'b1;
                else if (a_st) armed <= 1'b0;
                if (a_go && a_watch) begin
                    w_pend <= 1'b1; w_x <= a_bit;
                    // receiver reference after this cycle (a sample in the
                    // launch cycle still reads the old line) vs the level
                    // the NRZI bit is coded against
                    w_sync <= (c_tx == 2'd0) ||
                              (s_tl == ((a_st && rx_ok) ? (rx_dline ^ c_inv) : s_nz));
                end else if (a_st) begin
                    w_pend <= 1'b0;
                end
                // port-derived shadows: transmitter tl_q, receiver nz_q
                if (pin_we && a_pm0) s_tl <= (a_po0 ? a_pv0 : 1'b1) ^ c_inv;
                else if (tx_dreq && !tx_pay_rel) s_tl <= tx_pay_l;
                if (a_st && rx_ok) s_nz <= rx_dline ^ c_inv;
                if (a_st && w_pend && w_sync && c_tx == 2'd1) n_ok <= (n_ok == 2'd3) ? 2'd3 : n_ok + 2'd1;
                else if (a_pin || a_raw || a_rel) n_ok <= 2'd0;
            end
        end
        always @* begin
            if (rst_n && a_st && w_pend && w_sync) begin
                a_rt_ln_bit:  assert (rx_rxv && rx_rxb == w_x);
            end
        end
        always @(posedge clk) begin
            if (!$initstate && rst_n && $past(rst_n && a_st && w_pend && w_sync && c_tx == 2'd1)) begin
                a_rt_ln_keep: assert (s_tl == s_nz);
            end
        end
        if (STANDALONE) begin : g_cov
            always @(posedge clk) begin
                if (rst_n) begin
                    c_rt_lvl0:     cover (a_st && w_pend && c_tx == 2'd0 && !w_x && c_inv && c_od && c_dif);
                    c_rt_lvl1:     cover (a_st && w_pend && c_tx == 2'd0 &&  w_x && !c_inv && !c_od && !c_dif);
                    c_rt_nrzi_inv: cover (a_st && w_pend && w_sync && c_tx == 2'd1 && c_inv && n_ok == 2'd2);
                    c_rt_nrzi_raw: cover (a_st && w_pend && w_sync && c_tx == 2'd1 && !c_inv && c_dif && !w_x);
                    c_rt_nrzi_run: cover (n_ok == 2'd3 && c_od);
                    c_rt_nrzi_pin: cover (a_st && w_pend && w_sync && c_tx == 2'd1 && !c_inv && w_pin);
                end
            end
        end
    end
    // =====================================================================
    else if (KIND == 1) begin : g_man
        // stream generator truth
        logic hph, hx, twl, seen, q_chg, q_mid, q_x, q_req, st1, locked;
        logic [7:0] since;
        logic [1:0] nhb;
        logic [2:0] f_cnt;       // cover: bits decoded after lock
        wire lk_now = locked || (st1 && rx_dedge && q_mid && rx_ge0);
        always @* begin
            tx_go = a_go; tx_rel = 1'b0; raw_go = 1'b0; pin_we = 1'b0;
            tx_cte = a_cte; tx_hte = a_hte; rx_st = rx_dedge; rx_ge0 = ge0_model;
        end
        wire req1  = a_go;                 // first-half request (boundary)
        wire req2  = a_cte && hph;         // second-half request (mid-bit)
        wire l_new = req1 ? !a_bit : hx;   // logical level requested
        wire chg   = (req1 || req2) && ((l_new ^ c_inv) != twl);
        wire [5:0] nom = (nhb == 2'd0) ? {2'b00, c_h} : {1'b0, c_h, 1'b0};  // n*h, n = nhb+1
        always @* begin
            mn_code: assume (c_tx == 2'd2 && c_rx == 2'd2 && c_mode == 2'd3);
            mn_h:    assume (c_h >= 4'd3);
            mn_thr:  assume ({4'd0, c_h} + 8'd1 < c_thr0 && c_thr0 <= {3'd0, c_h, 1'b0} - 8'd1);
            mn_go:   assume (!(a_go && hph));
            mn_cte:  assume (!(a_cte && !hph));
            if (chg && seen) begin
                mn_jit: assume ({2'b00, since} + 10'd1 >= {4'd0, nom} &&
                                {2'b00, since} <= {4'd0, nom} + 10'd1);
            end
        end
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                hph <= 1'b0; hx <= 1'b0; twl <= 1'b1; seen <= 1'b0; since <= 8'd0; nhb <= 2'd0;
                q_chg <= 1'b0; q_mid <= 1'b0; q_x <= 1'b0; q_req <= 1'b0; st1 <= 1'b0;
                locked <= 1'b0; f_cnt <= 3'd0;
            end else begin
                if (req1) begin hph <= 1'b1; hx <= a_bit; end
                else if (req2) hph <= 1'b0;
                if (req1 || req2) twl <= l_new ^ c_inv;
                if (chg) begin seen <= 1'b1; since <= 8'd1; nhb <= 2'd0; end
                else begin
                    since <= (since == 8'hFF) ? 8'hFF : since + 8'd1;
                    if (req1 || req2) nhb <= (nhb == 2'd3) ? 2'd3 : nhb + 2'd1;
                end
                q_chg <= chg; q_mid <= req2; q_x <= hx; q_req <= req1 || req2;
                if (q_req) st1 <= 1'b1;
                if (st1 && rx_dedge && q_mid && rx_ge0) locked <= 1'b1;
                if (lk_now && rx_dedge && q_mid && f_cnt != 3'd7) f_cnt <= f_cnt + 3'd1;
            end
        end
        always @* begin
            if (rst_n && st1) begin
                a_rt_man_edge: assert (rx_dedge == q_chg);
            end
            if (rst_n && st1 && q_mid) begin
                a_rt_man_midchg: assert (q_chg);   // every mid-bit request changes the line
            end
            if (rst_n && lk_now && rx_dedge && q_mid) begin
                a_rt_man_bit:  assert (rx_rxv && rx_rxb == q_x);
            end
            if (rst_n && lk_now && rx_dedge && !q_mid) begin
                a_rt_man_bnd:  assert (!rx_rxv);
            end
        end
        if (STANDALONE) begin : g_cov
            always @(posedge clk) begin
                if (rst_n) begin
                    c_rt_man_lock:  cover (locked && !$past(locked));
                    c_rt_man_b0:    cover (lk_now && rx_dedge && q_mid && !q_x && !rx_ge0);
                    c_rt_man_b1:    cover (lk_now && rx_dedge && q_mid &&  q_x &&  rx_ge0);
                    c_rt_man_bnd:   cover (lk_now && rx_dedge && !q_mid && st1);
                    c_rt_man_jlo:   cover (lk_now && chg && seen && since == {4'd0, c_h} - 8'd1 && nhb == 2'd0);
                    c_rt_man_jhi:   cover (lk_now && chg && seen && since == {3'd0, c_h, 1'b0} + 8'd1);
                    c_rt_man_run:   cover (f_cnt == 3'd5 && c_od && c_dif && c_inv);
                    c_rt_man_h4:    cover (f_cnt == 3'd3 && c_h == 4'd4 && c_thr0 == 8'd7);
                end
            end
        end
    end
    // =====================================================================
    else begin : g_pw
        logic ph0, hpend, hx, twl, q_chg, q_fall, q_rise, q_x, q_rok, q_req, st1, rok;
        // q_req_any: a request has been made before this cycle (a rise then
        // lands after the receiver has seen a first valid pair)
        logic q_req_any;
        logic [7:0] kcnt;
        logic [2:0] f_cnt;
        wire [7:0] h_des = tx_pwb ? c_pw1 : c_pw0;     // tick uses pw_bit_o
        wire [7:0] h_tru = hx ? c_pw1 : c_pw0;         // the launched bit
        always @* begin
            tx_go = a_go; tx_rel = 1'b0; raw_go = 1'b0; pin_we = 1'b0;
            tx_cte = a_cte; tx_hte = ph0 && (kcnt == h_des) && (h_des != 8'd0);
            rx_st = rx_dedge; rx_ge0 = ge0_model;
        end
        wire rise = a_go;
        wire fall = hpend && ph0 && ((kcnt == h_tru) || a_cte);
        wire l_new = rise ? 1'b1 : 1'b0;
        wire chg_t = (rise || fall) && ((l_new ^ c_inv) != twl);
        always @* begin
            pw_code: assume (c_tx == 2'd3 && c_rx == 2'd3 && c_mode == 2'd3);
            pw_thr:  assume (c_pw0 != 8'd0 && c_pw0 < c_thr0 && c_thr0 <= c_pw1);
`ifndef RT_PW_FULL
            // H1 (so H0 and THR0) below 16 for the unbounded proof: the
            // design reads cfg_pw0_i/cfg_pw1_i only as "is it zero"
            // (coding.md 1; proven for all 8-bit values by the unit check,
            // a_m_dpay/a_m_dreq), so the magnitude only exercises this
            // harness's tick model, not the design.
            pw_rng:  assume (c_pw1 < 8'd16);
`endif
            pw_go:   assume (!(a_go && ph0));
            pw_cte:  assume (!(a_cte && !(ph0 && kcnt >= h_tru)));
        end
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                ph0 <= 1'b0; hpend <= 1'b0; hx <= 1'b0; twl <= 1'b1; kcnt <= 8'd0;
                q_chg <= 1'b0; q_fall <= 1'b0; q_rise <= 1'b0; q_x <= 1'b0; q_rok <= 1'b0;
                q_req <= 1'b0; st1 <= 1'b0; rok <= 1'b0; f_cnt <= 3'd0;
            end else begin
                if (rise) begin ph0 <= 1'b1; hpend <= 1'b1; hx <= a_bit; kcnt <= 8'd1;
                                rok <= chg_t && q_req_any; end
                else begin
                    if (a_cte) ph0 <= 1'b0;
                    if (fall) hpend <= 1'b0;
                    if (ph0) kcnt <= (kcnt == 8'hFF) ? 8'hFF : kcnt + 8'd1;
                end
                if (rise || fall) twl <= l_new ^ c_inv;
                q_chg <= chg_t; q_fall <= fall; q_rise <= rise; q_x <= hx; q_rok <= rok;
                q_req <= rise || fall;
                if (q_req) st1 <= 1'b1;
                if (st1 && rx_dedge && q_fall && q_rok && f_cnt != 3'd7) f_cnt <= f_cnt + 3'd1;
            end
        end
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) q_req_any <= 1'b0;
            else if (rise || fall) q_req_any <= 1'b1;
        end
        always @* begin
            if (rst_n && st1) begin
                a_rt_pw_edge: assert (rx_dedge == q_chg);
            end
            if (rst_n && st1 && rx_dedge && q_fall && q_rok) begin
                a_rt_pw_bit:  assert (rx_rxv && rx_rxb == q_x);
            end
            if (rst_n && st1 && rx_dedge && q_rise) begin
                a_rt_pw_rise: assert (!rx_rxv);
            end
        end
        if (STANDALONE) begin : g_cov
            always @(posedge clk) begin
                if (rst_n) begin
                    c_rt_pw0:    cover (st1 && rx_dedge && q_fall && q_rok && !q_x && c_pw0 == c_thr0 - 8'd1);
                    c_rt_pw1:    cover (st1 && rx_dedge && q_fall && q_rok &&  q_x && c_pw1 == c_thr0);
                    c_rt_pw_cte: cover (fall && a_cte && kcnt == h_tru);
                    c_rt_pw_run: cover (f_cnt == 3'd4 && c_od && c_dif && !c_inv);
                    c_rt_pw_inv: cover (f_cnt == 3'd2 && c_inv && !c_od);
                end
            end
        end
    end
    endgenerate
endmodule
