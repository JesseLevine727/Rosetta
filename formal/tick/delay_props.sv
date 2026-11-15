`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_delay (launch delay unit).
//
//  Written from docs/blocks/tick.md section 9 (and 10, rosetta_delay
//  items), lane.md 6.5 and 12.2 R9 ONLY. The RTL was not read. Every
//  property is about the ports.
//
//  Amended 15 Nov for ruling 58 (tick.md 9.2 rule 4, amended text only; the
//  new RTL was not read): a delayed request arriving in the cycle a PIN
//  writes an output that request would write is DROPPED (never pending,
//  never output); rules 2 and 3 are unchanged and a dropped request still
//  counts as "a new delayed request in t" for rule 3 (it flushes a pending
//  change the PIN does not cancel). Model, tracker, a_drop_* and c_drop_*.
//
//  Contents
//   * Executable model of 9.2 (pv, pclk, ppay, pcap, pcnt), compared with
//     every output in every cycle (a_m_*). d_pay_o / c_pay_o are compared
//     only when their write enable is 1, and are 0 while halted (ruling 45;
//     reading R-D1 in the report).
//   * Required properties 1-5 of tick.md 10 (rosetta_delay) restated on the
//     ports with a checker-side tracker of the one pending change
//     (a_p<N>_*), plus the PIN-cancel rule (a_pin_*), halt (a_halt) and
//     reset (a_rst).
//   * Environment (env_*), assumed when STANDALONE = 1, asserted otherwise.
//   * Covers.
// ===========================================================================

module delay_props #(
    parameter STANDALONE = 1
) (
    input wire       clk,
    input wire       rst_n,
    input wire       run_i,
    input wire       abort_i,
    input wire       wr_tick_i,
    input wire [7:0] cfg_dly_i,
    input wire       cfg_dlyclk_i,
    input wire       cfg_dlyps_i,
    input wire       pt_i,
    input wire       dreq_i,
    input wire [4:0] dpay_i,
    input wire       dreq_dly_i,
    input wire       creq_i,
    input wire [1:0] cpay_i,
    input wire       creq_dly_i,
    input wire       creq_cap_i,
    input wire       pin_we_i,
    input wire [3:0] pin_mask_i,
    input wire       d_we_o,
    input wire [4:0] d_pay_o,
    input wire       c_we_o,
    input wire [1:0] c_pay_o,
    input wire       dcap_o
);
    // ---------------------------------------------------------------------
    // Request classification (9.2)
    // ---------------------------------------------------------------------
    wire dz    = (cfg_dly_i == 8'd0);
    wire d_del = dreq_i && dreq_dly_i && !cfg_dlyclk_i && !dz;   // delayed data request
    wire c_del = creq_i && creq_dly_i &&  cfg_dlyclk_i && !dz;   // delayed clock request
    wire d_und = dreq_i && !d_del;                               // passes through
    wire c_und = creq_i && !c_del;
    wire newd  = d_del || c_del;
    // 9.2 rule 4 (ruling 58): the new delayed request is dropped when a PIN
    // in its cycle writes an output it would write; w1_new = dpay_i[2]
    wire drop  = newd && pin_we_i && (c_del ? pin_mask_i[3] :
                                      (pin_mask_i[0] || (pin_mask_i[1] && dpay_i[2])));

    // ---------------------------------------------------------------------
    // Executable model
    // ---------------------------------------------------------------------
    logic       pv, pclk, pcap;
    logic [4:0] ppay;
    logic [7:0] pcnt;

    wire [8:0] adv    = cfg_dlyps_i ? ((pcnt == 8'hFF) ? 9'd255 : {1'b0, pcnt} + {8'd0, pt_i})
                                    : {1'b0, pcnt} + 9'd1;
    wire       mat    = pv && (adv >= {1'b0, cfg_dly_i});
    wire       same_u = pv && (pclk ? c_und : d_und);
    wire       pinhit = pin_we_i && pv && (pclk ? pin_mask_i[3] :
                                           (pin_mask_i[0] || (pin_mask_i[1] && ppay[2])));
    wire       cancel = abort_i || wr_tick_i || same_u || pinhit;
    wire       pout   = pv && !cancel && (mat || newd);

    wire       mo_dwe  = run_i && (d_und || (pout && !pclk));
    wire [4:0] mo_dpay = d_und ? dpay_i : ppay;
    wire       mo_cwe  = run_i && (c_und || (pout && pclk));
    wire [1:0] mo_cpay = c_und ? cpay_i : ppay[1:0];
    wire       mo_dcap = run_i && pout && pclk && pcap;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pv <= 1'b0; pclk <= 1'b0; pcap <= 1'b0; ppay <= 5'd0; pcnt <= 8'd0;
        end else if (!run_i) begin
            pv <= 1'b0; pclk <= 1'b0; pcap <= 1'b0; ppay <= 5'd0; pcnt <= 8'd0;
        end else if (newd && !drop && !abort_i && !wr_tick_i) begin
            pv <= 1'b1; pclk <= cfg_dlyclk_i; pcap <= creq_cap_i;
            ppay <= c_del ? {3'd0, cpay_i} : dpay_i;
            pcnt <= 8'd0;
        end else if (pout || cancel) begin
            pv <= 1'b0;
        end else if (pv) begin
            pcnt <= (adv > 9'd255) ? 8'd255 : adv[7:0];
        end
    end

    // ---------------------------------------------------------------------
    // Environment
    //  env_rst  rst_n low in the first cycle
    //  env_cfg  cfg_dly_i, cfg_dlyclk_i, cfg_dlyps_i are stable while a
    //           change is pending (conventions.md: cfg_ stable while the
    //           block is active). Report reading R-D2: the lane can violate
    //           this (SETM right after a field end while its last delayed
    //           change still pends); the spec does not say what happens.
    // ---------------------------------------------------------------------
    wire [9:0] cfg = {cfg_dly_i, cfg_dlyclk_i, cfg_dlyps_i};
    generate
        if (STANDALONE) begin : g_env_assume
            always @* begin
                if ($initstate) begin
                    env_rst: assume (!rst_n);
                end
            end
            always @(posedge clk) begin
                if (!$initstate && pv) begin
                    env_cfg: assume (cfg == $past(cfg));
                end
            end
        end else begin : g_env_assert
            always @* begin
                if ($initstate) begin
                    env_rst: assert (!rst_n);
                end
            end
            always @(posedge clk) begin
                if (!$initstate && pv) begin
                    env_cfg: assert (cfg == $past(cfg));
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Exact behaviour
    // ---------------------------------------------------------------------
`ifndef DELAY_NO_MODEL_CHECK
    always @* begin
        a_m_dwe:  assert (d_we_o == mo_dwe);
        a_m_cwe:  assert (c_we_o == mo_cwe);
        a_m_dcap: assert (dcap_o == mo_dcap);
        if (mo_dwe) begin
            a_m_dpay: assert (d_pay_o == mo_dpay);
        end
        if (mo_cwe) begin
            a_m_cpay: assert (c_pay_o == mo_cpay);
        end
    end
`endif

    // ---------------------------------------------------------------------
    // Port-level tracker of the pending change: tp pending, tc its stream
    // (1 clock), tpay payload, tcap capture flag, tk cycles/strobes counted
    // in the cycles after the request (t0+1 .. t-1).
    // ---------------------------------------------------------------------
    logic       tp, tc, tcap;
    logic [4:0] tpay;
    logic [7:0] tk;
    wire  [8:0] tinc  = cfg_dlyps_i ? ((tk == 8'hFF) ? 9'd255 : {1'b0, tk} + {8'd0, pt_i})
                                    : {1'b0, tk} + 9'd1;
    wire        t_due = tinc >= {1'b0, cfg_dly_i};
    // cancel causes, from the ports
    wire        t_pin = pin_we_i && (tc ? pin_mask_i[3] : (pin_mask_i[0] || (pin_mask_i[1] && tpay[2])));
    wire        t_und = tc ? c_und : d_und;
    wire        t_can = abort_i || wr_tick_i || t_und || t_pin;
    // output of the pending change on its stream, as seen on the ports
    wire        t_we  = tc ? (c_we_o && !c_und) : (d_we_o && !d_und);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tp <= 1'b0; tc <= 1'b0; tcap <= 1'b0; tpay <= 5'd0; tk <= 8'd0;
        end else if (!run_i) begin
            tp <= 1'b0;
        end else if (newd && !drop && !abort_i && !wr_tick_i) begin
            tp <= 1'b1; tc <= c_del; tcap <= creq_cap_i;
            tpay <= c_del ? {3'd0, cpay_i} : dpay_i; tk <= 8'd0;
        end else if (tp && (t_can || t_due || newd)) begin   // newd here: a dropped request flushes
            tp <= 1'b0;
        end else if (tp) begin
            tk <= (tinc > 9'd255) ? 8'd255 : tinc[7:0];
        end
    end

    always @* begin
        if (rst_n && run_i) begin
            // 1. zero delay: outputs equal inputs in the same cycle
            if (dz) begin
                a_p1_zero: assert (d_we_o == dreq_i && c_we_o == creq_i && !dcap_o &&
                                   (!dreq_i || d_pay_o == dpay_i) && (!creq_i || c_pay_o == cpay_i));
            end
            // undelayed requests always pass through unchanged
            if (d_und) begin
                a_pass_d: assert (d_we_o && d_pay_o == dpay_i);
            end
            if (c_und) begin
                a_pass_c: assert (c_we_o && c_pay_o == cpay_i);
            end
            // a delayed request itself is never output in its own cycle;
            // a pending change of the other stream may be
            if (d_del && !(tp && !tc && !t_can)) begin
                a_p3_d_own: assert (!d_we_o);
            end
            if (c_del && !(tp && tc && !t_can)) begin
                a_p3_c_own: assert (!c_we_o);
            end
            // 2. exact delay: the pending change, not cancelled, is output
            //    exactly when its count reaches D (t0 + D raw, t0 (+) D
            //    prescaled), or earlier only when a new delayed request of
            //    its stream flushes it (3, order), with its own payload
            if (tp && !t_can) begin
                a_p2_exact: assert (t_we == (t_due || newd));
                if (t_we) begin
                    a_p3_order: assert (tc ? (c_pay_o == tpay[1:0]) : (d_pay_o == tpay));
                end
                // 5. dcap with the clock output of a capture change only
                a_p5_dcap: assert (dcap_o == (t_we && tc && tcap));
            end
            // 4. nothing is output after a cancel (or with nothing pending)
            //    except undelayed requests passing through
            if (!tp || t_can) begin
                a_p4_cancel: assert (d_we_o == d_und && c_we_o == c_und && !dcap_o);
            end
            // the other stream is never touched by the pending change
            if (tp && tc) begin
                a_p3_d_other: assert (d_we_o == d_und);
            end
            if (tp && !tc) begin
                a_p3_c_other: assert (c_we_o == c_und && !dcap_o);
            end
        end
        // halt: all outputs 0
        if (rst_n && !run_i) begin
            // ruling 45: while halted all outputs are 0 (payloads too)
            a_halt: assert (!d_we_o && !c_we_o && !dcap_o && d_pay_o == 5'd0 && c_pay_o == 2'd0);
        end
        if (!rst_n) begin
            a_rst: assert (!dcap_o && (d_we_o == (run_i && dreq_i && !(dreq_dly_i && !cfg_dlyclk_i && !dz))) &&
                           (c_we_o == (run_i && creq_i && !(creq_dly_i && cfg_dlyclk_i && !dz))));
        end
    end

    // ruling 58 (9.2 rule 4): a dropped request is never pending, so in the
    // cycle after a drop only undelayed requests pass; in the drop cycle a
    // pending change the PIN does not cancel is still output (rule 3 flush)
    always @(posedge clk) begin
        if (!$initstate && rst_n && run_i && $past(rst_n && run_i && drop && !abort_i && !wr_tick_i)) begin
            a_drop_gone: assert (d_we_o == d_und && c_we_o == c_und && !dcap_o);
        end
    end
    always @* begin
        if (rst_n && run_i && drop && tp && !t_can) begin
            a_drop_flush: assert (t_we);
        end
    end

    // PIN cancels a pending change only if it writes an output that change
    // writes (lane.md I22, 12.2 R9): a non-hitting PIN leaves it pending
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n && run_i && tp && pin_we_i && !t_pin &&
                                          !abort_i && !wr_tick_i && !t_und && !t_due && !newd)) begin
            a_pin_keep: assert (tp);
        end
    end

    // ---------------------------------------------------------------------
    // Covers
    // ---------------------------------------------------------------------
    logic f_ps, f_cap;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin f_ps <= 1'b0; f_cap <= 1'b0; end
        else if (newd) begin f_ps <= cfg_dlyps_i; f_cap <= c_del && creq_cap_i; end
    end
    generate
        if (STANDALONE) begin : g_cover
            always @(posedge clk) begin
                if (!$initstate && rst_n && run_i) begin
                    // raw delay of 3 on the data stream; on the clock stream
                    c_raw_d3:  cover (tp && !tc && t_we && !newd && cfg_dly_i == 8'd3);
                    c_raw_c3:  cover (tp &&  tc && t_we && !newd && cfg_dly_i == 8'd3);
                    // prescaled delay of 2 with gaps between strobes
                    c_ps:      cover (tp && t_we && !newd && f_ps && cfg_dly_i == 8'd2 &&
                                      $past(!pt_i) && $past(!pt_i, 2));
                    // a delayed capture-level clock change (dcap_o)
                    c_dcap:    cover (dcap_o && !newd);
                    // flush by a new delayed request
                    c_flush:   cover (tp && newd && t_we && !t_can);
                    // cancel by PIN (data via out0, data via out1 with w1,
                    // clock via out3); a PIN that does not hit keeps it
                    c_pin_d0:  cover (tp && !tc && pin_we_i && pin_mask_i == 4'b0001 && !abort_i && !wr_tick_i);
                    c_pin_d1:  cover (tp && !tc && pin_we_i && pin_mask_i == 4'b0010 && tpay[2]);
                    c_pin_c:   cover (tp &&  tc && pin_we_i && pin_mask_i == 4'b1000);
                    c_pin_keep: cover ($past(tp && !tc && pin_we_i && pin_mask_i == 4'b0110 && !tpay[2] && !t_due) &&
                                       t_we);
                    // ruling 58 drops: a clock request by a PIN on out3; a
                    // data request by a PIN on out0; a data request with w1
                    // by a PIN on out1 only; no drop for a data request
                    // without w1 and a PIN on out1 only; a drop that flushes
                    // a pending change the PIN does not cancel; a drop
                    // together with the PIN's cancel of the pending change
                    c_drop_c:   cover (c_del && pin_we_i && pin_mask_i == 4'b1000 && !abort_i && !wr_tick_i);
                    c_drop_d0:  cover (d_del && pin_we_i && pin_mask_i == 4'b0001 && !abort_i && !wr_tick_i);
                    c_drop_d1:  cover (d_del && dpay_i[2] && pin_we_i && pin_mask_i == 4'b0010 && !abort_i && !wr_tick_i);
                    c_nodrop_d1: cover ($past(d_del && !dpay_i[2] && pin_we_i && pin_mask_i == 4'b0010 &&
                                              !abort_i && !wr_tick_i && run_i) && tp);
                    c_drop_flush: cover (drop && tp && !t_can && t_we);
                    c_drop_cancel: cover (drop && tp && t_pin && !abort_i && !wr_tick_i);
                    // cancel by an undelayed request of the same stream
                    c_und_can: cover (tp && t_und && !pin_we_i && !abort_i && !wr_tick_i);
                    // an undelayed request and the other stream's output together
                    c_both:    cover (d_we_o && c_we_o && tp && t_we);
                    // zero delay passes through
                    c_zero:    cover (dz && dreq_i && dreq_dly_i && creq_i && creq_dly_i);
                    // environment: a pending change across many cycles with
                    // the configuration held
                    c_env:     cover (tp && t_we && cfg_dly_i == 8'd6 && !newd);
                end
            end
        end
    endgenerate
endmodule
