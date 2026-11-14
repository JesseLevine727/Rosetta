`default_nettype none
// Tick source (docs/blocks/tick.md sections 1-8; decisions.md rulings 3, 4,
// 5, 6, 18, 22, 23, 28, 29). Generates the launch, capture and sample
// strobes in the four tick modes, the clock output requests (BAUD timer,
// EDGE stretch_hold, TICK-write idle), the RTRIP-delayed BAUD sample, the
// edge-interval counter and the operation timer.
//
// State (all reset to 0, all cleared while run_i = 0):
//   pre (9)                   free-running prescaler (3.1, ruling 22)
//   bt_run, bt_ph, lr, hw     BAUD timer (4.1)
//   cnt (16)                  strobes in the current BAUD phase; EDGE stretch
//                             release counter; RECOVER timer (one register)
//   spend, sarm, scnt         BAUD pending sample and RTRIP countdown (5).
//                             With the clock delayed, "waiting for the
//                             reference dcap" is spend && !sarm.
//   stretch, srel             EDGE stretch_hold (4.2)
//   twq                       cycle after a TICK write (3.3)
//   ecnt (16)                 edge interval (6)
//   ov, ow (12), oc (16)      operation timer (7): remaining strobes are
//                             (ow - 1) * period + oc, oc in 1..period
//
// Clock request arbitration (section 8 claims at most one per cycle): the
// TICK-write idle request wins; the only other request that can coincide
// with it is an EDGE stretch start, which is suppressed in that cycle so the
// stretch flop never rises without its low request.
module rosetta_tick #(
    parameter int RTRIP = 4                // lane.md 1; must be >= 2
) (
    input  wire         clk, rst_n,
    input  wire         run_i, abort_i, wr_tick_i,
    input  wire  [1:0]  cfg_mode_i, cfg_ps_i, cfg_clkin_i,
    input  wire         cfg_capfall_i, cfg_clkout_i, cfg_idle_i, cfg_clkod_i,
    input  wire         cfg_rest_i, cfg_honour_i, cfg_hold_i,
    input  wire  [15:0] cfg_period_i, cfg_period2_i,
    input  wire  [7:0]  cfg_thr0_i, cfg_pw0_i, cfg_pw1_i,
    input  wire         cfg_dlyclk_i,
    input  wire  [7:0]  cfg_dly_i,
    input  wire  [3:0]  in_i, in_q_i,
    input  wire         dline_i, dedge_i,
    input  wire         pw_bit_i,
    input  wire         cu_v_i, cu_fld_i, cu_tx_i, cu_smp_i, pk_v_i,
    input  wire         launch_req_i, fld_next_i, fld_first_i,
    input  wire         data_wait_i,
    input  wire         ot_start_i, ot_raw_i,
    input  wire  [15:0] ot_n_i,
    input  wire         dcap_i,
    output logic        pt_o,
    output logic        lt_o, ct_o, st_o, cte_o, hte_o,
    output logic        spend_o, bt_run_o, bt_stop_o, ot_done_o,
    output logic [15:0] d16_o,
    output logic        ge0_o,
    output logic        creq_o,
    output logic [1:0]  cpay_o,
    output logic        creq_dly_o, creq_cap_o,
    output logic        stretch_o
);
  localparam logic [1:0] BAUD = 2'd0, EDGE = 2'd1, RECOVER = 2'd2, PULSE = 2'd3;
  localparam int         SW   = $clog2(RTRIP);
  localparam logic [SW-1:0] SLAST = SW'(RTRIP - 1);

  logic [8:0]    pre;
  logic          bt_run, bt_ph, lr, hw;
  logic [15:0]   cnt, ecnt;
  logic          spend, sarm;
  logic [SW-1:0] scnt;
  logic          stretch, srel, twq;
  logic          ov;
  logic [11:0]   ow;
  logic [15:0]   oc;

  // ---- Section 2: definitions ---------------------------------------------
  logic cl, ll, il, cin, cinq, g, baudm, baudt;
  assign cl    = !cfg_capfall_i;                 // capture level CL
  assign ll    = cfg_capfall_i;                  // launch level LL = !CL
  assign il    = cfg_idle_i;                     // idle level IL
  assign cin   = in_i[cfg_clkin_i];
  assign cinq  = in_q_i[cfg_clkin_i];
  assign g     = run_i && !abort_i;              // gate G
  assign baudm = cfg_mode_i == BAUD || cfg_mode_i == PULSE;   // 4.1 rules apply
  assign baudt = cfg_mode_i == BAUD || (cfg_mode_i == PULSE && cu_tx_i);

  // ---- 3.1 Prescaler --------------------------------------------------------
  // pt = (ps == 0) || (pre & PM) == PM, PM = 7, 63, 511; pt_next is pt
  // evaluated on pre + 1.
  function automatic logic ptf(input logic [1:0] ps, input logic [8:0] p);
    case (ps)
      2'd0:    ptf = 1'b1;
      2'd1:    ptf = &p[2:0];
      2'd2:    ptf = &p[5:0];
      default: ptf = &p[8:0];
    endcase
  endfunction

  logic       pt, pt_next;
  logic [8:0] pre_inc;
  assign pre_inc = pre + 9'd1;
  assign pt      = ptf(cfg_ps_i, pre);
  assign pt_next = ptf(cfg_ps_i, pre_inc);
  assign pt_o    = pt;

  // ---- Periods and the shared counter -------------------------------------
  logic [15:0] pd, pr, hr, cnt1, inc;
  logic        ge1, ge2, ge_pd, ge_p2d, ge_pr;
  assign pd  = (cfg_period_i  == 16'd0) ? 16'd1 : cfg_period_i;   // Pd
  assign pr  = (cfg_period_i < 16'd2)   ? 16'd2 : cfg_period_i;   // RECOVER Pr
  assign hr  = pr >> 1;                                           // RECOVER H
  assign cnt1 = cnt + 16'd1;
  assign inc  = (pt && cnt != 16'hFFFF) ? cnt1 : cnt;             // sat16(cnt + pt)
  // The equality rules "pt && inc == H" use cnt1 = cnt + 1 for inc: they need
  // pt, and at cnt = 0xFFFF neither side can match (H is 1..0x7FFF or 1..255).
  // The "inc >= limit" compares, arranged for depth: the max() in Pd, P2d,
  // Pr is a separate floor term (inc >= max(P, f) <=> inc >= P && inc >= f),
  // and the late pt only selects between two compares made from cnt:
  // cnt + pt >= l <=> cnt >= l || (pt && cnt + 1 == l). This equals
  // sat16(cnt + pt) >= l (at cnt = 0xFFFF both sides are 1).
  function automatic logic ge_inc(input logic [15:0] c, input logic p, input logic [15:0] l);
    ge_inc = c >= l || (p && c + 16'd1 == l);
  endfunction
  assign ge1    = cnt != 16'd0 || pt;                        // inc >= 1
  assign ge2    = cnt[15:1] != 15'd0 || (cnt[0] && pt);      // inc >= 2
  assign ge_pd  = ge_inc(cnt, pt, cfg_period_i)  && ge1;     // inc >= Pd
  assign ge_p2d = ge_inc(cnt, pt, cfg_period2_i) && ge1;     // inc >= P2d
  assign ge_pr  = ge_inc(cnt, pt, cfg_period_i)  && ge2;     // inc >= Pr

  // ---- 4.1 BAUD timer (BAUD, and PULSE) -----------------------------------
  logic        b_lt, b_ct, b_cte, b_hte, b_stop, b_req, b_lvl, b_cap;
  logic        bt_run_n, bt_ph_n, lr_n, hw_n;
  logic        b_clr, b_adv;                 // next cnt: 0, inc, else hold
  logic [7:0]  hpw;
  assign hpw = pw_bit_i ? cfg_pw1_i : cfg_pw0_i;     // H for hte

  always_comb begin
    b_lt = 1'b0; b_ct = 1'b0; b_cte = 1'b0; b_hte = 1'b0; b_stop = 1'b0;
    b_req = 1'b0; b_lvl = 1'b0; b_cap = 1'b0;
    bt_run_n = bt_run; bt_ph_n = bt_ph; lr_n = lr; hw_n = hw; b_clr = 1'b0; b_adv = 1'b0;
    if (g && baudm) begin
      if (!bt_run) begin
        // Stopped.
        if (launch_req_i) begin
          b_lt = 1'b1; b_req = 1'b1; b_lvl = ll;
          bt_run_n = 1'b1; bt_ph_n = 1'b0; b_clr = 1'b1; lr_n = 1'b0; hw_n = 1'b0;
        end
      end else if (!bt_ph && !hw) begin
        // Phase 0: due = inc >= Pd; postponed while spend.
        b_cte = ge_pd && !spend;
        b_hte = pt && cnt1 == {8'd0, hpw} && hpw != 8'd0;   // inc == H
        if (b_cte) begin
          b_req = 1'b1; b_lvl = cl; b_cap = 1'b1;
          if (!cfg_honour_i) begin b_ct = 1'b1; bt_ph_n = 1'b1; b_clr = 1'b1; end
          else               hw_n = 1'b1;
        end else
          b_adv = 1'b1;
      end else if (!bt_ph) begin
        // Honour wait: pause until the clock input reads the capture level.
        if (cin == cl) begin
          b_ct = 1'b1; bt_ph_n = 1'b1; b_clr = 1'b1; hw_n = 1'b0;
        end
      end else begin
        // Phase 1: eob = lr || inc >= P2d.
        if (!(lr || ge_p2d))
          b_adv = 1'b1;
        else if (launch_req_i) begin
          b_lt = 1'b1; b_req = 1'b1; b_lvl = ll;
          bt_ph_n = 1'b0; b_clr = 1'b1; lr_n = 1'b0;
        end else if (fld_next_i)
          lr_n = 1'b1;                                   // hold
        else begin
          b_stop = 1'b1; b_req = 1'b1; b_lvl = cfg_rest_i ? ll : il;   // stop
          bt_run_n = 1'b0; lr_n = 1'b0;
        end
      end
    end
  end

  // ---- 5. BAUD sample: pending flag and RTRIP countdown -------------------
  logic dmode, smp_set, ref_now, bst;
  // The reference is the delayed capture-level clock change when the launch
  // delay applies to the clock output.
  assign dmode   = cfg_dlyclk_i && cfg_dly_i != 8'd0 && cfg_clkout_i;
  assign smp_set = baudt && !cfg_honour_i && b_ct && cu_smp_i;
  assign ref_now = baudt && !cfg_honour_i &&
                   (dmode ? (b_ct || (spend && !sarm)) && dcap_i : b_ct);
  assign bst     = g && spend && sarm && scnt == SLAST;     // cycle r + RTRIP
  // Every capture restarts the countdown (ruling 42): a capture that is not
  // its own reference (clock delayed, no dcap_i yet) drops an older running
  // count, so the wait for its reference (spend && !sarm) starts at once and
  // an older round trip can never give this capture's st.
  logic cap_wait;
  assign cap_wait = baudt && !cfg_honour_i && b_ct && !ref_now;

  // ---- 4.2 EDGE --------------------------------------------------------------
  logic el, ec, e_start, e_rel;
  logic e_clr, e_adv;
  assign el      = (cin != cinq) && cin == ll;
  assign ec      = (cin != cinq) && cin == cl;
  assign e_start = g && cfg_mode_i == EDGE && cfg_clkout_i && cfg_hold_i && !twq &&
                   !stretch && el && !cu_v_i && !pk_v_i && data_wait_i;
  assign e_rel   = g && srel && ge_pd;
  assign e_clr   = !srel && stretch && fld_first_i;          // release starts
  assign e_adv   = srel && !ge_pd;

  // ---- 4.3 RECOVER --------------------------------------------------------------
  logic r_lt, r_ct;
  assign r_lt = dedge_i || (pt && ge_pr);
  assign r_ct = !dedge_i && pt && cnt1 == hr;               // inc == H

  // Next value of the shared counter: a TICK write clears it; abort clears
  // it except in RECOVER, whose timer keeps running (and counts while idle).
  logic c_clr, c_adv;
  always_comb begin
    if (wr_tick_i)                     begin c_clr = 1'b1;  c_adv = 1'b0;  end
    else if (cfg_mode_i == RECOVER)    begin c_clr = r_lt;  c_adv = 1'b1;  end
    else if (abort_i)                  begin c_clr = 1'b1;  c_adv = 1'b0;  end
    else if (cfg_mode_i == EDGE)       begin c_clr = e_clr; c_adv = e_adv; end
    else                               begin c_clr = b_clr; c_adv = b_adv; end
  end

  // ---- 4 / 4.4 strobe outputs ---------------------------------------------------
  always_comb begin
    lt_o = 1'b0; ct_o = 1'b0; st_o = 1'b0; cte_o = 1'b0; hte_o = 1'b0;
    if (g) begin
      if (baudt) begin
        lt_o = b_lt; ct_o = b_ct; cte_o = b_cte; hte_o = b_hte;
        st_o = cfg_honour_i ? b_ct && cu_smp_i : bst;
      end else
        case (cfg_mode_i)
          EDGE:    begin lt_o = el || (stretch && fld_first_i); ct_o = ec; st_o = ec; cte_o = ec; end
          RECOVER: begin lt_o = r_lt; ct_o = r_ct; st_o = r_ct; cte_o = r_ct; end
          default: begin ct_o = dedge_i; st_o = dedge_i; end      // PULSE receive
        endcase
    end
  end
  assign bt_stop_o = b_stop;
  assign bt_run_o  = bt_run;
  assign spend_o   = spend;
  assign stretch_o = stretch;

  // ---- 8. Clock output requests -------------------------------------------------
  logic lvl;
  always_comb begin
    creq_o = 1'b0; lvl = 1'b0; creq_dly_o = 1'b0; creq_cap_o = 1'b0;
    if (cfg_clkout_i) begin
      if (g && twq)                 begin creq_o = 1'b1; lvl = il; end   // 3.3
      else if (abort_i && stretch)  begin creq_o = 1'b1; lvl = il; end   // not gated by G
      else if (b_req)               begin creq_o = 1'b1; lvl = b_lvl; creq_dly_o = 1'b1; creq_cap_o = b_cap; end
      else if (e_start)             begin creq_o = 1'b1; lvl = 1'b0; end
      else if (e_rel)               begin creq_o = 1'b1; lvl = il; end
    end
    cpay_o = cfg_clkod_i ? {1'b0, !lvl} : {lvl, 1'b1};        // map(lvl) = {v3, e3}
  end

  // ---- 6. Edge interval and classification ---------------------------------
  logic [7:0] d8;
  assign d16_o = ecnt;
  assign d8    = (ecnt[15:8] != 8'd0) ? 8'hFF : ecnt[7:0];
  assign ge0_o = d8 >= cfg_thr0_i;

  // ---- 7. Operation timer ---------------------------------------------------------
  // In the start cycle the timer acts on the new count, so a count that ends
  // at the first strobe (N = 1 and pt_next) gives ot_done in the start cycle.
  logic        o_on, o_last, o_step;
  logic [11:0] ow_e;
  logic [15:0] oc_e;
  always_comb begin
    if (ot_start_i) begin
      o_on = ot_raw_i || ot_n_i[11:0] != 12'd0;      // N = 0 never starts
      ow_e = ot_raw_i ? 12'd1 : ot_n_i[11:0];
      oc_e = !ot_raw_i ? pd : (ot_n_i == 16'd0) ? 16'd1 : ot_n_i;
    end else begin
      o_on = ov; ow_e = ow; oc_e = oc;
    end
  end
  assign o_last    = ow_e == 12'd1 && oc_e == 16'd1;  // one strobe left
  assign o_step    = o_on && pt_next;                 // strobe in the next cycle
  assign ot_done_o = g && o_step && o_last;

  // ---- Registers -------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      pre <= '0; bt_run <= 1'b0; bt_ph <= 1'b0; lr <= 1'b0; hw <= 1'b0; cnt <= '0;
      spend <= 1'b0; sarm <= 1'b0; scnt <= '0; stretch <= 1'b0; srel <= 1'b0;
      twq <= 1'b0; ecnt <= '0; ov <= 1'b0; ow <= '0; oc <= '0;
    end else if (!run_i) begin
      // 3.2 halt: every flop to its reset value.
      pre <= '0; bt_run <= 1'b0; bt_ph <= 1'b0; lr <= 1'b0; hw <= 1'b0; cnt <= '0;
      spend <= 1'b0; sarm <= 1'b0; scnt <= '0; stretch <= 1'b0; srel <= 1'b0;
      twq <= 1'b0; ecnt <= '0; ov <= 1'b0; ow <= '0; oc <= '0;
    end else begin
      pre <= pre_inc;
      twq <= wr_tick_i;

      // BAUD timer state.
      if (abort_i || wr_tick_i) begin
        bt_run <= 1'b0; bt_ph <= 1'b0; lr <= 1'b0; hw <= 1'b0;
      end else begin
        bt_run <= bt_run_n; bt_ph <= bt_ph_n; lr <= lr_n; hw <= hw_n;
      end

      // Shared counter.
      if (c_clr)      cnt <= '0;
      else if (c_adv) cnt <= inc;

      // Pending sample and countdown.
      if (abort_i || wr_tick_i) begin
        spend <= 1'b0; sarm <= 1'b0; scnt <= '0;
      end else begin
        if (smp_set)  spend <= 1'b1;
        else if (bst) spend <= 1'b0;
        if (ref_now) begin
          sarm <= 1'b1; scnt <= '0;
        end else if (cap_wait) begin
          sarm <= 1'b0;                      // ruling 42: discard the older count
        end else if (sarm) begin
          if (scnt == SLAST) sarm <= 1'b0;
          else               scnt <= scnt + 1'b1;
        end
      end

      // EDGE stretch_hold. A release in progress (srel) is not restarted by
      // another field start.
      if (abort_i || wr_tick_i) begin
        stretch <= 1'b0; srel <= 1'b0;
      end else if (srel) begin
        if (ge_pd) begin stretch <= 1'b0; srel <= 1'b0; end
      end else if (e_start)
        stretch <= 1'b1;
      else if (stretch && fld_first_i)
        srel <= 1'b1;

      // Edge interval counter: unaffected by abort and TICK writes.
      if (dedge_i)                        ecnt <= {15'd0, pt};
      else if (pt && ecnt != 16'hFFFF)    ecnt <= ecnt + 16'd1;

      // Operation timer: cancelled by abort (and by a new start).
      if (abort_i) begin
        ov <= 1'b0; ow <= '0; oc <= '0;
      end else begin
        ov <= o_on && !(o_step && o_last);
        if (o_step && oc_e == 16'd1) begin ow <= ow_e - 12'd1; oc <= pd; end
        else if (o_step)             begin ow <= ow_e;         oc <= oc_e - 16'd1; end
        else                         begin ow <= ow_e;         oc <= oc_e; end
      end
    end

  // Spec ports not used by any rule: dline_i (only dedge_i is), cu_fld_i
  // (an assumption on fld_first_i/launch_req_i, not an input to a rule).
  logic unused_ok;
  assign unused_ok = &{1'b0, dline_i, cu_fld_i};
endmodule
