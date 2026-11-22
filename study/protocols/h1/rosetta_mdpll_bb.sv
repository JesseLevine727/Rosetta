`default_nettype none
// H1 prototype, compact variant (study): phase-tracking Manchester receive
// decoder with sign-only ("bang-bang") phase and duty-cycle updates.
// Module name rosetta_mdpll_bb; same ports as rosetta_mdpll.sv.
//
// Fixed point: one cycle = 2^F units. P = PERIOD[PI-1:0], below 4 acts as 4.
//   ph   signed: time since the estimated last mid-bit instant (in UNLOCK:
//        time since the last edge, saturating)
//   o    signed: duty-cycle offset; rising mid-bit edges are expected at
//        P - o, falling ones at P + o (DCD = 1 only)
// UNLOCK: an edge after an interval of 5P/8 .. 11P/8 starts ACQ: ph = 0,
//   o = 0, n = 0 (the preamble has only mid-bit edges, one bit apart).
// ACQ, LOCK: at an edge, err = (ph + 1 cycle) - (P -+ o).
//   |err| <= win (3P/8 in ACQ, P/4 in LOCK): a mid-bit edge.
//     ACQ:  ph = ph + 1 - P - (err >>> 1)        (gain 1/2; o stays 0)
//     LOCK: ph = ph + 1 - P - sign(err) * 1 unit; o = o +- 1 unit toward
//           the error of this edge's polarity (rising: o += sign(-err),
//           falling: o += sign(err)), saturating
//     n = min(n + 1, NACQ); ACQ -> LOCK when n reaches NACQ.
//     st_o = 1 in this cycle if the state was LOCK.
//   otherwise (boundary edge): no change.
//   no mid-bit edge by ph + 1 >= 3P/2: ph -= P, miss += 1; ACQ unlocks at
//   once, LOCK after MISSMAX misses in a row.
// Halt, a TICK write and !cfg_en_i clear the state; abort only gates st_o.
module rosetta_mdpll_bb #(
    parameter int PI      = 5,    // P <= 2^PI - 1 prescaled cycles
    parameter int F       = 3,    // fractional bits
    parameter int OI      = 2,    // offset integer bits (|o| < 2^OI cycles)
    parameter int DCD     = 1,
    parameter int NACQ    = 8,
    parameter int MISSMAX = 2
) (
    input  wire         clk, rst_n,
    input  wire         run_i, abort_i, wr_tick_i,
    input  wire         cfg_en_i,
    input  wire  [15:0] cfg_period_i,
    input  wire         cfg_inv_i,
    input  wire         pt_i,
    input  wire         dline_i, dedge_i,
    output logic        st_o,
    output logic        rxb_o,
    output logic        lock_o, acq_o
);
  localparam int W  = PI + 2 + F;          // ph: signed, holds -P .. 2P
  localparam int OW = OI + 1 + F;          // o: signed
  localparam int NW = $clog2(NACQ + 1);
  localparam logic [1:0] UNLOCK = 2'd0, ACQ = 2'd1, LOCK = 2'd2;

  logic [1:0]           state, state_n;
  logic signed [W-1:0]  ph, ph_n;
  logic signed [OW-1:0] o, o_n;
  logic [NW-1:0]        n, n_n;
  logic [1:0]           miss, miss_n;
  logic                 st_n;

  // constants from PERIOD
  logic [PI-1:0]       praw, pint;
  logic signed [W-1:0] pq, w_trk, w_acq, t_miss, a_lo, a_hi, unit1;
  assign praw   = cfg_period_i[PI-1:0];
  assign pint   = (praw < PI'(4)) ? PI'(4) : praw;
  assign pq     = $signed({2'b00, pint, {F{1'b0}}});
  assign w_trk  = pq >>> 2;
  assign w_acq  = (pq >>> 2) + (pq >>> 3);
  assign t_miss = pq + (pq >>> 1);
  assign a_lo   = (pq >>> 1) + (pq >>> 3);
  assign a_hi   = pq + (pq >>> 2) + (pq >>> 3);
  assign unit1  = $signed({{(W-1){1'b0}}, 1'b1});

  // ph + one cycle (only when pt_i), saturating at the largest value
  localparam logic signed [W-1:0] PHMAX = {1'b0, {(W-1){1'b1}}};
  logic signed [W-1:0] ph_inc;
  logic                sat;
  assign sat    = pt_i && (ph > PHMAX - (unit1 <<< F));
  assign ph_inc = sat ? PHMAX : (pt_i ? ph + (unit1 <<< F) : ph);

  // error at an edge
  logic                rising, acq_s;
  logic signed [W-1:0] oext, expv, err, win;
  assign rising = dline_i;
  assign oext   = (DCD != 0) ? W'(o) : W'(0);
  assign expv   = rising ? pq - oext : pq + oext;
  assign err    = ph_inc - expv;
  assign acq_s  = state == ACQ;
  assign win    = acq_s ? w_acq : w_trk;
  logic acc;
  assign acc = (err <= win) && (err >= -win);

  // phase correction: err/2 in ACQ, one unit toward err in LOCK
  // (written as statements: a '0 in a ?: chain would make the shift unsigned)
  logic signed [W-1:0] corr;
  always_comb begin
    if (acq_s)        corr = err >>> 1;
    else if (err > 0) corr = unit1;
    else if (err < 0) corr = -unit1;
    else              corr = unit1 - unit1;
  end
  // offset step: rising edges move o toward -err, falling toward +err
  localparam logic signed [OW-1:0] OMAX = {1'b0, {(OW-1){1'b1}}};
  localparam logic signed [OW-1:0] OMIN = {1'b1, {(OW-1){1'b0}}};
  logic up, dn;
  assign up = (DCD != 0) && !acq_s && (rising ? err < 0 : err > 0) && o != OMAX;
  assign dn = (DCD != 0) && !acq_s && (rising ? err > 0 : err < 0) && o != OMIN;

  logic in_iv;
  assign in_iv = ph_inc >= a_lo && ph_inc <= a_hi;

  always_comb begin
    state_n = state; ph_n = ph_inc; o_n = o; n_n = n; miss_n = miss; st_n = 1'b0;
    if (state == UNLOCK) begin
      if (dedge_i) begin
        ph_n = '0;
        if (in_iv) begin state_n = ACQ; o_n = '0; n_n = '0; miss_n = '0; end
      end
    end else if (dedge_i && acc) begin
      ph_n   = ph_inc - pq - corr;
      o_n    = up ? o + 1'b1 : dn ? o - 1'b1 : o;
      n_n    = (n == NW'(NACQ)) ? n : n + 1'b1;
      miss_n = '0;
      st_n   = state == LOCK;
      if (acq_s && n + 1'b1 == NW'(NACQ)) state_n = LOCK;
    end else if (ph_inc >= t_miss) begin
      ph_n   = ph_inc - pq;
      miss_n = (miss == 2'd3) ? miss : miss + 2'd1;
      if (acq_s || miss + 2'd1 >= 2'(MISSMAX)) state_n = UNLOCK;
    end
  end

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      state <= UNLOCK; ph <= '0; o <= '0; n <= '0; miss <= '0;
    end else if (!run_i || wr_tick_i || !cfg_en_i) begin
      state <= UNLOCK; ph <= '0; o <= '0; n <= '0; miss <= '0;
    end else begin
      state <= state_n; ph <= ph_n; o <= o_n; n <= n_n; miss <= miss_n;
    end

  assign st_o   = run_i && !abort_i && cfg_en_i && st_n;
  assign rxb_o  = dline_i ^ cfg_inv_i;
  assign lock_o = state == LOCK;
  assign acq_o  = state == ACQ;
endmodule
