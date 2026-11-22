`default_nettype none
// H1 prototype, minimal variant 2 (study): rosetta_mdpll_min.sv with the
// arithmetic reduced further (P >= 4 is a programming rule, no floor; the
// acquisition phase update is ph = base >>> 1, exact because o = 0 during
// acquisition so err = base; the window test is one addition and one
// comparison; "late" compares base with P/2; one 5P/8 comparison serves
// both the acquisition test and the UNLOCK counter stop). Otherwise as
// rosetta_mdpll_min.sv:
//   - one shared subtraction  base = ph - (P - 1) cycles  (= ph + 1 - P)
//     gives both the error and the next phase;
//   - acquisition needs only "interval >= 5P/8" (the miss rule rejects a
//     wrong start); in UNLOCK the phase stops counting at 5P/8, so it needs
//     no saturation;
//   - phase update: err/2 (a wire shift) during acquisition, one unit toward
//     the error once locked (sign only); offset: one unit per edge, sign only.
// Fixed point: one cycle = 2^F units; P = PERIOD[PI-1:0], below 4 acts as 4.
//
// State (all reset to 0 = UNLOCK): st (2), ph (signed, PI+2+F), o (signed,
// OI+1+F, DCD = 1 only), n (acquisition count), miss (1 bit: MISSMAX = 2).
//
// Per cycle with run_i, cfg_en_i and no TICK write (else everything clears):
//   inc  = pt_i ? 1 cycle : 0
//   UNLOCK: edge: ph = 0, and if ph + inc >= 5P/8: st = ACQ, o = 0, n = 0.
//           no edge: ph = (ph >= 5P/8) ? ph : ph + inc.
//   ACQ/LOCK: base = ph + inc - P; err = base +- o (rising: + o, falling: - o);
//     edge with |err| <= win (ACQ 3P/8, LOCK P/4):
//        ph = base - (ACQ ? err >>> 1 : sign(err));
//        LOCK: o += (rising ? sign(-err) : sign(err)), saturating;
//        n += 1 up to NACQ; ACQ -> LOCK when n reaches NACQ;
//        st_o in this cycle iff the state is LOCK.
//     else if ph + inc >= 3P/2 (no mid-bit edge for half a bit past the
//        prediction): ph = base; ACQ -> UNLOCK; LOCK: a second miss in a row
//        -> UNLOCK, else miss = 1.
//     else ph = ph + inc.
module rosetta_mdpll_min2 #(
    parameter int PI   = 4,
    parameter int F    = 3,
    parameter int OI   = 2,
    parameter int DCD  = 1,
    parameter int NACQ = 8,
    parameter int PFIX = 0,    // > 0: bit time fixed at PFIX prescaled cycles, PERIOD ignored
    parameter int OF   = -1    // offset fractional bits (step 2^-OF cycle); -1: same as F
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
  localparam int W  = PI + 2 + F;
  localparam int OFB = (OF < 0) ? F : OF;
  localparam int OW = OI + 1 + OFB;
  localparam int NW = $clog2(NACQ + 1);
  localparam logic [1:0] UNLOCK = 2'd0, ACQ = 2'd1, LOCK = 2'd2;

  logic [1:0]           state, state_n;
  logic signed [W-1:0]  ph, ph_n;
  logic signed [OW-1:0] o, o_n;
  logic [NW-1:0]        n, n_n;
  logic                 miss, miss_n, st_n;

  // constants (P has no fractional bits; P >= 4 by programming rule)
  logic signed [W-1:0] pq, w_trk, w_acq, a_lo, inc;
  assign pq     = (PFIX > 0) ? W'(PFIX << F) : $signed({2'b00, cfg_period_i[PI-1:0], {F{1'b0}}});
  assign w_trk  = pq >>> 2;
  assign w_acq  = (pq >>> 2) + (pq >>> 3);
  assign a_lo   = (pq >>> 1) + (pq >>> 3);
  assign inc    = $signed({{(W-F-1){1'b0}}, pt_i, {F{1'b0}}});

  logic signed [W-1:0] ph_inc, base, oext, err, win;
  logic signed [W:0]   wsum;
  logic                rising, acq_s, acc, long_iv, late;
  assign ph_inc  = ph + inc;
  assign base    = ph_inc - pq;
  assign rising  = dline_i;
  assign oext    = (DCD != 0) ? (W'(o) <<< (F - OFB)) : W'(0);
  assign err     = rising ? base + oext : base - oext;
  assign acq_s   = state == ACQ;
  assign win     = acq_s ? w_acq : w_trk;
  // |err| <= win  <=>  0 <= err + win <= 2 win
  assign wsum    = $signed({err[W-1], err}) + $signed({win[W-1], win});
  assign acc     = !wsum[W] && wsum <= $signed({win, 1'b0});
  assign long_iv = ph >= a_lo;
  assign late    = base >= (pq >>> 1);

  localparam logic signed [OW-1:0] OMAX = {1'b0, {(OW-1){1'b1}}};
  localparam logic signed [OW-1:0] OMIN = {1'b1, {(OW-1){1'b0}}};
  logic up, dn, epos, eneg;
  assign epos = !err[W-1] && err != W'(0);
  assign eneg = err[W-1];
  assign up = (DCD != 0) && !acq_s && (rising ? eneg : epos) && o != OMAX;
  assign dn = (DCD != 0) && !acq_s && (rising ? epos : eneg) && o != OMIN;

  always_comb begin
    state_n = state; ph_n = ph_inc; o_n = o; n_n = n; miss_n = miss; st_n = 1'b0;
    if (state == UNLOCK) begin
      if (dedge_i) begin
        ph_n = '0;
        if (long_iv) begin state_n = ACQ; o_n = '0; n_n = '0; miss_n = 1'b0; end
      end else if (long_iv)
        ph_n = ph;                                   // stop counting: no wrap
    end else if (dedge_i && acc) begin
      if (acq_s)     ph_n = base >>> 1;              // = base - err/2, err = base
      else if (epos) ph_n = base - W'(1);
      else if (eneg) ph_n = base + W'(1);
      else           ph_n = base;
      o_n    = up ? o + 1'b1 : dn ? o - 1'b1 : o;
      n_n    = (n == NW'(NACQ)) ? n : n + 1'b1;
      miss_n = 1'b0;
      st_n   = state == LOCK;
      if (acq_s && n + 1'b1 == NW'(NACQ)) state_n = LOCK;
    end else if (late) begin
      ph_n   = base;
      miss_n = 1'b1;
      if (acq_s || miss) state_n = UNLOCK;
    end
  end

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      state <= UNLOCK; ph <= '0; o <= '0; n <= '0; miss <= 1'b0;
    end else if (!run_i || wr_tick_i || !cfg_en_i) begin
      state <= UNLOCK; ph <= '0; o <= '0; n <= '0; miss <= 1'b0;
    end else begin
      state <= state_n; ph <= ph_n; o <= o_n; n <= n_n; miss <= miss_n;
    end

  assign st_o   = run_i && !abort_i && cfg_en_i && st_n;
  assign rxb_o  = dline_i ^ cfg_inv_i;
  assign lock_o = state == LOCK;
  assign acq_o  = state == ACQ;
endmodule
