`default_nettype none
// H1 prototype (study, not part of the chip): phase-tracking Manchester
// receive decoder. Standalone; meant to sit inside rosetta_tick next to the
// RECOVER timer and to replace RECOVER's capture strobe when enabled.
//
// Inputs are what the lane already has: dline_i/dedge_i are rosetta_coding's
// cleaned data level and edge (coding.md 3.1), pt_i the prescale strobe
// (tick.md 3.1), cfg_period_i the PERIOD register (one bit time, in
// prescaled cycles). cfg_en_i is the decoded enable (TICK.mode == RECOVER
// with the new TICK bit set). Output st_o is the capture/sample strobe that
// rosetta_field takes as a bit (one per accepted mid-bit edge, while locked);
// the bit value is the logical level after that edge, which rosetta_coding's
// level decode already gives (rxb = lvl); rxb_o is provided for the study
// testbench only.
//
// Algorithm, fixed point with F fractional bits (one cycle = 2^F units):
//   ph  time since the estimated last mid-bit instant (signed; in UNLOCK it
//       is the time since the last edge)
//   o   duty-cycle offset: rising mid-bit edges are expected at P - o,
//       falling ones at P + o (DCD = 1 only)
//   UNLOCK: an edge whose preceding interval is 5P/8 .. 11P/8 (about one
//       bit, mid-bit to mid-bit, allowing for duty-cycle distortion: the
//       preamble has only such edges) starts ACQ with ph = 0, o = 0, n = 0.
//   ACQ / LOCK, at an edge: err = (ph + 1 cycle) - (P -+ o). |err| <= win:
//       mid-bit edge: ph = ph + 1 - P - round(err * g), o += round(-+err * go),
//       n = min(n + 1, NMAX); ACQ -> LOCK when n reaches NACQ; st_o = 1 if the
//       state was LOCK. Otherwise a boundary edge: no change.
//   ACQ / LOCK, ph + 1 reaching 3P/2 without a mid-bit edge: ph -= P,
//       miss += 1; ACQ -> UNLOCK at once, LOCK -> UNLOCK at MISSMAX misses.
//   Gain schedule by n (STAGES = 4): n < NACQ: win 3P/8, g 1/2, go 0;
//       < 24: win 3P/8, g 1/8, go 1/16; < 40: win 3P/8, g 1/32, go 1/32;
//       else win P/4, g 2^-GT, go 2^-GO. STAGES = 2: n < NACQ as above, then
//       win P/4, g 2^-GT, go 2^-GO.
// Reset values 0 (UNLOCK). Halt (run_i = 0), a TICK write and !cfg_en_i
// clear the state; abort does not (like the RECOVER timer), it only gates
// the strobe.
module rosetta_mdpll #(
    parameter int PI      = 8,   // period integer bits: P = PERIOD[PI-1:0], below 4 acts as 4
    parameter int F       = 8,   // fractional bits
    parameter int DCD     = 1,   // separate rising / falling offset
    parameter int STAGES  = 4,   // gain schedule, 4 or 2
    parameter int GT      = 6,   // tracking phase gain 2^-GT
    parameter int GO      = 6,   // tracking offset gain 2^-GO
    parameter int NACQ    = 8,   // accepted edges before bits are given out
    parameter int MISSMAX = 2    // misses (bits without a mid-bit edge) that unlock
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
  localparam int IW = PI + 2;            // signed integer bits of ph (holds -P .. 2P)
  localparam int W  = IW + F;
  localparam int OW = PI + F;            // signed offset width
  localparam int NW = 6;
  localparam logic [1:0] UNLOCK = 2'd0, ACQ = 2'd1, LOCK = 2'd2;

  logic [1:0]          state, state_n;
  logic signed [W-1:0] ph, ph_n;
  logic signed [OW-1:0] o, o_n;
  logic [NW-1:0]       n, n_n;
  logic [1:0]          miss, miss_n;
  logic                st_n;

  // ---- constants from PERIOD ------------------------------------------------
  logic [PI-1:0]       praw, pint;
  logic signed [W-1:0] pq, w_trk, w_acq, t_miss, a_lo, a_hi, one;
  assign praw  = cfg_period_i[PI-1:0];
  assign pint  = (praw < PI'(4)) ? PI'(4) : praw;
  assign pq    = W'(pint) <<< F;
  assign w_trk = pq >>> 2;
  assign w_acq = (pq >>> 2) + (pq >>> 3);
  assign t_miss = pq + (pq >>> 1);
  assign a_lo  = (pq >>> 1) + (pq >>> 3);              // 5P/8
  assign a_hi  = pq + (pq >>> 2) + (pq >>> 3);         // 11P/8
  assign one   = pt_i ? (W'(1) <<< F) : W'(0);

  // ph + one cycle, saturating at the largest positive value (only reachable
  // in UNLOCK, where ph counts the time since the last edge). All signed.
  localparam logic signed [W-1:0] PHMAX = {1'b0, {(W-1){1'b1}}};
  logic signed [W:0]   ph_sum, phmax_x;
  logic signed [W-1:0] ph_inc;
  assign ph_sum  = $signed({ph[W-1], ph}) + $signed({one[W-1], one});
  assign phmax_x = $signed({1'b0, PHMAX});
  assign ph_inc  = (ph_sum > phmax_x) ? PHMAX : ph_sum[W-1:0];

  // ---- stage-dependent window and gains ------------------------------------
  logic s_acq, s_mid1, s_mid2;
  assign s_acq  = n < NW'(NACQ);
  assign s_mid1 = (STAGES == 4) && !s_acq && n < NW'(24);
  assign s_mid2 = (STAGES == 4) && !s_acq && !s_mid1 && n < NW'(40);

  logic signed [W-1:0] win;
  assign win = (s_acq || s_mid1 || s_mid2) ? w_acq : w_trk;

  // ---- error at an edge -----------------------------------------------------
  logic                rising;
  logic signed [W-1:0] oext, expv, err, aerr;
  assign rising = dline_i;                         // physical level after the edge
  assign oext   = (DCD != 0) ? W'(o) : W'(0);     // sign-extended
  assign expv   = rising ? pq - oext : pq + oext;
  assign err    = ph_inc - expv;
  assign aerr   = err[W-1] ? -err : err;
  logic acc;
  assign acc = aerr <= win;

  // round(err * 2^-s) for the scheduled s
  function automatic logic signed [W-1:0] rsh(input logic signed [W-1:0] x, input integer s);
    rsh = (x + (W'(1) <<< (s - 1))) >>> s;
  endfunction
  logic signed [W-1:0] corr, ocorr, oerr;
  always_comb begin
    if (s_acq)       corr = rsh(err, 1);
    else if (s_mid1) corr = rsh(err, 3);
    else if (s_mid2) corr = rsh(err, 5);
    else             corr = rsh(err, GT);
  end
  assign oerr = rising ? -err : err;
  always_comb begin
    if (DCD == 0 || s_acq) ocorr = '0;
    else if (s_mid1)       ocorr = rsh(oerr, 4);
    else if (s_mid2)       ocorr = rsh(oerr, 5);
    else                   ocorr = rsh(oerr, GO);
  end
  // offset update, saturating to OW bits
  localparam logic signed [OW-1:0] OMAX = {1'b0, {(OW-1){1'b1}}};
  localparam logic signed [OW-1:0] OMIN = {1'b1, {(OW-1){1'b0}}};
  logic signed [W:0] o_sum, omax_x, omin_x;
  assign o_sum  = $signed({{(W+1-OW){o[OW-1]}}, o}) + $signed({ocorr[W-1], ocorr});
  assign omax_x = $signed({{(W+1-OW){1'b0}}, OMAX});
  assign omin_x = $signed({{(W+1-OW){1'b1}}, OMIN});
  logic signed [OW-1:0] o_upd;
  assign o_upd = (o_sum > omax_x) ? OMAX : (o_sum < omin_x) ? OMIN : o_sum[OW-1:0];

  // ---- next state -------------------------------------------------------------
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
      o_n    = o_upd;
      n_n    = (n == NW'(40)) ? n : n + NW'(1);
      miss_n = '0;
      st_n   = state == LOCK;
      if (state == ACQ && n + NW'(1) >= NW'(NACQ)) state_n = LOCK;
    end else if (ph_inc >= t_miss) begin
      ph_n   = ph_inc - pq;
      miss_n = (miss == 2'd3) ? miss : miss + 2'd1;
      if (state == ACQ || miss + 2'd1 >= 2'(MISSMAX)) state_n = UNLOCK;
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
