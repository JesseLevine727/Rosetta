`default_nettype none
// Four event detectors and the pending bits E0-E3 (docs/blocks/events.md;
// decisions.md rulings 10 and 21: trigger 0 and 5-7 off, bm = 3 never).
//
// The spec's per-detector state (win, cnt, done, lv) is kept in 7 flops:
//   cnt (6): window counter, win == (cnt != 0). A window opens with
//            cnt = K >= 1; the fire at cnt == 1 decrements it to 0, and a
//            cancel, a write or a halt zeroes it. Loading K = 0 is "fire
//            now, no window", so a trigger or an episode start just loads K.
//   x   (1): trigger 3: lv. Triggers 1 and 4: cond in the previous cycle
//            (run_i, no write). After a cycle with cond the spec has
//            win || done and after one without it both are 0, so
//            done == x && !win, and x == 0 implies win == 0 (an episode
//            starts when cond && !x). Trigger 2 and off: unused.
// The sharing is exact under events.md 4's assumption (EVCk changes only
// at the end of a cycle with evc_wr_i[k], which zeroes cnt and x).
// Off triggers and bm = 3 (triggers 1-3) never load and never keep a
// window, so they cannot fire whatever the state.
module rosetta_events (
    input  wire         clk, rst_n,
    input  wire         run_i,
    input  wire  [3:0]  in_i, in_q_i,
    input  wire         pt_i,
    input  wire  [63:0] cfg_evc_i,       // EVCk at [16k +: 16]
    input  wire  [3:0]  evc_wr_i,
    input  wire         cfg_stuffev_i,
    input  wire         stuff_viol_i,
    input  wire  [3:0]  ev_clr_i,
    output logic [3:0]  ev_pend_o,
    output logic [3:0]  fire_o
);
  logic [3:0] f, pend;

  for (genvar k = 0; k < 4; k++) begin : g_det
    logic [15:0] c;
    logic [5:0]  cnt, cnt_n;
    logic        x, x_n;
    logic        a, aq, b, bok, ml, t1, t2, t3, t4, win, trg, keep, load, clr;

    assign c   = cfg_evc_i[16*k +: 16];
    assign a   = in_i[c[1:0]];
    assign aq  = in_q_i[c[1:0]];
    assign b   = in_i[c[7:6]];
    assign bok = (c[9:8] == 2'd0) || (c[9:8] == 2'd1 && b) || (c[9:8] == 2'd2 && !b);
    assign t1  = c[5:3] == 3'd1;     // A at level L
    assign t2  = c[5:3] == 3'd2;     // A edge to L
    assign t3  = c[5:3] == 3'd3;     // A any edge
    assign t4  = c[5:3] == 3'd4;     // A and B both low
    assign ml  = (a == c[2]) && bok;
    assign win = cnt != 6'd0;

    // trg (edge triggers); keep: hc (2, 3) or cond (1, 4), 0 when off.
    assign trg  = (a != aq) && ((t2 && ml) || (t3 && bok));
    assign keep = t3 ? bok && (a == x) : (t1 || t2) ? ml : t4 && !a && !b;
    assign load = trg || ((t1 || t4) && keep && !x);
    assign clr  = !run_i || evc_wr_i[k];

    // A write still evaluates cycle t (may fire); a halted cycle never fires.
    assign fire_o[k] = run_i && (load ? c[15:10] == 6'd0 : keep && pt_i && cnt == 6'd1);

    // Zero on write, halt or cancel (with no window cnt is 0 already).
    always_comb begin
      if (clr || (!load && !keep)) cnt_n = 6'd0;
      else if (load)               cnt_n = c[15:10];
      else                         cnt_n = cnt - {5'd0, pt_i && win};
    end
    assign x_n = !clr && (t3 ? (trg ? a : x) : keep);

    always_ff @(posedge clk or negedge rst_n)
      if (!rst_n) begin cnt <= 6'd0; x <= 1'b0; end
      else begin cnt <= cnt_n; x <= x_n; end
  end

  // Pending bits: set wins over clear; 0 while halted.
  assign f = {fire_o[3] || (cfg_stuffev_i && stuff_viol_i), fire_o[2:0]};
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) pend <= 4'd0;
    else        pend <= {4{run_i}} & (f | (pend & ~ev_clr_i));
  assign ev_pend_o = pend;
endmodule
