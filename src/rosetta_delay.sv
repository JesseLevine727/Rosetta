`default_nettype none
// Launch delay unit (docs/blocks/tick.md section 9; lane.md I22, R9).
// Holds at most one postponed output change, of the data stream or of the
// clock stream (cfg_dlyclk_i selects which stream is delayed). Requests that
// are not delayed pass through in the same cycle.
//
// State: pv (pending), pclk (it belongs to the clock stream), ppay (payload:
// dpay layout {v0, e0, w1, v1, e1}, or {3'b0, cpay} for the clock), pcap
// (capture-level clock change), pcnt (cycles, or strobes when prescaled, in
// t0+1 .. t-1 after the request cycle t0).
module rosetta_delay (
    input  wire        clk, rst_n,
    input  wire        run_i, abort_i, wr_tick_i,
    input  wire  [7:0] cfg_dly_i,
    input  wire        cfg_dlyclk_i, cfg_dlyps_i,
    input  wire        pt_i,
    input  wire        dreq_i,
    input  wire  [4:0] dpay_i,
    input  wire        dreq_dly_i,
    input  wire        creq_i,
    input  wire  [1:0] cpay_i,
    input  wire        creq_dly_i, creq_cap_i,
    input  wire        pin_we_i,
    input  wire  [3:0] pin_mask_i,
    output logic       d_we_o,
    output logic [4:0] d_pay_o,
    output logic       c_we_o,
    output logic [1:0] c_pay_o,
    output logic       dcap_o
);
  logic       pv, pclk, pcap;
  logic [4:0] ppay;
  logic [7:0] pcnt;

  // Delayed requests: _dly flag, target stream, D != 0.
  logic ddel, cdel, newdel, dpass, cpass;
  assign ddel   = dreq_i && dreq_dly_i && !cfg_dlyclk_i && cfg_dly_i != 8'd0;
  assign cdel   = creq_i && creq_dly_i &&  cfg_dlyclk_i && cfg_dly_i != 8'd0;
  assign newdel = ddel || cdel;
  assign dpass  = dreq_i && !ddel;               // undelayed data request
  assign cpass  = creq_i && !cdel;               // undelayed clock request

  // 1. Maturity.
  logic [7:0] pstep, pnext;
  logic       mat;
  assign pstep = {7'd0, cfg_dlyps_i ? pt_i : 1'b1};
  assign pnext = (pcnt == 8'hFF) ? pcnt : pcnt + pstep;   // saturating
  assign mat   = pv && pnext >= cfg_dly_i;

  // 2. Cancel. w1 = ppay[2]: the pending data change also writes out1.
  logic cancel, pinhit;
  assign pinhit = pclk ? pin_mask_i[3] : (pin_mask_i[0] || (pin_mask_i[1] && ppay[2]));
  assign cancel = abort_i || wr_tick_i ||
                  (pv && (pclk ? cpass : dpass)) ||
                  (pin_we_i && pv && pinhit);

  // A delayed request made in the cycle a PIN writes an output that request
  // would write is dropped: the PIN wins (tick.md 9.2 rule 4, ruling 58).
  logic newhit, newdrop;
  assign newhit  = cdel ? pin_mask_i[3] : (pin_mask_i[0] || (pin_mask_i[1] && dpay_i[2]));
  assign newdrop = pin_we_i && newdel && newhit;

  // 3. Output.
  logic pout;
  assign pout = pv && !cancel && (mat || newdel);

  always_comb begin
    // run_i = 0: all outputs 0.
    d_we_o = 1'b0; d_pay_o = '0; c_we_o = 1'b0; c_pay_o = '0; dcap_o = 1'b0;
    if (run_i) begin
      d_we_o  = dpass || (pout && !pclk);
      c_we_o  = cpass || (pout &&  pclk);
      d_pay_o = (pout && !pclk) ? ppay      : dpay_i;
      c_pay_o = (pout &&  pclk) ? ppay[1:0] : cpay_i;
      dcap_o  = pout && pclk && pcap;
    end
  end

  // 4. Next state.
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      pv <= 1'b0; pclk <= 1'b0; ppay <= '0; pcap <= 1'b0; pcnt <= '0;
    end else if (!run_i) begin
      pv <= 1'b0; pclk <= 1'b0; ppay <= '0; pcap <= 1'b0; pcnt <= '0;
    end else if (newdel && !newdrop && !abort_i && !wr_tick_i) begin
      pv   <= 1'b1;
      pclk <= cdel;
      ppay <= cdel ? {3'd0, cpay_i} : dpay_i;
      pcap <= creq_cap_i;
      pcnt <= '0;
    end else if (pout || cancel)
      pv <= 1'b0;
    else
      pcnt <= pnext;

  // out2 is never written by a delayed change, so a PIN on it cancels nothing.
  logic unused_ok;
  assign unused_ok = &{1'b0, pin_mask_i[2]};
endmodule
