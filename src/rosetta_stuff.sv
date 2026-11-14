`default_nettype none

// rosetta_stuff: bit stuffing tracker (spec: docs/blocks/stuff.md).
//
// Watches the wire bits and says when the next wire bit must be a stuff bit
// and what its value must be. Used both by the transmitter (insert) and the
// receiver (drop and check).
//
// State: run_q (0-7), last_q, viol_q. 5 flops, all reset to 0.
// The spec's "no history" flag is not stored: it only exists right after
// clear/reset, when run_q = 0, and in that state the EQUAL rule's two
// branches agree (run + 1 = 1), so it never changes behaviour.
//
// All outputs are functions of the flops and the (static) cfg_ inputs only;
// there is no path from clear_i, bit_valid_i or bit_i to any output.

module rosetta_stuff (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cfg_en_i,
    input  wire  [2:0] cfg_n_i,
    input  wire        cfg_mode_i,    // 0 = ONES, 1 = EQUAL
    input  wire        clear_i,
    input  wire        bit_valid_i,
    input  wire        bit_i,
    output logic       stuff_next_o,
    output logic       stuff_bit_o,
    output logic       violation_o
);

  logic [2:0] run_q;
  logic       last_q;
  logic       viol_q;

  logic       active;   // stuffing in effect: enabled and N != 0
  logic       take;     // a wire bit is consumed this cycle
  logic       extend;   // the bit continues the current run
  logic [2:0] run_inc;  // run + 1, saturating at 7

  assign active = cfg_en_i && (cfg_n_i != 3'd0);
  assign take   = bit_valid_i && !clear_i;

  // Outputs.
  assign stuff_next_o = active && (run_q == cfg_n_i);
  assign stuff_bit_o  = cfg_mode_i && !last_q;   // ONES: 0, EQUAL: ~last
  // viol_q is only ever set while active; gating it again keeps the
  // "disabled => violation_o = 0" rule true even if cfg changes.
  assign violation_o  = viol_q && active;

  // extend = (bit_i != stuff_bit_o):
  //   ONES:  bit_i = 1        -> run + 1, else run = 0
  //   EQUAL: bit_i = last     -> run + 1, else run = 1 (new run; a correct
  //                              stuff bit is such a bit)
  // The same signal is "this bit is not the required stuff bit".
  assign extend  = bit_i ^ stuff_bit_o;
  assign run_inc = (run_q == 3'd7) ? 3'd7 : run_q + 3'd1;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run_q  <= 3'd0;
      last_q <= 1'b0;
      viol_q <= 1'b0;
    end else begin
      // Wrong stuff bit: a bit taken while stuff_next_o and != stuff_bit_o.
      viol_q <= take && stuff_next_o && extend;
      if (clear_i) begin
        run_q <= 3'd0;
      end else if (bit_valid_i) begin
        run_q <= extend ? run_inc : {2'b00, cfg_mode_i};
        // The spec updates last only in EQUAL mode; clear leaves it alone
        // (it is only read through stuff_bit_o, which is meaningful only
        // when stuff_next_o, i.e. run >= 1, i.e. after a bit since clear).
        if (cfg_mode_i) begin
          last_q <= bit_i;
        end
      end
    end
  end

endmodule
