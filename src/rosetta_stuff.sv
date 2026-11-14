`default_nettype none

// rosetta_stuff: bit stuffing tracker (spec v2: docs/blocks/stuff.md).
//
// Watches the wire bits and says when the next wire bit must be a stuff bit
// and what its value must be. Used both by the transmitter (insert) and the
// receiver (drop and check).
//
// State: run_q (0-7), last_q, viol_q. 5 flops, all reset to 0.
// "No history" is run_q == 0 (spec v2); the EQUAL rule's "run != 0" test
// needs no logic because with run_q == 0 both branches give run = 1.
//
// All outputs are functions of the flops and the cfg_ inputs only; there is
// no path from clear_i, bit_valid_i or bit_i to any output.

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

  logic       active;   // stuffing in effect
  logic       take;     // a wire bit is consumed this cycle
  logic       differ;   // bit_i != stuff_bit_o
  logic       bad;      // a stuff position carries the wrong bit
  logic [2:0] run_inc;  // min(run + 1, 7)

  // N = 0, and N = 1 in EQUAL mode, behave as disabled.
  assign active = cfg_en_i && (cfg_n_i != 3'd0)
               && !(cfg_mode_i && (cfg_n_i == 3'd1));
  assign take   = bit_valid_i && !clear_i;

  // Outputs.
  assign stuff_next_o = active && (run_q == cfg_n_i);
  assign stuff_bit_o  = cfg_mode_i && !last_q;   // ONES: 0, EQUAL: ~last
  assign violation_o  = viol_q && active;

  // differ = bit_i != stuff_bit_o, which is also "the bit extends the run":
  //   ONES:  bit_i = 1        (else run = 0)
  //   EQUAL: bit_i = last     (else run = 1, a new run)
  assign differ  = bit_i ^ stuff_bit_o;
  assign bad     = stuff_next_o && differ;
  assign run_inc = (run_q == 3'd7) ? 3'd7 : run_q + 3'd1;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run_q  <= 3'd0;
      last_q <= 1'b0;
      viol_q <= 1'b0;
    end else begin
      viol_q <= take && bad;
      if (clear_i) begin
        run_q <= 3'd0;                     // last_q unchanged by clear
      end else if (bit_valid_i) begin
        if (bad) begin
          run_q <= 3'd1;                   // violating bit starts a new run
        end else if (differ) begin
          run_q <= run_inc;
        end else begin
          run_q <= {2'b00, cfg_mode_i};    // ONES: 0, EQUAL: 1
        end
        // EQUAL: last = bit_i on every bit (violating or not).
        // ONES: last is not updated.
        if (cfg_mode_i) begin
          last_q <= bit_i;
        end
      end
    end
  end

endmodule
