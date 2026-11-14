`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/stuff.md (spec v2) written by the
//  formal property writer, only to show that formal/stuff/*.sv can PASS and
//  that every property is non-vacuous (planted bugs below make tasks FAIL).
//  The real block is src/rosetta_stuff.sv, written independently.
//  Do not integrate, synthesise or copy this file.
//
//  Planted bugs (mutation testing), enabled one at a time with a define:
//    MUT_OFF_BY_ONE         stuff after N+1 bits instead of N
//    MUT_EQ_NO_RESTART      EQUAL: a correct stuff bit does not count as the
//                           first bit of the next run (run = 0, not 1)
//    MUT_ONES_NO_CLEAR      ONES: a correct stuff bit does not clear the run
//    MUT_SB_POLARITY        stuff_bit_o inverted (1 in ONES, last in EQUAL)
//    MUT_VIOL_NEVER         violation_o stuck at 0
//    MUT_VIOL_ALWAYS        violation at every stuff position (no compare)
//    MUT_CLEAR_IGNORED      clear_i has no effect
//    MUT_CLEAR_LOSES        bit_valid_i wins over clear_i
//    MUT_N0_ENABLED         cfg_n_i == 0 not treated as disabled
//    MUT_EQ_N1_ENABLED      EQUAL with N = 1 not treated as disabled (v1)
//    MUT_VIOL_NO_RESTART    a violating bit does not restart the run (v1:
//                           it extends the run, saturating at 7)
//    MUT_NO_SATURATE        run counter wraps 7 -> 0 instead of saturating
//    MUT_EQ_FIRST_UNCOUNTED EQUAL: first bit of a frame does not count
//    MUT_DIS_VIOL           violation computed and output without the
//                           active gate
//    MUT_VIOL_COMB          violation_o combinational, same cycle as the bit
//    MUT_IDLE_RESET         a cycle without bit_valid_i forgets the run
//    MUT_REG_STUFF          stuff_next_o is a flop computed at the previous
//                           edge (v1 style), so it ignores a cfg change in
//                           the clear cycle
// ===========================================================================
module rosetta_stuff (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       cfg_en_i,
    input  logic [2:0] cfg_n_i,
    input  logic       cfg_mode_i,     // 0 = ONES, 1 = EQUAL
    input  logic       clear_i,
    input  logic       bit_valid_i,
    input  logic       bit_i,
    output logic       stuff_next_o,
    output logic       stuff_bit_o,
    output logic       violation_o
);
    logic [2:0] run_q;
    logic       last_q;
    logic       viol_q;

    // ---- active -----------------------------------------------------------
`ifdef MUT_N0_ENABLED
    wire active = cfg_en_i && !(cfg_mode_i && cfg_n_i == 3'd1);
`elsif MUT_EQ_N1_ENABLED
    wire active = cfg_en_i && (cfg_n_i != 3'd0);
`else
    wire active = cfg_en_i && (cfg_n_i != 3'd0) && !(cfg_mode_i && cfg_n_i == 3'd1);
`endif

`ifdef MUT_OFF_BY_ONE
    wire [3:0] n_eff = {1'b0, cfg_n_i} + 4'd1;
`else
    wire [3:0] n_eff = {1'b0, cfg_n_i};
`endif

`ifdef MUT_SB_POLARITY
    wire sb = cfg_mode_i ? last_q : 1'b1;
`else
    wire sb = cfg_mode_i ? ~last_q : 1'b0;
`endif

    wire sn_comb = active && ({1'b0, run_q} == n_eff);

`ifdef MUT_NO_SATURATE
    wire [2:0] run_inc = run_q + 3'd1;
`else
    wire [2:0] run_inc = (run_q == 3'd7) ? 3'd7 : run_q + 3'd1;
`endif

    // ---- bad (violating) bit ------------------------------------------------
    wire bad = sn_comb && (bit_i != sb);     // drives the run update
`ifdef MUT_VIOL_ALWAYS
    wire bad_v = sn_comb;
`elsif MUT_DIS_VIOL
    wire bad_v = ({1'b0, run_q} == n_eff) && (bit_i != sb);
`else
    wire bad_v = bad;
`endif

    // ---- next run -------------------------------------------------------------
    logic [2:0] run_d;
    always_comb begin
        if (bad) begin
`ifdef MUT_VIOL_NO_RESTART
            run_d = run_inc;
`else
            run_d = 3'd1;
`endif
        end else if (!cfg_mode_i) begin
`ifdef MUT_ONES_NO_CLEAR
            if (sn_comb) run_d = run_q;
            else
`endif
            run_d = bit_i ? run_inc : 3'd0;
        end else begin
`ifdef MUT_EQ_NO_RESTART
            if (sn_comb) run_d = 3'd0;
            else
`endif
`ifdef MUT_EQ_FIRST_UNCOUNTED
            if (run_q == 3'd0) run_d = 3'd0;
            else
`endif
            run_d = (run_q != 3'd0 && bit_i == last_q) ? run_inc : 3'd1;
        end
    end

`ifdef MUT_CLEAR_IGNORED
    wire clr = 1'b0;
`else
    wire clr = clear_i;
`endif
`ifdef MUT_CLEAR_LOSES
    wire take    = bit_valid_i;
    wire clr_eff = clr && !bit_valid_i;
`else
    wire take    = bit_valid_i && !clr;
    wire clr_eff = clr;
`endif

`ifdef MUT_VIOL_NEVER
    wire viol_d = 1'b0;
`else
    wire viol_d = bad_v;
`endif

    logic sn_q;   // only used by MUT_REG_STUFF
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run_q  <= 3'd0;
            last_q <= 1'b0;
            viol_q <= 1'b0;
            sn_q   <= 1'b0;
        end else if (clr_eff) begin
            run_q  <= 3'd0;
            viol_q <= 1'b0;
            sn_q   <= 1'b0;
        end else if (take) begin
            run_q  <= run_d;
            if (cfg_mode_i) last_q <= bit_i;
            viol_q <= viol_d;
            sn_q   <= active && ({1'b0, run_d} == n_eff);
        end else begin
`ifdef MUT_IDLE_RESET
            run_q  <= 3'd0;
            sn_q   <= 1'b0;
`endif
            viol_q <= 1'b0;
        end
    end

`ifdef MUT_REG_STUFF
    assign stuff_next_o = sn_q;
`else
    assign stuff_next_o = sn_comb;
`endif
    assign stuff_bit_o = sb;
`ifdef MUT_VIOL_COMB
    assign violation_o = take && bad_v;
`elsif MUT_DIS_VIOL
    assign violation_o = viol_q;
`else
    assign violation_o = viol_q && active;
`endif

endmodule
