`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/stuff.md written by the formal
//  property writer, only to show that formal/stuff/*.sv can PASS and that
//  every property is non-vacuous (planted bugs below make tasks FAIL).
//  The real block is src/rosetta_stuff.sv, written independently.
//  Do not integrate, synthesise or copy this file.
//
//  Planted bugs (mutation testing), enabled one at a time with a define:
//    MUT_OFF_BY_ONE      stuff after N+1 bits instead of N
//    MUT_EQ_NO_RESTART   EQUAL: stuff bit does not count as 1st bit of next run
//    MUT_ONES_NO_CLEAR   ONES: a stuff bit does not clear the run
//    MUT_SB_POLARITY     stuff_bit_o inverted (1 in ONES, last in EQUAL)
//    MUT_VIOL_NEVER      violation_o stuck at 0
//    MUT_VIOL_ALWAYS     violation_o fires at every stuff position (no compare)
//    MUT_CLEAR_IGNORED   clear_i has no effect
//    MUT_CLEAR_LOSES     bit_valid_i wins over clear_i
//    MUT_N0_ENABLED      cfg_n_i == 0 not treated as disabled
//    MUT_NO_SATURATE     run counter wraps 7 -> 0 instead of saturating
//    MUT_EQ_FIRST_UNCOUNTED EQUAL: first bit of a frame does not count
//                        (run starts at 0 instead of 1 after clear)
//    MUT_DIS_VIOL        violation_o not gated by enable
//    MUT_VIOL_COMB       violation_o combinational, in the same cycle as the bit
//    MUT_IDLE_RESET      a cycle without bit_valid_i forgets the run (timing
//                        dependent: transmitter and receiver diverge)
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
    logic       hist_q;
    logic       sn_q;
    logic       viol_q;

`ifdef MUT_N0_ENABLED
    wire        on       = cfg_en_i;
`else
    wire        on       = cfg_en_i && (cfg_n_i != 3'd0);
`endif

`ifdef MUT_SB_POLARITY
    wire        sb       = cfg_mode_i ? last_q : 1'b1;
`else
    wire        sb       = cfg_mode_i ? ~last_q : 1'b0;
`endif

`ifdef MUT_NO_SATURATE
    wire  [2:0] run_inc  = run_q + 3'd1;
`else
    wire  [2:0] run_inc  = (run_q == 3'd7) ? 3'd7 : run_q + 3'd1;
`endif

    wire        hist_eff = hist_q;

    logic [2:0] run_d;
    always_comb begin
        if (!cfg_mode_i) begin
`ifdef MUT_ONES_NO_CLEAR
            if (sn_q)       run_d = run_q;
            else
`endif
            run_d = bit_i ? run_inc : 3'd0;
        end else begin
`ifdef MUT_EQ_NO_RESTART
            if (sn_q && bit_i != last_q) run_d = 3'd0;
            else
`endif
`ifdef MUT_EQ_FIRST_UNCOUNTED
            if (!hist_eff) run_d = 3'd0;
            else
`endif
            run_d = (hist_eff && bit_i == last_q) ? run_inc : 3'd1;
        end
    end

`ifdef MUT_OFF_BY_ONE
    wire [3:0] n_eff = {1'b0, cfg_n_i} + 4'd1;
`else
    wire [3:0] n_eff = {1'b0, cfg_n_i};
`endif

`ifdef MUT_CLEAR_IGNORED
    wire clr = 1'b0;
`else
    wire clr = clear_i;
`endif

`ifdef MUT_CLEAR_LOSES
    wire take = bit_valid_i;
    wire clr_eff = clr && !bit_valid_i;
`else
    wire take = bit_valid_i && !clr;
    wire clr_eff = clr;
`endif

`ifdef MUT_VIOL_NEVER
    wire viol_d = 1'b0;
`elsif MUT_VIOL_ALWAYS
    wire viol_d = sn_q;
`elsif MUT_DIS_VIOL
    wire viol_d = (run_q == cfg_n_i) && (bit_i != sb);
`else
    wire viol_d = sn_q && (bit_i != sb);
`endif

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run_q       <= 3'd0;
            last_q      <= 1'b0;
            hist_q      <= 1'b0;
            sn_q        <= 1'b0;
            viol_q      <= 1'b0;
        end else if (clr_eff) begin
            run_q       <= 3'd0;
            hist_q      <= 1'b0;
            sn_q        <= 1'b0;
            viol_q      <= 1'b0;
        end else if (take) begin
            run_q       <= run_d;
            if (cfg_mode_i) begin
                last_q  <= bit_i;
                hist_q  <= 1'b1;
            end
            sn_q        <= on && ({1'b0, run_d} == n_eff);
            viol_q      <= viol_d;
        end else begin
`ifdef MUT_IDLE_RESET
            run_q       <= 3'd0;
            hist_q      <= 1'b0;
            sn_q        <= 1'b0;
`endif
            viol_q      <= 1'b0;
        end
    end

    assign stuff_next_o = sn_q;
    assign stuff_bit_o  = sb;
`ifdef MUT_VIOL_COMB
    assign violation_o  = take && viol_d;
`else
    assign violation_o  = viol_q;
`endif

endmodule
