`default_nettype none
// ===========================================================================
//  Proof top for rosetta_events.
//
//  One rosetta_events (u_dut) with every input free, checked by events_props
//  (exact behaviour of fire_o and ev_pend_o in every cycle, the required
//  properties of events.md section 4 and the I2C case of section 5 stated
//  on the ports, covers). rst_n is low in the first cycle (assumed in
//  events_props) and free after. Flops start at arbitrary values (no
//  initial values anywhere).
//
//  Required property 6 (independence), INDEP = 1: a second copy u_dut2
//  shares every input except cfg_evc_i and evc_wr_i, which are free
//  (cfg2 / evc2_wr, under the same environment rule). For each detector
//  k: while EVCk and
//  evc_wr_i[k] have been equal in both copies in every cycle since the last
//  reset, fire_o[k] and ev_pend_o[k] are equal in both copies, whatever the
//  other three configurations and write pulses do.
//
//  Spec: docs/blocks/events.md, docs/decisions.md rulings 10 and 21.
// ===========================================================================

module events_fv #(
    parameter INDEP = 1
) (
    input wire        clk,
    input wire        rst_n,
    input wire        run_i,
    input wire [3:0]  in_i,
    input wire [3:0]  in_q_i,
    input wire        pt_i,
    input wire [63:0] cfg_evc_i,
    input wire [3:0]  evc_wr_i,
    input wire        cfg_stuffev_i,
    input wire        stuff_viol_i,
    input wire [3:0]  ev_clr_i,
    input wire [63:0] cfg2_evc_i,     // second copy (independence) only
    input wire [3:0]  evc2_wr_i
);
    wire [3:0] ev_pend_o, fire_o;

    rosetta_events u_dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .run_i         (run_i),
        .in_i          (in_i),
        .in_q_i        (in_q_i),
        .pt_i          (pt_i),
        .cfg_evc_i     (cfg_evc_i),
        .evc_wr_i      (evc_wr_i),
        .cfg_stuffev_i (cfg_stuffev_i),
        .stuff_viol_i  (stuff_viol_i),
        .ev_clr_i      (ev_clr_i),
        .ev_pend_o     (ev_pend_o),
        .fire_o        (fire_o)
    );

    events_props #(.STANDALONE(1), .COVERS(1)) u_props (
        .clk           (clk),
        .rst_n         (rst_n),
        .run_i         (run_i),
        .in_i          (in_i),
        .in_q_i        (in_q_i),
        .pt_i          (pt_i),
        .cfg_evc_i     (cfg_evc_i),
        .evc_wr_i      (evc_wr_i),
        .cfg_stuffev_i (cfg_stuffev_i),
        .stuff_viol_i  (stuff_viol_i),
        .ev_clr_i      (ev_clr_i),
        .ev_pend_o     (ev_pend_o),
        .fire_o        (fire_o)
    );

    generate
        if (INDEP) begin : g_indep
            wire [3:0] ev_pend2, fire2;

            rosetta_events u_dut2 (
                .clk           (clk),
                .rst_n         (rst_n),
                .run_i         (run_i),
                .in_i          (in_i),
                .in_q_i        (in_q_i),
                .pt_i          (pt_i),
                .cfg_evc_i     (cfg2_evc_i),
                .evc_wr_i      (evc2_wr_i),
                .cfg_stuffev_i (cfg_stuffev_i),
                .stuff_viol_i  (stuff_viol_i),
                .ev_clr_i      (ev_clr_i),
                .ev_pend_o     (ev_pend2),
                .fire_o        (fire2)
            );

            // the environment rule of events.md 4 for the second copy's
            // configuration (u_props assumes reset in the first cycle)
            genvar g;
            for (g = 0; g < 4; g = g + 1) begin : g_env2
                always @(posedge clk) begin
                    if (!$initstate && $past(rst_n && run_i && !evc2_wr_i[g])) begin
                        assume (cfg2_evc_i[16*g +: 16] == $past(cfg2_evc_i[16*g +: 16]));  // @env_cfg2
                    end
                end
            end

            // same[k]: EVCk and evc_wr_i[k] equal in both copies this cycle
            // ag[k]:   ... and in every cycle since the last reset
            wire [3:0] same;
            logic [3:0] ag;
            for (g = 0; g < 4; g = g + 1) begin : g_same
                assign same[g] = (cfg_evc_i[16*g +: 16] == cfg2_evc_i[16*g +: 16]) &&
                                 (evc_wr_i[g] == evc2_wr_i[g]);
            end
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) ag <= 4'hF;
                else        ag <= ag & same;
            end
            for (g = 0; g < 4; g = g + 1) begin : g_ind
                always @* begin
                    if (rst_n && ag[g] && same[g]) begin
                        assert (fire2[g] == fire_o[g]);  // @a_indep_fire
                    end
                    if (ag[g]) begin
                        assert (ev_pend2[g] == ev_pend_o[g]);  // @a_indep_pend
                    end
                end
            end
            // the copies really differ elsewhere while detector 0 agrees
            always @(posedge clk) begin
                c_indep: cover (rst_n && run_i && ag[0] && same[0] && fire_o[0] &&
                                cfg_evc_i[63:16] != cfg2_evc_i[63:16] &&
                                fire_o[3:1] != fire2[3:1]);
            end
        end
    endgenerate
endmodule
