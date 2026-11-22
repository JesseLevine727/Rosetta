`default_nettype none
// ===========================================================================
//  ZERO-RULE CHECK (property side), direction 1. Not the design.
//  Presents the property writer's NEW reference model (amendment H4,
//  renamed rosetta_tick_h34 by ref/zr/run_zero_rule.sh) with the PRE-
//  amendment port list and the new inputs (cfg_rone_i, cfg_rsp2_i,
//  ot_cancel_i) tied to 0, so that the PREVIOUS
//  property files (git HEAD formal/tick/tick_props.sv, tick_fv.sv) run
//  against it unchanged.
// ===========================================================================
module rosetta_tick #(
    parameter RTRIP = 4
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run_i,
    input  wire        abort_i,
    input  wire        wr_tick_i,
    input  wire [1:0]  cfg_mode_i,
    input  wire [1:0]  cfg_ps_i,
    input  wire [1:0]  cfg_clkin_i,
    input  wire        cfg_capfall_i,
    input  wire        cfg_clkout_i,
    input  wire        cfg_idle_i,
    input  wire        cfg_clkod_i,
    input  wire        cfg_rest_i,
    input  wire        cfg_honour_i,
    input  wire        cfg_hold_i,
    input  wire [15:0] cfg_period_i,
    input  wire [15:0] cfg_period2_i,
    input  wire [7:0]  cfg_thr0_i,
    input  wire [7:0]  cfg_pw0_i,
    input  wire [7:0]  cfg_pw1_i,
    input  wire        cfg_dlyclk_i,
    input  wire [7:0]  cfg_dly_i,
    input  wire [3:0]  in_i,
    input  wire [3:0]  in_q_i,
    input  wire        dline_i,
    input  wire        dedge_i,
    input  wire        pw_bit_i,
    input  wire        cu_v_i,
    input  wire        cu_fld_i,
    input  wire        cu_tx_i,
    input  wire        cu_smp_i,
    input  wire        pk_v_i,
    input  wire        launch_req_i,
    input  wire        fld_next_i,
    input  wire        fld_first_i,
    input  wire        data_wait_i,
    input  wire        ot_start_i,
    input  wire        ot_raw_i,
    input  wire [15:0] ot_n_i,
    input  wire        dcap_i,
    output wire        pt_o,
    output wire        lt_o,
    output wire        ct_o,
    output wire        st_o,
    output wire        cte_o,
    output wire        hte_o,
    output wire        spend_o,
    output wire        bt_run_o,
    output wire        bt_stop_o,
    output wire        ot_done_o,
    output wire [15:0] d16_o,
    output wire        ge0_o,
    output wire        creq_o,
    output wire [1:0]  cpay_o,
    output wire        creq_dly_o,
    output wire        creq_cap_o,
    output wire        stretch_o
);
    rosetta_tick_h34 #(.RTRIP(RTRIP)) u (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(abort_i), .wr_tick_i(wr_tick_i),
        .cfg_mode_i(cfg_mode_i), .cfg_ps_i(cfg_ps_i), .cfg_clkin_i(cfg_clkin_i),
        .cfg_capfall_i(cfg_capfall_i), .cfg_clkout_i(cfg_clkout_i), .cfg_idle_i(cfg_idle_i),
        .cfg_clkod_i(cfg_clkod_i), .cfg_rest_i(cfg_rest_i), .cfg_honour_i(cfg_honour_i),
        .cfg_hold_i(cfg_hold_i),
        .cfg_rone_i(1'b0), .cfg_rsp2_i(1'b0),                 // the new bits tied to 0
        .ot_cancel_i(1'b0),                                   // ruling 60: tied to 0
        .cfg_period_i(cfg_period_i), .cfg_period2_i(cfg_period2_i),
        .cfg_thr0_i(cfg_thr0_i), .cfg_pw0_i(cfg_pw0_i), .cfg_pw1_i(cfg_pw1_i),
        .cfg_dlyclk_i(cfg_dlyclk_i), .cfg_dly_i(cfg_dly_i), .in_i(in_i), .in_q_i(in_q_i),
        .dline_i(dline_i), .dedge_i(dedge_i), .pw_bit_i(pw_bit_i), .cu_v_i(cu_v_i),
        .cu_fld_i(cu_fld_i), .cu_tx_i(cu_tx_i), .cu_smp_i(cu_smp_i), .pk_v_i(pk_v_i),
        .launch_req_i(launch_req_i), .fld_next_i(fld_next_i), .fld_first_i(fld_first_i),
        .data_wait_i(data_wait_i), .ot_start_i(ot_start_i), .ot_raw_i(ot_raw_i),
        .ot_n_i(ot_n_i), .dcap_i(dcap_i),
        .pt_o(pt_o), .lt_o(lt_o), .ct_o(ct_o), .st_o(st_o), .cte_o(cte_o), .hte_o(hte_o),
        .spend_o(spend_o), .bt_run_o(bt_run_o), .bt_stop_o(bt_stop_o), .ot_done_o(ot_done_o),
        .d16_o(d16_o), .ge0_o(ge0_o), .creq_o(creq_o), .cpay_o(cpay_o),
        .creq_dly_o(creq_dly_o), .creq_cap_o(creq_cap_o), .stretch_o(stretch_o)
    );
endmodule
