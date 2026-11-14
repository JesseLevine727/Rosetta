`default_nettype none
// ===========================================================================
//  Proof top for rosetta_tick.
//
//   u_dut / u_props  every input free (environment rules of tick_props
//                    assumed); rst_n low in cycle 0, free after. Flops start
//                    at arbitrary values. Exact model + required properties.
//   g_w[1..2]        tick.md 11.1 and 11.2: scripted instances with their
//                    own reset; the tables asserted cycle by cycle.
//                    Removed with `read -define FV_NO_WORKED`.
//   u_fi / u_fip     edge interval counter from a free initial state (its
//                    own free reset, no reset in cycle 0): reaches the
//                    16-bit saturation. Remove with `read -define FV_NO_FI`
//                    if the design keeps edge-interval state that d16_o
//                    does not show.
//
//  Proof range (tick_props parameters): `define TICK_PBITS / TICK_OBITS
//  (default 16 = full range); the sby file sets them per task.
//
//  Spec: docs/blocks/tick.md (sections 1-8, 10, 11), lane.md 1-6, 12.2,
//  decisions.md.
// ===========================================================================
`ifndef TICK_PBITS
 `define TICK_PBITS 16
`endif
`ifndef TICK_OBITS
 `define TICK_OBITS 16
`endif

module tick_fv (
    input wire        clk,
    input wire        rst_n,
    input wire        run_i,
    input wire        abort_i,
    input wire        wr_tick_i,
    input wire [1:0]  cfg_mode_i,
    input wire [1:0]  cfg_ps_i,
    input wire [1:0]  cfg_clkin_i,
    input wire        cfg_capfall_i,
    input wire        cfg_clkout_i,
    input wire        cfg_idle_i,
    input wire        cfg_clkod_i,
    input wire        cfg_rest_i,
    input wire        cfg_honour_i,
    input wire        cfg_hold_i,
    input wire [15:0] cfg_period_i,
    input wire [15:0] cfg_period2_i,
    input wire [7:0]  cfg_thr0_i,
    input wire [7:0]  cfg_pw0_i,
    input wire [7:0]  cfg_pw1_i,
    input wire        cfg_dlyclk_i,
    input wire [7:0]  cfg_dly_i,
    input wire [3:0]  in_i,
    input wire [3:0]  in_q_i,
    input wire        dline_i,
    input wire        dedge_i,
    input wire        pw_bit_i,
    input wire        cu_v_i,
    input wire        cu_fld_i,
    input wire        cu_tx_i,
    input wire        cu_smp_i,
    input wire        pk_v_i,
    input wire        launch_req_i,
    input wire        fld_next_i,
    input wire        fld_first_i,
    input wire        data_wait_i,
    input wire        ot_start_i,
    input wire        ot_raw_i,
    input wire [15:0] ot_n_i,
    input wire        dcap_i,
    // free-init instance
    input wire        fi_rst_n,
    input wire        fi_run_i,
    input wire        fi_dedge_i,
    input wire [1:0]  fi_ps_i,
    input wire [7:0]  fi_thr0_i
);
    wire        pt_o, lt_o, ct_o, st_o, cte_o, hte_o, spend_o, bt_run_o, bt_stop_o;
    wire        ot_done_o, ge0_o, creq_o, creq_dly_o, creq_cap_o, stretch_o;
    wire [15:0] d16_o;
    wire [1:0]  cpay_o;

`define TICK_PORTS(P) \
        .clk(clk), .run_i(P``run_i), .abort_i(P``abort_i), .wr_tick_i(P``wr_tick_i), \
        .cfg_mode_i(P``cfg_mode_i), .cfg_ps_i(P``cfg_ps_i), .cfg_clkin_i(P``cfg_clkin_i), \
        .cfg_capfall_i(P``cfg_capfall_i), .cfg_clkout_i(P``cfg_clkout_i), .cfg_idle_i(P``cfg_idle_i), \
        .cfg_clkod_i(P``cfg_clkod_i), .cfg_rest_i(P``cfg_rest_i), .cfg_honour_i(P``cfg_honour_i), \
        .cfg_hold_i(P``cfg_hold_i), .cfg_period_i(P``cfg_period_i), .cfg_period2_i(P``cfg_period2_i), \
        .cfg_thr0_i(P``cfg_thr0_i), .cfg_pw0_i(P``cfg_pw0_i), .cfg_pw1_i(P``cfg_pw1_i), \
        .cfg_dlyclk_i(P``cfg_dlyclk_i), .cfg_dly_i(P``cfg_dly_i), .in_i(P``in_i), .in_q_i(P``in_q_i), \
        .dline_i(P``dline_i), .dedge_i(P``dedge_i), .pw_bit_i(P``pw_bit_i), .cu_v_i(P``cu_v_i), \
        .cu_fld_i(P``cu_fld_i), .cu_tx_i(P``cu_tx_i), .cu_smp_i(P``cu_smp_i), .pk_v_i(P``pk_v_i), \
        .launch_req_i(P``launch_req_i), .fld_next_i(P``fld_next_i), .fld_first_i(P``fld_first_i), \
        .data_wait_i(P``data_wait_i), .ot_start_i(P``ot_start_i), .ot_raw_i(P``ot_raw_i), \
        .ot_n_i(P``ot_n_i), .dcap_i(P``dcap_i), \
        .pt_o(P``pt_o), .lt_o(P``lt_o), .ct_o(P``ct_o), .st_o(P``st_o), .cte_o(P``cte_o), \
        .hte_o(P``hte_o), .spend_o(P``spend_o), .bt_run_o(P``bt_run_o), .bt_stop_o(P``bt_stop_o), \
        .ot_done_o(P``ot_done_o), .d16_o(P``d16_o), .ge0_o(P``ge0_o), .creq_o(P``creq_o), \
        .cpay_o(P``cpay_o), .creq_dly_o(P``creq_dly_o), .creq_cap_o(P``creq_cap_o), \
        .stretch_o(P``stretch_o)

    // ------------------------------------------------------------ main
    rosetta_tick #(.RTRIP(4)) u_dut (.rst_n(rst_n), `TICK_PORTS());
    tick_props #(.STANDALONE(1), .PBITS(`TICK_PBITS), .OBITS(`TICK_OBITS))
        u_props (.rst_n(rst_n), `TICK_PORTS());

    // ------------------------------------------------------------ 11.1, 11.2
`ifndef FV_NO_WORKED
    genvar s;
    generate
        for (s = 1; s <= 2; s = s + 1) begin : g_w
            wire        w_rst_n, w_run_i, w_abort_i, w_wr_tick_i, w_cfg_capfall_i, w_cfg_clkout_i;
            wire        w_cfg_idle_i, w_cfg_clkod_i, w_cfg_rest_i, w_cfg_honour_i, w_cfg_hold_i;
            wire        w_cfg_dlyclk_i, w_dline_i, w_dedge_i, w_pw_bit_i, w_cu_v_i, w_cu_fld_i;
            wire        w_cu_tx_i, w_cu_smp_i, w_pk_v_i, w_launch_req_i, w_fld_next_i, w_fld_first_i;
            wire        w_data_wait_i, w_ot_start_i, w_ot_raw_i, w_dcap_i;
            wire [1:0]  w_cfg_mode_i, w_cfg_ps_i, w_cfg_clkin_i;
            wire [15:0] w_cfg_period_i, w_cfg_period2_i, w_ot_n_i;
            wire [7:0]  w_cfg_thr0_i, w_cfg_pw0_i, w_cfg_pw1_i, w_cfg_dly_i;
            wire [3:0]  w_in_i, w_in_q_i;
            wire        w_pt_o, w_lt_o, w_ct_o, w_st_o, w_cte_o, w_hte_o, w_spend_o, w_bt_run_o;
            wire        w_bt_stop_o, w_ot_done_o, w_ge0_o, w_creq_o, w_creq_dly_o, w_creq_cap_o;
            wire        w_stretch_o;
            wire [15:0] w_d16_o;
            wire [1:0]  w_cpay_o;
            rosetta_tick #(.RTRIP(4)) u_wdut (.rst_n(w_rst_n), `TICK_PORTS(w_));
            tick_worked #(.SCEN(s)) u_wchk (.w_rst_n(w_rst_n), `TICK_PORTS(w_));
        end
    endgenerate
`endif

    // ------------------------------------------------------------ free init
`ifndef FV_NO_FI
    wire        fi_pt_o, fi_ge0_o;
    wire [15:0] fi_d16_o;
    rosetta_tick #(.RTRIP(4)) u_fi (
        .clk(clk), .rst_n(fi_rst_n), .run_i(fi_run_i), .abort_i(abort_i), .wr_tick_i(wr_tick_i),
        .cfg_mode_i(cfg_mode_i), .cfg_ps_i(fi_ps_i), .cfg_clkin_i(cfg_clkin_i),
        .cfg_capfall_i(cfg_capfall_i), .cfg_clkout_i(cfg_clkout_i), .cfg_idle_i(cfg_idle_i),
        .cfg_clkod_i(cfg_clkod_i), .cfg_rest_i(cfg_rest_i), .cfg_honour_i(cfg_honour_i),
        .cfg_hold_i(cfg_hold_i), .cfg_period_i(cfg_period_i), .cfg_period2_i(cfg_period2_i),
        .cfg_thr0_i(fi_thr0_i), .cfg_pw0_i(cfg_pw0_i), .cfg_pw1_i(cfg_pw1_i),
        .cfg_dlyclk_i(cfg_dlyclk_i), .cfg_dly_i(cfg_dly_i), .in_i(in_i), .in_q_i(in_q_i),
        .dline_i(dline_i), .dedge_i(fi_dedge_i), .pw_bit_i(pw_bit_i), .cu_v_i(cu_v_i),
        .cu_fld_i(cu_fld_i), .cu_tx_i(cu_tx_i), .cu_smp_i(cu_smp_i), .pk_v_i(pk_v_i),
        .launch_req_i(launch_req_i), .fld_next_i(fld_next_i), .fld_first_i(fld_first_i),
        .data_wait_i(data_wait_i), .ot_start_i(ot_start_i), .ot_raw_i(ot_raw_i),
        .ot_n_i(ot_n_i), .dcap_i(dcap_i),
        .pt_o(fi_pt_o), .lt_o(), .ct_o(), .st_o(), .cte_o(), .hte_o(), .spend_o(), .bt_run_o(),
        .bt_stop_o(), .ot_done_o(), .d16_o(fi_d16_o), .ge0_o(fi_ge0_o), .creq_o(), .cpay_o(),
        .creq_dly_o(), .creq_cap_o(), .stretch_o()
    );
    tick_ecnt_props u_fip (
        .clk(clk), .rst_n(fi_rst_n), .run_i(fi_run_i), .dedge_i(fi_dedge_i),
        .cfg_thr0_i(fi_thr0_i), .pt_o(fi_pt_o), .d16_o(fi_d16_o), .ge0_o(fi_ge0_o)
    );
`endif
endmodule
