`default_nettype none
// ===========================================================================
//  lane_model: the lane top as lane.md WRITES it (sections 3, 5, 6, 7.6), a
//  transcription of the spec's wiring of the nine proven sub-blocks plus the
//  two glue flops (in_q, crcw_q). Written from lane.md / decisions.md and the
//  sub-block port lists only (the lane RTL was not read).
//
//  Used by the checker as the golden composition (layer G of lane_props.sv:
//  every lane output of the design equals this model's output in every
//  cycle, bmc task). The same expressions are asserted net by net on the
//  design's sub-block instances by layer W (unbounded).
//
//  Amendment H3/H4 (amendment-eth-can.md, ruling 59; H1 on hold): TICK is
//  stored as [15:0] with bit 13 constant 0, CODE as [12:0]; u_tick gets
//  cfg_rone_i = TICK[14], cfg_rsp2_i = TICK[15]; u_field gets cfg_rawedge_i
//  = CODE[12]. The RECOVER strobes (redge, H', the !(inc >= Pr) term) and the
//  RAW DRIVE completion are inside the real sub-blocks this model
//  instantiates; the model itself has no copy of those rules. Amendment 2.5
//  (ruling 60): u_field.ot_cancel_o drives u_tick.ot_cancel_i.
//
//  Reading choices (each switchable, see lane_props.sv header, "Spec
//  readings"):
//    LANE_CRCW_UNGATED  crcw_q term of u_crc.init_i not gated by run_i
//                       (default: gated, lane.md 5.4 "All are gated by
//                       run_i except the last term").
// ===========================================================================
module lane_model #(
    parameter int RTRIP = 4
) (
    input  wire         clk, rst_n,
    input  wire         run_i,
    input  wire         park_we_i,
    input  wire  [1:0]  park_op_i,
    input  wire  [27:0] park_ctl_i,
    input  wire  [31:0] park_data_i,
    input  wire         res_ack_i,
    input  wire         abort_i,
    input  wire  [3:0]  ev_clr_i,
    input  wire         cfg_we_i,
    input  wire  [4:0]  cfg_reg_i,
    input  wire         cfg_hi_i,
    input  wire  [15:0] cfg_data_i,
    input  wire  [1:0]  cmd_i,
    input  wire  [31:0] cmd_data_i,
    input  wire         data_wait_i,
    input  wire  [3:0]  in_i,
    input  wire  [15:0] ts_i,
    output logic        busy_o,
    output logic        park_full_o,
    output logic        res_valid_o,
    output logic [3:0]  res_dst_o,
    output logic [31:0] res_data_o,
    output logic [15:0] res_ts_o,
    output logic [3:0]  dst_busy_o,
    output logic        m_o,
    output logic [3:0]  ev_pend_o,
    output logic [31:0] sr_o,
    output logic [31:0] crc_o,
    output logic [3:0]  out_val_o,
    output logic [3:0]  out_oe_o
);
    // ---------------------------------------------------------------- u_cfg
    wire [15:0] r_tick;  /* TICK[15:0], bit 13 reads 0 */  wire [15:0] r_period, r_period2, r_pulse;
    wire [7:0]  r_thr0, r_dly;  wire [12:0] r_code;  /* CODE[12:0] */  wire [5:0] r_stuff;
    wire [31:0] r_crcpoly;  wire [6:0] r_crccfg;  wire [63:0] r_evc;
    wire        wr_tick, wr_code, wr_stuff, wr_crc;  wire [3:0] wr_evc;
    rosetta_cfg u_cfg (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .cfg_we_i(cfg_we_i),
        .cfg_reg_i(cfg_reg_i), .cfg_hi_i(cfg_hi_i), .cfg_data_i(cfg_data_i),
        .tick_o(r_tick), .period_o(r_period), .period2_o(r_period2),
        .thr0_o(r_thr0), .dly_o(r_dly), .code_o(r_code), .pulse_o(r_pulse),
        .stuff_o(r_stuff), .crcpoly_o(r_crcpoly), .crccfg_o(r_crccfg),
        .evc_o(r_evc), .wr_tick_o(wr_tick), .wr_code_o(wr_code),
        .wr_stuff_o(wr_stuff), .wr_crc_o(wr_crc), .wr_evc_o(wr_evc));

    // ---------------------------------------------------------------- glue
    logic [3:0] in_q;          // lane.md 3: previous in_i, every cycle
    logic       crcw_q;        // lane.md 5.3: second CRC init
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin in_q <= 4'd0; crcw_q <= 1'b0; end
        else        begin in_q <= in_i; crcw_q <= wr_crc; end

    // ------------------------------------------------------- interface nets
    wire        pt, lt, ct, st, cte, hte, spend, bt_run, bt_stop, ot_done, ge0;
    wire [15:0] d16;
    wire        creq, creq_dly, creq_cap, stretch;  wire [1:0] cpay;
    wire        d_we, c_we, dcap;  wire [4:0] d_pay;  wire [1:0] c_pay;
    wire        dline, dedge, rxb, rxv, pw_bit, dreq, dreq_dly;  wire [4:0] dpay;
    wire        cu_v, cu_fld, cu_tx, cu_smp, pk_v, launch_req, fld_next, fld_first;
    wire        ot_start, ot_raw;  wire [15:0] ot_n;
    wire        tx_go, tx_bit, tx_rel, raw_go, raw_lvl, pin_we;
    wire [3:0]  pin_mask, pin_val, pin_oe;
    wire        crc_en, crc_bit, stf_valid, stf_bit, cu_done, hold;
    wire        ot_cancel;                   // amendment 2.5 (ruling 60)
    wire [31:0] crc_state, crc_value;
    wire        stuff_next, stuff_bit, stuff_viol;
    wire [3:0]  fire;

    // ---------------------------------------------------------------- u_tick
    rosetta_tick #(.RTRIP(RTRIP)) u_tick (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(abort_i), .wr_tick_i(wr_tick),
        .cfg_mode_i(r_tick[1:0]), .cfg_ps_i(r_tick[3:2]), .cfg_clkin_i(r_tick[5:4]),
        .cfg_capfall_i(r_tick[6]), .cfg_clkout_i(r_tick[7]), .cfg_idle_i(r_tick[8]),
        .cfg_clkod_i(r_tick[9]), .cfg_rest_i(r_tick[10]), .cfg_honour_i(r_tick[11]),
        .cfg_hold_i(r_tick[12]), .cfg_rone_i(r_tick[14]), .cfg_rsp2_i(r_tick[15]),
        .cfg_period_i(r_period), .cfg_period2_i(r_period2), .cfg_thr0_i(r_thr0),
        .cfg_pw0_i(r_pulse[7:0]), .cfg_pw1_i(r_pulse[15:8]),
        .cfg_dlyclk_i(r_code[10]), .cfg_dly_i(r_dly),
        .in_i(in_i), .in_q_i(in_q), .dline_i(dline), .dedge_i(dedge), .pw_bit_i(pw_bit),
        .cu_v_i(cu_v), .cu_fld_i(cu_fld), .cu_tx_i(cu_tx), .cu_smp_i(cu_smp), .pk_v_i(pk_v),
        .launch_req_i(launch_req), .fld_next_i(fld_next), .fld_first_i(fld_first),
        .data_wait_i(data_wait_i), .ot_start_i(ot_start), .ot_raw_i(ot_raw), .ot_n_i(ot_n),
        .dcap_i(dcap), .ot_cancel_i(ot_cancel),
        .pt_o(pt), .lt_o(lt), .ct_o(ct), .st_o(st), .cte_o(cte), .hte_o(hte),
        .spend_o(spend), .bt_run_o(bt_run), .bt_stop_o(bt_stop), .ot_done_o(ot_done),
        .d16_o(d16), .ge0_o(ge0), .creq_o(creq), .cpay_o(cpay), .creq_dly_o(creq_dly),
        .creq_cap_o(creq_cap), .stretch_o(stretch));

    // ---------------------------------------------------------------- u_dly
    rosetta_delay u_dly (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(abort_i), .wr_tick_i(wr_tick),
        .cfg_dly_i(r_dly), .cfg_dlyclk_i(r_code[10]), .cfg_dlyps_i(r_code[11]),
        .pt_i(pt), .dreq_i(dreq), .dpay_i(dpay), .dreq_dly_i(dreq_dly),
        .creq_i(creq), .cpay_i(cpay), .creq_dly_i(creq_dly), .creq_cap_i(creq_cap),
        .pin_we_i(pin_we), .pin_mask_i(pin_mask),
        .d_we_o(d_we), .d_pay_o(d_pay), .c_we_o(c_we), .c_pay_o(c_pay), .dcap_o(dcap));

    // ---------------------------------------------------------------- u_code
    rosetta_coding u_code (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(abort_i),
        .wr_code_i(wr_code), .wr_tick_i(wr_tick), .cfg_mode_i(r_tick[1:0]),
        .cfg_rx_i(r_code[1:0]), .cfg_tx_i(r_code[3:2]), .cfg_din_i(r_code[5:4]),
        .cfg_inv_i(r_code[6]), .cfg_od_i(r_code[7]), .cfg_dif_i(r_code[8]),
        .cfg_dfi_i(r_code[9]), .cfg_pw0_i(r_pulse[7:0]), .cfg_pw1_i(r_pulse[15:8]),
        .in_i(in_i), .st_i(st), .cte_i(cte), .hte_i(hte), .ge0_i(ge0),
        .tx_go_i(tx_go), .tx_bit_i(tx_bit), .tx_rel_i(tx_rel), .raw_go_i(raw_go),
        .raw_lvl_i(raw_lvl), .pin_we_i(pin_we), .pin_mask0_i(pin_mask[0]),
        .pin_val0_i(pin_val[0]), .pin_oe0_i(pin_oe[0]), .out_val_i(out_val_o[1:0]),
        .dline_o(dline), .dedge_o(dedge), .rxb_o(rxb), .rxv_o(rxv), .pw_bit_o(pw_bit),
        .dreq_o(dreq), .dpay_o(dpay), .dreq_dly_o(dreq_dly));

    // ---------------------------------------------------------------- u_field
    rosetta_field u_field (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(abort_i),
        .park_we_i(park_we_i), .park_op_i(park_op_i), .park_ctl_i(park_ctl_i),
        .park_data_i(park_data_i), .res_ack_i(res_ack_i), .cmd_i(cmd_i),
        .cmd_data_i(cmd_data_i), .ts_i(ts_i), .cfg_mode_i(r_tick[1:0]),
        .lt_i(lt), .ct_i(ct), .st_i(st), .spend_i(spend), .bt_run_i(bt_run),
        .bt_stop_i(bt_stop), .ot_done_i(ot_done), .cfg_honour_i(r_tick[11]),
        .cfg_rawedge_i(r_code[12]), .d16_i(d16), .dedge_i(dedge), .rxb_i(rxb), .rxv_i(rxv),
        .stuff_next_i(stuff_next), .stuff_bit_i(stuff_bit),
        .crc_state_i(crc_state), .crc_value_i(crc_value),
        .park_full_o(park_full_o), .res_valid_o(res_valid_o), .res_dst_o(res_dst_o),
        .res_data_o(res_data_o), .res_ts_o(res_ts_o), .dst_busy_o(dst_busy_o),
        .m_o(m_o), .sr_o(sr_o),
        .cu_v_o(cu_v), .cu_fld_o(cu_fld), .cu_tx_o(cu_tx), .cu_smp_o(cu_smp), .pk_v_o(pk_v),
        .launch_req_o(launch_req), .fld_next_o(fld_next), .fld_first_o(fld_first),
        .ot_start_o(ot_start), .ot_raw_o(ot_raw), .ot_n_o(ot_n),
        .tx_go_o(tx_go), .tx_bit_o(tx_bit), .tx_rel_o(tx_rel), .raw_go_o(raw_go),
        .raw_lvl_o(raw_lvl), .pin_we_o(pin_we), .pin_mask_o(pin_mask),
        .pin_val_o(pin_val), .pin_oe_o(pin_oe), .crc_en_o(crc_en), .crc_bit_o(crc_bit),
        .stf_valid_o(stf_valid), .stf_bit_o(stf_bit), .cu_done_o(cu_done), .hold_o(hold),
        .ot_cancel_o(ot_cancel));

    // ---------------------------------------------------------------- u_crc
`ifdef LANE_CRCW_UNGATED
    wire crc_init = (run_i && cmd_i == 2'd1) || wr_crc || crcw_q;
`else
    wire crc_init = run_i && (cmd_i == 2'd1 || wr_crc || crcw_q);
`endif
    rosetta_crc u_crc (
        .clk(clk), .rst_n(rst_n), .cfg_poly_i(r_crcpoly), .cfg_init_i({32{r_crccfg[5]}}),
        .cfg_width_m1_i(r_crccfg[4:0]), .cfg_xorout_i(r_crccfg[6]),
        .init_i(crc_init), .en_i(crc_en), .bit_i(crc_bit),
        .state_o(crc_state), .value_o(crc_value));

    // ---------------------------------------------------------------- u_stuff
    rosetta_stuff u_stuff (
        .clk(clk), .rst_n(rst_n), .cfg_en_i(r_stuff[4]), .cfg_n_i(r_stuff[2:0]),
        .cfg_mode_i(r_stuff[3]),
        .clear_i((run_i && cmd_i == 2'd3) || wr_stuff || !run_i),
        .bit_valid_i(stf_valid), .bit_i(stf_bit),
        .stuff_next_o(stuff_next), .stuff_bit_o(stuff_bit), .violation_o(stuff_viol));

    // ---------------------------------------------------------------- u_evt
    rosetta_events u_evt (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .in_i(in_i), .in_q_i(in_q), .pt_i(pt),
        .cfg_evc_i(r_evc), .evc_wr_i(wr_evc), .cfg_stuffev_i(r_stuff[5]),
        .stuff_viol_i(stuff_viol), .ev_clr_i(ev_clr_i),
        .ev_pend_o(ev_pend_o), .fire_o(fire));

    // ---------------------------------------------------------------- u_pins
    rosetta_pins u_pins (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .d_we_i(d_we), .d_pay_i(d_pay),
        .c_we_i(c_we), .c_pay_i(c_pay), .pin_we_i(pin_we), .pin_mask_i(pin_mask),
        .pin_val_i(pin_val), .pin_oe_i(pin_oe),
        .out_val_o(out_val_o), .out_oe_o(out_oe_o));

    // ---------------------------------------------------------------- outputs
    assign busy_o = cu_v || bt_run;        // lane.md 7.3 / 7.6
    assign crc_o  = crc_value;             // lane.md 2.3
endmodule
