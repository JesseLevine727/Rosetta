`default_nettype none
// ===========================================================================
//  Proof top for rosetta_field.
//
//   u_dut   the block, every input free; field_props (STANDALONE = 1)
//           assumes the environment of field.md 9 / lane.md 2.1 / tick.md
//           and checks layers 1-3 (field_props.sv header).
//   u_bus   (unless FV_NO_BUS) a second copy for the shared-bus lemma
//           (brief item k; lane.md 9.3, sequencer.md 7.3): same inputs,
//           except that park_op_i/park_ctl_i/park_data_i are replaced by
//           unrelated free values in every cycle without park_we_i, and
//           cmd_data_i in every cycle with cmd_i != 2. Every output of the
//           two copies must be equal in every cycle (a_bus_*).
//
//  rst_n is low in the first cycle (assumed in field_props) and free after.
//
//  Partition switches (read -define), used only to find which group of
//  properties is expensive; the `prove` task runs with none of them:
//    FV_NO_BUS    drop u_bus            FV_NO_MODEL  drop layer 1 (a_mdl_*)
//    FV_NO_E2E    drop layers 2-3 (a_e2e_*, a_aux_*, a_p*_*, a_f_*, ...)
//  Stage switches used by field.sby:
//    FV_NO_HARD   (task prove) drop the layer-2 value group and lemmas
//    FV_STAGE_B   (task prove_e2e) layer 1 ASSUMED (it is proved by task
//                 prove), value group + lemmas + layers 2-3 asserted
// ===========================================================================
module field_fv (
    input wire        clk,
    input wire        rst_n,
    input wire        run_i,
    input wire        abort_i,
    input wire        park_we_i,
    input wire [1:0]  park_op_i,
    input wire [27:0] park_ctl_i,
    input wire [31:0] park_data_i,
    input wire        res_ack_i,
    input wire [1:0]  cmd_i,
    input wire [31:0] cmd_data_i,
    input wire [15:0] ts_i,
    input wire [1:0]  cfg_mode_i,
    input wire        lt_i,
    input wire        ct_i,
    input wire        st_i,
    input wire        spend_i,
    input wire        bt_run_i,
    input wire        bt_stop_i,
    input wire        ot_done_i,
    input wire        cfg_honour_i,
    input wire [15:0] d16_i,
    input wire        dedge_i,
    input wire        rxb_i,
    input wire        rxv_i,
    input wire        stuff_next_i,
    input wire        stuff_bit_i,
    input wire [31:0] crc_state_i,
    input wire [31:0] crc_value_i,
    // unrelated bus values for the u_bus copy
    input wire [1:0]  b_park_op_i,
    input wire [27:0] b_park_ctl_i,
    input wire [31:0] b_park_data_i,
    input wire [31:0] b_cmd_data_i
);
    // output bundle of one copy, in a fixed order
    `define FIELD_OUTS(p) \
        wire        p``park_full, p``res_valid, p``m, p``cu_v, p``cu_fld, p``cu_tx, p``cu_smp, \
                    p``pk_v, p``launch_req, p``fld_next, p``fld_first, p``ot_start, p``ot_raw, \
                    p``tx_go, p``tx_bit, p``tx_rel, p``raw_go, p``raw_lvl, p``pin_we, \
                    p``crc_en, p``crc_bit, p``stf_valid, p``stf_bit, p``cu_done, p``hold; \
        wire [3:0]  p``res_dst, p``dst_busy, p``pin_mask, p``pin_val, p``pin_oe; \
        wire [31:0] p``res_data, p``sr; \
        wire [15:0] p``res_ts, p``ot_n;

    `FIELD_OUTS(d_)

    `define FIELD_CONN(p) \
        .park_full_o(p``park_full), .res_valid_o(p``res_valid), .res_dst_o(p``res_dst), \
        .res_data_o(p``res_data), .res_ts_o(p``res_ts), .dst_busy_o(p``dst_busy), .m_o(p``m), \
        .sr_o(p``sr), .cu_v_o(p``cu_v), .cu_fld_o(p``cu_fld), .cu_tx_o(p``cu_tx), \
        .cu_smp_o(p``cu_smp), .pk_v_o(p``pk_v), .launch_req_o(p``launch_req), \
        .fld_next_o(p``fld_next), .fld_first_o(p``fld_first), .ot_start_o(p``ot_start), \
        .ot_raw_o(p``ot_raw), .ot_n_o(p``ot_n), .tx_go_o(p``tx_go), .tx_bit_o(p``tx_bit), \
        .tx_rel_o(p``tx_rel), .raw_go_o(p``raw_go), .raw_lvl_o(p``raw_lvl), .pin_we_o(p``pin_we), \
        .pin_mask_o(p``pin_mask), .pin_val_o(p``pin_val), .pin_oe_o(p``pin_oe), \
        .crc_en_o(p``crc_en), .crc_bit_o(p``crc_bit), .stf_valid_o(p``stf_valid), \
        .stf_bit_o(p``stf_bit), .cu_done_o(p``cu_done), .hold_o(p``hold)

    `define FIELD_INS(OP, CTL, DATA, CMDD) \
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(abort_i), .park_we_i(park_we_i), \
        .park_op_i(OP), .park_ctl_i(CTL), .park_data_i(DATA), .res_ack_i(res_ack_i), \
        .cmd_i(cmd_i), .cmd_data_i(CMDD), .ts_i(ts_i), .cfg_mode_i(cfg_mode_i), \
        .lt_i(lt_i), .ct_i(ct_i), .st_i(st_i), .spend_i(spend_i), .bt_run_i(bt_run_i), \
        .bt_stop_i(bt_stop_i), .ot_done_i(ot_done_i), .cfg_honour_i(cfg_honour_i), \
        .d16_i(d16_i), .dedge_i(dedge_i), .rxb_i(rxb_i), .rxv_i(rxv_i), \
        .stuff_next_i(stuff_next_i), .stuff_bit_i(stuff_bit_i), \
        .crc_state_i(crc_state_i), .crc_value_i(crc_value_i)

    rosetta_field u_dut (
        `FIELD_INS(park_op_i, park_ctl_i, park_data_i, cmd_data_i),
        `FIELD_CONN(d_)
    );

`ifdef FV_NO_MODEL
    field_props #(.STANDALONE(1), .MODEL_CHECK(0)) u_props (
`elsif FV_STAGE_B
    field_props #(.STANDALONE(1), .MODEL_CHECK(2)) u_props (
`else
    field_props #(.STANDALONE(1), .MODEL_CHECK(1)) u_props (
`endif
        `FIELD_INS(park_op_i, park_ctl_i, park_data_i, cmd_data_i),
        `FIELD_CONN(d_)
    );

`ifndef FV_NO_BUS
    `FIELD_OUTS(b_)
    wire [1:0]  bus_op   = park_we_i ? park_op_i   : b_park_op_i;
    wire [27:0] bus_ctl  = park_we_i ? park_ctl_i  : b_park_ctl_i;
    wire [31:0] bus_data = park_we_i ? park_data_i : b_park_data_i;
    wire [31:0] bus_cmdd = (cmd_i == 2'd2) ? cmd_data_i : b_cmd_data_i;

    rosetta_field u_bus (
        `FIELD_INS(bus_op, bus_ctl, bus_data, bus_cmdd),
        `FIELD_CONN(b_)
    );

    wire [254:0] d_all = {d_park_full, d_res_valid, d_m, d_cu_v, d_cu_fld, d_cu_tx, d_cu_smp,
                          d_pk_v, d_launch_req, d_fld_next, d_fld_first, d_ot_start, d_ot_raw,
                          d_tx_go, d_tx_bit, d_tx_rel, d_raw_go, d_raw_lvl, d_pin_we,
                          d_crc_en, d_crc_bit, d_stf_valid, d_stf_bit, d_cu_done, d_hold,
                          d_res_dst, d_dst_busy, d_pin_mask, d_pin_val, d_pin_oe,
                          d_res_data, d_sr, d_res_ts, d_ot_n, 50'h0};
    wire [254:0] b_all = {b_park_full, b_res_valid, b_m, b_cu_v, b_cu_fld, b_cu_tx, b_cu_smp,
                          b_pk_v, b_launch_req, b_fld_next, b_fld_first, b_ot_start, b_ot_raw,
                          b_tx_go, b_tx_bit, b_tx_rel, b_raw_go, b_raw_lvl, b_pin_we,
                          b_crc_en, b_crc_bit, b_stf_valid, b_stf_bit, b_cu_done, b_hold,
                          b_res_dst, b_dst_busy, b_pin_mask, b_pin_val, b_pin_oe,
                          b_res_data, b_sr, b_res_ts, b_ot_n, 50'h0};
    always @* begin
        a_bus_lemma: assert (d_all == b_all);
    end
`endif
endmodule
