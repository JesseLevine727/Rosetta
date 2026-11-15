`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL of rosetta_lane -- NOT the
//  design. Written from lane.md (3, 5, 6, 7.6), decisions.md and the proven
//  sub-block port lists only. It exists to show the lane properties are not
//  vacuous: the clean model must pass every task, and every `MUT_*` below (a
//  plausible integration bug, one per define) must fail at least one of
//  prove / bmc. Run with ref/run_mutants.sh.
//
//  Instance names are those of lane.md section 3 (u_cfg, u_tick, u_dly,
//  u_code, u_field, u_crc, u_stuff, u_evt, u_pins); layer W of the checker
//  observes their ports by these names.
// ===========================================================================
module rosetta_lane #(
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
    wire [12:0] r_tick;  wire [15:0] r_period, r_period2, r_pulse;
    wire [7:0]  r_thr0, r_dly;  wire [11:0] r_code;  wire [5:0] r_stuff;
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
    wire [31:0] crc_state, crc_value;
    wire        stuff_next, stuff_bit, stuff_viol;
    wire [3:0]  fire;
    wire        f_res_valid;  wire [3:0] f_dst_busy;
    wire [3:0]  p_out_val, p_out_oe;

    // ---------------------------------------------------------------- glue
    logic [3:0]  in_q;
    logic        crcw_q;
    logic [15:0] ts_q;                       // MUT_TS_LATE only
    logic        pwe_q;  logic [1:0] pop_q;  logic [27:0] pctl_q;  logic [31:0] pdat_q;
    logic        rv_q;                       // MUT_RESV_LATE only
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            in_q <= 4'd0; crcw_q <= 1'b0; ts_q <= 16'd0;
            pwe_q <= 1'b0; pop_q <= 2'd0; pctl_q <= 28'd0; pdat_q <= 32'd0; rv_q <= 1'b0;
        end else begin
`ifdef MUT_INQ_RUNONLY
            if (run_i) in_q <= in_i;         // BUG: in_q frozen while halted
`else
            in_q <= in_i;
`endif
            crcw_q <= wr_crc;
            ts_q   <= ts_i;
            pwe_q  <= park_we_i && run_i; pop_q <= park_op_i; pctl_q <= park_ctl_i;
            pdat_q <= park_data_i;
            rv_q   <= f_res_valid;
        end

    // ---------------------------------------------------------------- u_tick
`ifdef MUT_RTRIP3
    localparam int TR = 3;                   // BUG: wrong round trip
`else
    localparam int TR = RTRIP;
`endif
`ifdef MUT_CLKIN_BITS
    wire [1:0] t_clkin = r_tick[6:5];        // BUG: field read from the wrong bits
`else
    wire [1:0] t_clkin = r_tick[5:4];
`endif
`ifdef MUT_PW_SWAP_TICK
    wire [7:0] t_pw0 = r_pulse[15:8], t_pw1 = r_pulse[7:0];
`else
    wire [7:0] t_pw0 = r_pulse[7:0],  t_pw1 = r_pulse[15:8];
`endif
`ifdef MUT_TICK_INQ_RAW
    wire [3:0] t_inq = in_i;                 // BUG: no previous-cycle copy
`else
    wire [3:0] t_inq = in_q;
`endif
`ifdef MUT_DCAP_OPEN
    wire t_dcap = 1'b0;
`else
    wire t_dcap = dcap;
`endif
`ifdef MUT_DATAWAIT_OPEN
    wire t_dwait = 1'b0;
`else
    wire t_dwait = data_wait_i;
`endif
`ifdef MUT_DLYCLK_TICK
    wire t_dlyclk = r_code[11];
`else
    wire t_dlyclk = r_code[10];
`endif
`ifdef MUT_HALT_TICK
    wire t_run = 1'b1;                       // BUG: halt does not stop the tick unit
`else
    wire t_run = run_i;
`endif
`ifdef MUT_CU_TXSMP_SWAP
    wire t_cutx = cu_smp, t_cusmp = cu_tx;
`else
    wire t_cutx = cu_tx,  t_cusmp = cu_smp;
`endif
`ifdef MUT_FLDNEXT_OPEN
    wire t_fldnext = 1'b0;
`else
    wire t_fldnext = fld_next;
`endif
`ifdef MUT_PKV_TICK_OPEN
    wire t_pkv = 1'b0;
`else
    wire t_pkv = pk_v;
`endif
    rosetta_tick #(.RTRIP(TR)) u_tick (
        .clk(clk), .rst_n(rst_n), .run_i(t_run), .abort_i(abort_i), .wr_tick_i(wr_tick),
        .cfg_mode_i(r_tick[1:0]), .cfg_ps_i(r_tick[3:2]), .cfg_clkin_i(t_clkin),
        .cfg_capfall_i(r_tick[6]), .cfg_clkout_i(r_tick[7]), .cfg_idle_i(r_tick[8]),
        .cfg_clkod_i(r_tick[9]), .cfg_rest_i(r_tick[10]), .cfg_honour_i(r_tick[11]),
        .cfg_hold_i(r_tick[12]),
        .cfg_period_i(r_period), .cfg_period2_i(r_period2), .cfg_thr0_i(r_thr0),
        .cfg_pw0_i(t_pw0), .cfg_pw1_i(t_pw1),
        .cfg_dlyclk_i(t_dlyclk), .cfg_dly_i(r_dly),
        .in_i(in_i), .in_q_i(t_inq), .dline_i(dline), .dedge_i(dedge), .pw_bit_i(pw_bit),
        .cu_v_i(cu_v), .cu_fld_i(cu_fld), .cu_tx_i(t_cutx), .cu_smp_i(t_cusmp), .pk_v_i(t_pkv),
        .launch_req_i(launch_req), .fld_next_i(t_fldnext), .fld_first_i(fld_first),
        .data_wait_i(t_dwait), .ot_start_i(ot_start), .ot_raw_i(ot_raw), .ot_n_i(ot_n),
        .dcap_i(t_dcap),
        .pt_o(pt), .lt_o(lt), .ct_o(ct), .st_o(st), .cte_o(cte), .hte_o(hte),
        .spend_o(spend), .bt_run_o(bt_run), .bt_stop_o(bt_stop), .ot_done_o(ot_done),
        .d16_o(d16), .ge0_o(ge0), .creq_o(creq), .cpay_o(cpay), .creq_dly_o(creq_dly),
        .creq_cap_o(creq_cap), .stretch_o(stretch));

    // ---------------------------------------------------------------- u_dly
`ifdef MUT_WRTICK_DLY_OPEN
    wire y_wrt = 1'b0;
`else
    wire y_wrt = wr_tick;
`endif
`ifdef MUT_DLY_PINMASK
    wire [3:0] y_pmask = pin_val;            // BUG: wrong PIN field to the delay unit
`else
    wire [3:0] y_pmask = pin_mask;
`endif
`ifdef MUT_DLY_NOPIN
    wire y_pwe = 1'b0;                       // BUG: PIN does not cancel a pending change
`else
    wire y_pwe = pin_we;
`endif
`ifdef MUT_DLYPS_BIT
    wire y_dlyps = r_code[10];
`else
    wire y_dlyps = r_code[11];
`endif
`ifdef MUT_ABORT_DLY_OPEN
    wire y_abort = 1'b0;
`else
    wire y_abort = abort_i;
`endif
    rosetta_delay u_dly (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(y_abort), .wr_tick_i(y_wrt),
        .cfg_dly_i(r_dly), .cfg_dlyclk_i(r_code[10]), .cfg_dlyps_i(y_dlyps),
        .pt_i(pt), .dreq_i(dreq), .dpay_i(dpay), .dreq_dly_i(dreq_dly),
        .creq_i(creq), .cpay_i(cpay), .creq_dly_i(creq_dly), .creq_cap_i(creq_cap),
        .pin_we_i(y_pwe), .pin_mask_i(y_pmask),
        .d_we_o(d_we), .d_pay_o(d_pay), .c_we_o(c_we), .c_pay_o(c_pay), .dcap_o(dcap));

    // ---------------------------------------------------------------- u_code
`ifdef MUT_WRCODE_OPEN
    wire k_wrc = 1'b0;
`else
    wire k_wrc = wr_code;
`endif
`ifdef MUT_OUTVAL_OE
    wire [1:0] k_oval = p_out_oe[1:0];       // BUG: release keeps the enables, not the values
`else
    wire [1:0] k_oval = p_out_val[1:0];
`endif
`ifdef MUT_CODE_NOPIN
    wire k_pwe = 1'b0;
`else
    wire k_pwe = pin_we;
`endif
`ifdef MUT_ST_CT_CODE
    wire k_st = ct;                          // BUG: decodes at the capture tick
`else
    wire k_st = st;
`endif
`ifdef MUT_CTE_HTE_SWAP
    wire k_cte = hte, k_hte = cte;
`else
    wire k_cte = cte, k_hte = hte;
`endif
`ifdef MUT_CODE_IN_ROT
    wire [3:0] k_in = {in_i[2:0], in_i[3]};  // BUG: lane inputs rotated on one path
`else
    wire [3:0] k_in = in_i;
`endif
`ifdef MUT_DIN_BITS
    wire [1:0] k_din = r_code[6:5];
`else
    wire [1:0] k_din = r_code[5:4];
`endif
    rosetta_coding u_code (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(abort_i),
        .wr_code_i(k_wrc), .wr_tick_i(wr_tick), .cfg_mode_i(r_tick[1:0]),
        .cfg_rx_i(r_code[1:0]), .cfg_tx_i(r_code[3:2]), .cfg_din_i(k_din),
        .cfg_inv_i(r_code[6]), .cfg_od_i(r_code[7]), .cfg_dif_i(r_code[8]),
        .cfg_dfi_i(r_code[9]), .cfg_pw0_i(r_pulse[7:0]), .cfg_pw1_i(r_pulse[15:8]),
        .in_i(k_in), .st_i(k_st), .cte_i(k_cte), .hte_i(k_hte), .ge0_i(ge0),
        .tx_go_i(tx_go), .tx_bit_i(tx_bit), .tx_rel_i(tx_rel), .raw_go_i(raw_go),
        .raw_lvl_i(raw_lvl), .pin_we_i(k_pwe), .pin_mask0_i(pin_mask[0]),
        .pin_val0_i(pin_val[0]), .pin_oe0_i(pin_oe[0]), .out_val_i(k_oval),
        .dline_o(dline), .dedge_o(dedge), .rxb_o(rxb), .rxv_o(rxv), .pw_bit_o(pw_bit),
        .dreq_o(dreq), .dpay_o(dpay), .dreq_dly_o(dreq_dly));

    // ---------------------------------------------------------------- u_field
`ifdef MUT_LTCT_SWAP
    wire f_lt = ct, f_ct = lt;
`else
    wire f_lt = lt, f_ct = ct;
`endif
`ifdef MUT_HONOUR_BIT
    wire f_hon = r_tick[12];
`else
    wire f_hon = r_tick[11];
`endif
`ifdef MUT_CRCCHK_VALUE
    wire [31:0] f_cstate = crc_value;        // BUG: CRCCHK compares the XORed value
`else
    wire [31:0] f_cstate = crc_state;
`endif
`ifdef MUT_TS_LATE
    wire [15:0] f_ts = ts_q;
`else
    wire [15:0] f_ts = ts_i;
`endif
`ifdef MUT_ABORT_FIELD_OPEN
    wire f_abort = 1'b0;
`else
    wire f_abort = abort_i;
`endif
`ifdef MUT_CMD_FIELD_OPEN
    wire [1:0] f_cmd = 2'd0;                 // BUG: CRCCHK never reaches the field
`else
    wire [1:0] f_cmd = cmd_i;
`endif
`ifdef MUT_PARK_LATE
    // BUG: the park strobe and buses are registered in the top (park_full_o
    // one cycle late; the "never park when full" rule no longer protects).
    wire f_pwe = pwe_q;  wire [1:0] f_pop = pop_q;  wire [27:0] f_pctl = pctl_q;
    wire [31:0] f_pdat = pdat_q;
`else
    wire f_pwe = park_we_i;  wire [1:0] f_pop = park_op_i;  wire [27:0] f_pctl = park_ctl_i;
    wire [31:0] f_pdat = park_data_i;
`endif
    wire [3:0] f_res_dst;  wire [31:0] f_res_data, f_sr;  wire [15:0] f_res_ts;
    wire f_park_full, f_m;
    rosetta_field u_field (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(f_abort),
        .park_we_i(f_pwe), .park_op_i(f_pop), .park_ctl_i(f_pctl),
        .park_data_i(f_pdat), .res_ack_i(res_ack_i), .cmd_i(f_cmd),
        .cmd_data_i(cmd_data_i), .ts_i(f_ts), .cfg_mode_i(r_tick[1:0]),
        .lt_i(f_lt), .ct_i(f_ct), .st_i(st), .spend_i(spend), .bt_run_i(bt_run),
        .bt_stop_i(bt_stop), .ot_done_i(ot_done), .cfg_honour_i(f_hon),
        .d16_i(d16), .dedge_i(dedge), .rxb_i(rxb), .rxv_i(rxv),
        .stuff_next_i(stuff_next), .stuff_bit_i(stuff_bit),
        .crc_state_i(f_cstate), .crc_value_i(crc_value),
        .park_full_o(f_park_full), .res_valid_o(f_res_valid), .res_dst_o(f_res_dst),
        .res_data_o(f_res_data), .res_ts_o(f_res_ts), .dst_busy_o(f_dst_busy),
        .m_o(f_m), .sr_o(f_sr),
        .cu_v_o(cu_v), .cu_fld_o(cu_fld), .cu_tx_o(cu_tx), .cu_smp_o(cu_smp), .pk_v_o(pk_v),
        .launch_req_o(launch_req), .fld_next_o(fld_next), .fld_first_o(fld_first),
        .ot_start_o(ot_start), .ot_raw_o(ot_raw), .ot_n_o(ot_n),
        .tx_go_o(tx_go), .tx_bit_o(tx_bit), .tx_rel_o(tx_rel), .raw_go_o(raw_go),
        .raw_lvl_o(raw_lvl), .pin_we_o(pin_we), .pin_mask_o(pin_mask),
        .pin_val_o(pin_val), .pin_oe_o(pin_oe), .crc_en_o(crc_en), .crc_bit_o(crc_bit),
        .stf_valid_o(stf_valid), .stf_bit_o(stf_bit), .cu_done_o(cu_done), .hold_o(hold));

    // ---------------------------------------------------------------- u_crc
`ifdef MUT_CRCW_MISSING
    wire crc_init = run_i && (cmd_i == 2'd1 || wr_crc);          // BUG: no second init
`elsif MUT_CRCINIT_CMD3
    wire crc_init = run_i && (cmd_i == 2'd3 || wr_crc || crcw_q); // BUG: wrong command
`elsif LANE_CRCW_UNGATED
    wire crc_init = (run_i && cmd_i == 2'd1) || wr_crc || crcw_q;
`else
    wire crc_init = run_i && (cmd_i == 2'd1 || wr_crc || crcw_q);
`endif
`ifdef MUT_CRC_INIT_BIT
    wire [31:0] c_init = {32{r_crccfg[6]}};
`else
    wire [31:0] c_init = {32{r_crccfg[5]}};
`endif
    rosetta_crc u_crc (
        .clk(clk), .rst_n(rst_n), .cfg_poly_i(r_crcpoly), .cfg_init_i(c_init),
        .cfg_width_m1_i(r_crccfg[4:0]), .cfg_xorout_i(r_crccfg[6]),
        .init_i(crc_init), .en_i(crc_en), .bit_i(crc_bit),
        .state_o(crc_state), .value_o(crc_value));

    // ---------------------------------------------------------------- u_stuff
`ifdef MUT_STUFF_NOHALT
    wire s_clr = (run_i && cmd_i == 2'd3) || wr_stuff;            // BUG: not held while halted
`elsif MUT_WRSTUFF_OPEN
    wire s_clr = (run_i && cmd_i == 2'd3) || !run_i;              // BUG: SETM STUFF no clear
`else
    wire s_clr = (run_i && cmd_i == 2'd3) || wr_stuff || !run_i;
`endif
    rosetta_stuff u_stuff (
        .clk(clk), .rst_n(rst_n), .cfg_en_i(r_stuff[4]), .cfg_n_i(r_stuff[2:0]),
        .cfg_mode_i(r_stuff[3]), .clear_i(s_clr),
        .bit_valid_i(stf_valid), .bit_i(stf_bit),
        .stuff_next_o(stuff_next), .stuff_bit_o(stuff_bit), .violation_o(stuff_viol));

    // ---------------------------------------------------------------- u_evt
`ifdef MUT_EVCWR_REV
    wire [3:0] v_wr = {wr_evc[0], wr_evc[1], wr_evc[2], wr_evc[3]};
`else
    wire [3:0] v_wr = wr_evc;
`endif
`ifdef MUT_STUFFEV_BIT
    wire v_sev = r_stuff[4];
`else
    wire v_sev = r_stuff[5];
`endif
`ifdef MUT_VIOL_OPEN
    wire v_viol = 1'b0;
`else
    wire v_viol = stuff_viol;
`endif
`ifdef MUT_RUN_EVT
    wire v_run = 1'b1;                       // BUG: pending bits survive a halt
`else
    wire v_run = run_i;
`endif
`ifdef MUT_EVT_INQ_RAW
    wire [3:0] v_inq = in_i;
`else
    wire [3:0] v_inq = in_q;
`endif
`ifdef MUT_PT_EVT_ONE
    wire v_pt = 1'b1;                        // BUG: windows ignore the prescaler
`else
    wire v_pt = pt;
`endif
    rosetta_events u_evt (
        .clk(clk), .rst_n(rst_n), .run_i(v_run), .in_i(in_i), .in_q_i(v_inq), .pt_i(v_pt),
        .cfg_evc_i(r_evc), .evc_wr_i(v_wr), .cfg_stuffev_i(v_sev),
        .stuff_viol_i(v_viol), .ev_clr_i(ev_clr_i),
        .ev_pend_o(ev_pend_o), .fire_o(fire));

    // ---------------------------------------------------------------- u_pins
`ifdef MUT_RUN_PINS
    wire p_run = 1'b1;                       // BUG: halted lane still enables outputs
`else
    wire p_run = run_i;
`endif
    rosetta_pins u_pins (
        .clk(clk), .rst_n(rst_n), .run_i(p_run), .d_we_i(d_we), .d_pay_i(d_pay),
        .c_we_i(c_we), .c_pay_i(c_pay), .pin_we_i(pin_we), .pin_mask_i(pin_mask),
        .pin_val_i(pin_val), .pin_oe_i(pin_oe),
        .out_val_o(p_out_val), .out_oe_o(p_out_oe));

    // ---------------------------------------------------------------- outputs
`ifdef MUT_BUSY_NO_BT
    assign busy_o = cu_v;                    // BUG: busy ignores the BAUD timer
`elsif MUT_COMB_BUSY
    assign busy_o = cu_v || bt_run || park_we_i;   // BUG: busy already in the park cycle (comb path)
`else
    assign busy_o = cu_v || bt_run;
`endif
`ifdef MUT_CRCO_STATE
    assign crc_o = crc_state;                // BUG: raw state instead of value_o
`else
    assign crc_o = crc_value;
`endif
`ifdef MUT_RESV_LATE
    assign res_valid_o = rv_q;               // BUG: result valid one cycle late
`elsif MUT_COMB_ACK
    assign res_valid_o = f_res_valid && !res_ack_i;   // BUG: "fast" ack, comb path to an output
`else
    assign res_valid_o = f_res_valid;
`endif
`ifdef MUT_DSTBUSY_BUSY
    assign dst_busy_o = f_dst_busy & {4{busy_o}};   // BUG: held result not marked
`else
    assign dst_busy_o = f_dst_busy;
`endif
    assign park_full_o = f_park_full;
    assign res_dst_o   = f_res_dst;
    assign res_data_o  = f_res_data;
    assign res_ts_o    = f_res_ts;
    assign m_o         = f_m;
    assign sr_o        = f_sr;
    assign out_val_o   = p_out_val;
    assign out_oe_o    = p_out_oe;
endmodule
