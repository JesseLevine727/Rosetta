`default_nettype none
`include "lane_wb.svh"
// ===========================================================================
//  Proof top for rosetta_lane.
//
//   u_dut  the design, every input free under the sequencer guarantees
//          (u_chk = lane_props, MAIN = 1). White-box bus wb_dut is tied to
//          u_dut's sub-block ports by lane_wb_dut.ys after flatten.
//   u_gold (FV_GOLD) lane_model: the spec composition, same inputs as
//          u_dut. Layer G: every lane output of the design equals the
//          model's in every cycle (result fields while res_valid_o).
//   u_bus  (FV_BUS) a second copy of the design for lane.md 9.3 (shared-bus
//          lemma): identical inputs except park_op/ctl/data in cycles with
//          park_we_i = 0, cfg_reg/hi/data with cfg_we_i = 0 and cmd_data
//          with cmd_i != 2, which are free there. Every output must equal
//          u_dut's in every cycle.
//   u_r53  (FV_R53) a copy of the design whose clock-input pin
//          (in_i[TICK.clkin]) is the pin model of lane_props (master clock
//          wired-AND with out3, 3 cycles of input latency, compiler rule of
//          ruling 53); its own checker (MAIN = 0) asserts that launch and
//          capture tick never coincide (wb_r53, lane_wb_r53.ys).
//  FV_NO_MAIN drops u_dut (for the R53 task).
//  LANE_NO_WHITEBOX: u_chk without layers W, E, L (port level P and G only),
//  for a design whose sub-block instances do not carry the lane.md 3 names
//  (then also remove the `script lane_wb_dut.ys` lines from lane.sby).
// ===========================================================================
`ifdef LANE_NO_WHITEBOX
  `define LANE_WBP 0
`else
  `define LANE_WBP 1
`endif
module lane_fv (
    input wire         clk,
    input wire         rst_n,
    input wire         run_i, park_we_i, res_ack_i, abort_i, cfg_we_i, cfg_hi_i, data_wait_i,
    input wire [1:0]   park_op_i, cmd_i,
    input wire [27:0]  park_ctl_i,
    input wire [31:0]  park_data_i, cmd_data_i,
    input wire [3:0]   ev_clr_i, in_i,
    input wire [4:0]   cfg_reg_i,
    input wire [15:0]  cfg_data_i, ts_i,
    // free values for the bus copy
    input wire [1:0]   b_park_op,
    input wire [27:0]  b_park_ctl,
    input wire [31:0]  b_park_data, b_cmd_data,
    input wire [4:0]   b_cfg_reg,
    input wire         b_cfg_hi,
    input wire [15:0]  b_cfg_data,
    // R53 copy
    input wire         r_rst_n,
    input wire         r_run_i, r_park_we_i, r_res_ack_i, r_abort_i, r_cfg_we_i, r_cfg_hi_i,
    input wire         r_data_wait_i, r_mclk,
    input wire [1:0]   r_park_op_i, r_cmd_i,
    input wire [27:0]  r_park_ctl_i,
    input wire [31:0]  r_park_data_i, r_cmd_data_i,
    input wire [3:0]   r_ev_clr_i, r_in_free,
    input wire [4:0]   r_cfg_reg_i,
    input wire [15:0]  r_cfg_data_i, r_ts_i
);
`ifndef FV_NO_MAIN
    // ------------------------------------------------------------- u_dut
    wire        busy, pf, rv, m;
    wire [3:0]  rdst, dbusy, evp, oval, ooe;
    wire [31:0] rdata, sr, crc;
    wire [15:0] rts;
    rosetta_lane u_dut (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .park_we_i(park_we_i), .park_op_i(park_op_i),
        .park_ctl_i(park_ctl_i), .park_data_i(park_data_i), .res_ack_i(res_ack_i),
        .abort_i(abort_i), .ev_clr_i(ev_clr_i), .cfg_we_i(cfg_we_i), .cfg_reg_i(cfg_reg_i),
        .cfg_hi_i(cfg_hi_i), .cfg_data_i(cfg_data_i), .cmd_i(cmd_i), .cmd_data_i(cmd_data_i),
        .data_wait_i(data_wait_i), .in_i(in_i), .ts_i(ts_i),
        .busy_o(busy), .park_full_o(pf), .res_valid_o(rv), .res_dst_o(rdst), .res_data_o(rdata),
        .res_ts_o(rts), .dst_busy_o(dbusy), .m_o(m), .ev_pend_o(evp), .sr_o(sr), .crc_o(crc),
        .out_val_o(oval), .out_oe_o(ooe));

    wire [`LANE_WBW-1:0] wb_dut;      // driven by lane_wb_dut.ys (connect after flatten)
    wire unused_cin;  wire [1:0] unused_clkin;
    lane_props #(.MAIN(1), .WB(`LANE_WBP)) u_chk (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .park_we_i(park_we_i), .park_op_i(park_op_i),
        .park_ctl_i(park_ctl_i), .park_data_i(park_data_i), .res_ack_i(res_ack_i),
        .abort_i(abort_i), .ev_clr_i(ev_clr_i), .cfg_we_i(cfg_we_i), .cfg_reg_i(cfg_reg_i),
        .cfg_hi_i(cfg_hi_i), .cfg_data_i(cfg_data_i), .cmd_i(cmd_i), .cmd_data_i(cmd_data_i),
        .data_wait_i(data_wait_i), .in_i(in_i), .ts_i(ts_i),
        .busy_o(busy), .park_full_o(pf), .res_valid_o(rv), .res_dst_o(rdst), .res_data_o(rdata),
        .res_ts_o(rts), .dst_busy_o(dbusy), .m_o(m), .ev_pend_o(evp), .sr_o(sr), .crc_o(crc),
        .out_val_o(oval), .out_oe_o(ooe), .wb(wb_dut),
        .mclk_i(1'b1), .r53_cin_o(unused_cin), .sh_clkin_o(unused_clkin));

`ifdef FV_GOLD
    // ------------------------------------------------------------- u_gold
    wire        g_busy, g_pf, g_rv, g_m;
    wire [3:0]  g_rdst, g_dbusy, g_evp, g_oval, g_ooe;
    wire [31:0] g_rdata, g_sr, g_crc;
    wire [15:0] g_rts;
    lane_model u_gold (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .park_we_i(park_we_i), .park_op_i(park_op_i),
        .park_ctl_i(park_ctl_i), .park_data_i(park_data_i), .res_ack_i(res_ack_i),
        .abort_i(abort_i), .ev_clr_i(ev_clr_i), .cfg_we_i(cfg_we_i), .cfg_reg_i(cfg_reg_i),
        .cfg_hi_i(cfg_hi_i), .cfg_data_i(cfg_data_i), .cmd_i(cmd_i), .cmd_data_i(cmd_data_i),
        .data_wait_i(data_wait_i), .in_i(in_i), .ts_i(ts_i),
        .busy_o(g_busy), .park_full_o(g_pf), .res_valid_o(g_rv), .res_dst_o(g_rdst),
        .res_data_o(g_rdata), .res_ts_o(g_rts), .dst_busy_o(g_dbusy), .m_o(g_m),
        .ev_pend_o(g_evp), .sr_o(g_sr), .crc_o(g_crc), .out_val_o(g_oval), .out_oe_o(g_ooe));
    always @* if (rst_n) begin
        a_g_busy:  assert (busy  == g_busy);
        a_g_pf:    assert (pf    == g_pf);
        a_g_rv:    assert (rv    == g_rv);
        if (rv) a_g_res: assert (rdst == g_rdst && rdata == g_rdata && rts == g_rts);
        a_g_dbusy: assert (dbusy == g_dbusy);
        a_g_m:     assert (m     == g_m);
        a_g_evp:   assert (evp   == g_evp);
        a_g_sr:    assert (sr    == g_sr);
        a_g_crc:   assert (crc   == g_crc);
        a_g_oval:  assert (oval  == g_oval);
        a_g_ooe:   assert (ooe   == g_ooe);
    end
`endif

`ifdef FV_BUS
    // ------------------------------------------------------------- u_bus
    wire        x_busy, x_pf, x_rv, x_m;
    wire [3:0]  x_rdst, x_dbusy, x_evp, x_oval, x_ooe;
    wire [31:0] x_rdata, x_sr, x_crc;
    wire [15:0] x_rts;
    rosetta_lane u_bus (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .park_we_i(park_we_i),
        .park_op_i(park_we_i ? park_op_i : b_park_op),
        .park_ctl_i(park_we_i ? park_ctl_i : b_park_ctl),
        .park_data_i(park_we_i ? park_data_i : b_park_data), .res_ack_i(res_ack_i),
        .abort_i(abort_i), .ev_clr_i(ev_clr_i), .cfg_we_i(cfg_we_i),
        .cfg_reg_i(cfg_we_i ? cfg_reg_i : b_cfg_reg), .cfg_hi_i(cfg_we_i ? cfg_hi_i : b_cfg_hi),
        .cfg_data_i(cfg_we_i ? cfg_data_i : b_cfg_data), .cmd_i(cmd_i),
        .cmd_data_i(cmd_i == 2'd2 ? cmd_data_i : b_cmd_data),
        .data_wait_i(data_wait_i), .in_i(in_i), .ts_i(ts_i),
        .busy_o(x_busy), .park_full_o(x_pf), .res_valid_o(x_rv), .res_dst_o(x_rdst),
        .res_data_o(x_rdata), .res_ts_o(x_rts), .dst_busy_o(x_dbusy), .m_o(x_m),
        .ev_pend_o(x_evp), .sr_o(x_sr), .crc_o(x_crc), .out_val_o(x_oval), .out_oe_o(x_ooe));
    always @* if (rst_n) begin
        a_p3_bus: assert ({busy, pf, rv, rdst, rdata, rts, dbusy, m, evp, sr, crc, oval, ooe} ==
                          {x_busy, x_pf, x_rv, x_rdst, x_rdata, x_rts, x_dbusy, x_m, x_evp, x_sr,
                           x_crc, x_oval, x_ooe});
    end
`endif
`endif

`ifdef FV_R53
    // ------------------------------------------------------------- u_r53
    wire        r_busy, r_pf, r_rv, r_m;
    wire [3:0]  r_rdst, r_dbusy, r_evp, r_oval, r_ooe;
    wire [31:0] r_rdata, r_sr, r_crc;
    wire [15:0] r_rts;
    wire        r_cin;  wire [1:0] r_clkin;
    wire [3:0]  r_in = (r_in_free & ~(4'd1 << r_clkin)) | ({3'd0, r_cin} << r_clkin);
    rosetta_lane u_r53 (
        .clk(clk), .rst_n(r_rst_n), .run_i(r_run_i), .park_we_i(r_park_we_i),
        .park_op_i(r_park_op_i), .park_ctl_i(r_park_ctl_i), .park_data_i(r_park_data_i),
        .res_ack_i(r_res_ack_i), .abort_i(r_abort_i), .ev_clr_i(r_ev_clr_i),
        .cfg_we_i(r_cfg_we_i), .cfg_reg_i(r_cfg_reg_i), .cfg_hi_i(r_cfg_hi_i),
        .cfg_data_i(r_cfg_data_i), .cmd_i(r_cmd_i), .cmd_data_i(r_cmd_data_i),
        .data_wait_i(r_data_wait_i), .in_i(r_in), .ts_i(r_ts_i),
        .busy_o(r_busy), .park_full_o(r_pf), .res_valid_o(r_rv), .res_dst_o(r_rdst),
        .res_data_o(r_rdata), .res_ts_o(r_rts), .dst_busy_o(r_dbusy), .m_o(r_m),
        .ev_pend_o(r_evp), .sr_o(r_sr), .crc_o(r_crc), .out_val_o(r_oval), .out_oe_o(r_ooe));
    wire [`LANE_WBW-1:0] wb_r53;      // driven by lane_wb_r53.ys
    lane_props #(.MAIN(0), .WB(1)) u_r53chk (
        .clk(clk), .rst_n(r_rst_n), .run_i(r_run_i), .park_we_i(r_park_we_i),
        .park_op_i(r_park_op_i), .park_ctl_i(r_park_ctl_i), .park_data_i(r_park_data_i),
        .res_ack_i(r_res_ack_i), .abort_i(r_abort_i), .ev_clr_i(r_ev_clr_i),
        .cfg_we_i(r_cfg_we_i), .cfg_reg_i(r_cfg_reg_i), .cfg_hi_i(r_cfg_hi_i),
        .cfg_data_i(r_cfg_data_i), .cmd_i(r_cmd_i), .cmd_data_i(r_cmd_data_i),
        .data_wait_i(r_data_wait_i), .in_i(r_in), .ts_i(r_ts_i),
        .busy_o(r_busy), .park_full_o(r_pf), .res_valid_o(r_rv), .res_dst_o(r_rdst),
        .res_data_o(r_rdata), .res_ts_o(r_rts), .dst_busy_o(r_dbusy), .m_o(r_m),
        .ev_pend_o(r_evp), .sr_o(r_sr), .crc_o(r_crc), .out_val_o(r_oval), .out_oe_o(r_ooe),
        .wb(wb_r53), .mclk_i(r_mclk), .r53_cin_o(r_cin), .sh_clkin_o(r_clkin));
`endif
endmodule
