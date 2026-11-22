`default_nettype none
`include "lane_wb.svh"
// ===========================================================================
//  Zero rule of the H3/H4 amendment (amendment-eth-can.md, "Zero rule"; ruling
//  59) at lane level: with TICK[15:14] and CODE[12] never written as 1, the
//  amended lane is cycle for cycle today's lane.
//
//   u_new  the amended lane (lane.sby's design line; real sub-blocks).
//   u_old  today's lane: the lane and its nine sub-blocks at the pinned
//          pre-amendment revision (gen_zero.py, modules renamed *_h).
//   u_chk  lane_props on u_new for the sequencer environment (u_*), WB = 0,
//          with LANE_ZERO_H34: SETM TICK writes have data[15:14] = 0 and
//          SETM CODE writes data[12] = 0 (the "new bits never written"
//          assumption; covers c_z_* show TICK and CODE writes still happen).
//  Assertions a_z_*: every output of u_new equals u_old's in every cycle
//  (result fields while res_valid_o, as layer G). Both lanes get the same
//  inputs and the same reset.
// ===========================================================================
module lane_zero_fv (
    input wire         clk,
    input wire         rst_n,
    input wire         run_i, park_we_i, res_ack_i, abort_i, cfg_we_i, cfg_hi_i, data_wait_i,
    input wire [1:0]   park_op_i, cmd_i,
    input wire [27:0]  park_ctl_i,
    input wire [31:0]  park_data_i, cmd_data_i,
    input wire [3:0]   ev_clr_i, in_i,
    input wire [4:0]   cfg_reg_i,
    input wire [15:0]  cfg_data_i, ts_i
);
    wire        busy, pf, rv, m, o_busy, o_pf, o_rv, o_m;
    wire [3:0]  rdst, dbusy, evp, oval, ooe, o_rdst, o_dbusy, o_evp, o_oval, o_ooe;
    wire [31:0] rdata, sr, crc, o_rdata, o_sr, o_crc;
    wire [15:0] rts, o_rts;
`define ZLANE_IN \
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .park_we_i(park_we_i), .park_op_i(park_op_i), \
        .park_ctl_i(park_ctl_i), .park_data_i(park_data_i), .res_ack_i(res_ack_i), \
        .abort_i(abort_i), .ev_clr_i(ev_clr_i), .cfg_we_i(cfg_we_i), .cfg_reg_i(cfg_reg_i), \
        .cfg_hi_i(cfg_hi_i), .cfg_data_i(cfg_data_i), .cmd_i(cmd_i), .cmd_data_i(cmd_data_i), \
        .data_wait_i(data_wait_i), .in_i(in_i), .ts_i(ts_i)
    rosetta_lane u_new (`ZLANE_IN,
        .busy_o(busy), .park_full_o(pf), .res_valid_o(rv), .res_dst_o(rdst), .res_data_o(rdata),
        .res_ts_o(rts), .dst_busy_o(dbusy), .m_o(m), .ev_pend_o(evp), .sr_o(sr), .crc_o(crc),
        .out_val_o(oval), .out_oe_o(ooe));
    rosetta_lane_h u_old (`ZLANE_IN,
        .busy_o(o_busy), .park_full_o(o_pf), .res_valid_o(o_rv), .res_dst_o(o_rdst),
        .res_data_o(o_rdata), .res_ts_o(o_rts), .dst_busy_o(o_dbusy), .m_o(o_m), .ev_pend_o(o_evp),
        .sr_o(o_sr), .crc_o(o_crc), .out_val_o(o_oval), .out_oe_o(o_ooe));

    wire unused_cin;  wire [1:0] unused_clkin;
    lane_props #(.MAIN(1), .WB(0)) u_chk (`ZLANE_IN,
        .busy_o(busy), .park_full_o(pf), .res_valid_o(rv), .res_dst_o(rdst), .res_data_o(rdata),
        .res_ts_o(rts), .dst_busy_o(dbusy), .m_o(m), .ev_pend_o(evp), .sr_o(sr), .crc_o(crc),
        .out_val_o(oval), .out_oe_o(ooe), .wb({`LANE_WBW{1'b0}}),
        .mclk_i(1'b1), .r53_cin_o(unused_cin), .sh_clkin_o(unused_clkin));

    always @* if (rst_n) begin
        a_z_busy:  assert (busy  == o_busy);
        a_z_pf:    assert (pf    == o_pf);
        a_z_rv:    assert (rv    == o_rv);
        if (rv) a_z_res: assert (rdst == o_rdst && rdata == o_rdata && rts == o_rts);
        a_z_dbusy: assert (dbusy == o_dbusy);
        a_z_m:     assert (m     == o_m);
        a_z_evp:   assert (evp   == o_evp);
        a_z_sr:    assert (sr    == o_sr);
        a_z_crc:   assert (crc   == o_crc);
        a_z_oval:  assert (oval  == o_oval);
        a_z_ooe:   assert (ooe   == o_ooe);
    end
    // the assumption leaves TICK and CODE writes possible
    always @* if (rst_n) begin
        c_z_tick: cover (run_i && cfg_we_i && !cfg_hi_i && cfg_reg_i == 5'd0 && cfg_data_i[1:0] == 2'd2);
        c_z_code: cover (run_i && cfg_we_i && !cfg_hi_i && cfg_reg_i == 5'd5 && cfg_data_i[11:0] != 12'd0);
    end
endmodule
