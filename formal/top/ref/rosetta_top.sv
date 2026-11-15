`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL of the chip top (NOT the design).
//
//  Written from docs/blocks/top.md (sections 1-4) and decisions.md rulings
//  2, 14, 15, 30, 31 only; src/rosetta_top.sv was not read. It instantiates
//  the real component RTL (src/rosetta_*.sv), including src/rosetta_lane.sv
//  as a black box by port. Its only job is to show that the top-level proofs
//  in formal/top pass on a correct top and fail on each planted integration
//  bug (`ifdef MUT_<NAME>`, one at a time, see ref/run_mutants.sh).
//
//  Spec readings (see the header of formal/top/top_props.sv, "Spec
//  readings"): lane run_i is driven from u_seq.run_o (top.md 4); the
//  sequencer's lane_run_o is left unused (sequencer.md 2.6: equal to run_o,
//  proven as seq a_p5_lane_run). Lane ts_i = ts_o[15:0] (lane.md 2.2,
//  ruling 30). Lane RTRIP = 4 (lane.md 1, ruling 18).
// ===========================================================================
module tt_um_jesselevine727_rosetta (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
  // ------------------------------------------------------------ reset (3)
  logic [1:0] rst_sync;
`ifdef MUT_RST_SYNC_SYNC
  // planted: the synchroniser asserts synchronously (not at rst_n falling)
  always_ff @(posedge clk)
    if (!rst_n) rst_sync <= 2'b00; else rst_sync <= {rst_sync[0], 1'b1};
`else
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) rst_sync <= 2'b00; else rst_sync <= {rst_sync[0], 1'b1};
`endif
  wire rst_i_n;
`ifdef MUT_RST_SYNC_ONE
  assign rst_i_n = rst_sync[0];          // planted: one synchroniser flop
`else
  assign rst_i_n = rst_sync[1];
`endif

  // per-block reset (normally all rst_i_n)
  wire rst_xbar = rst_i_n;
`ifdef MUT_RST_LANE2_RAW
  wire rst_lane2 = rst_n;                // planted: lane 2 on the raw pin
`else
  wire rst_lane2 = rst_i_n;
`endif
`ifdef MUT_RST_SEQ_OPEN
  wire rst_seq = 1'b1;                   // planted: reset does not reach seq
`else
  wire rst_seq = rst_i_n;
`endif
`ifdef MUT_RST_MEMDATA_OPEN
  wire rst_mem = 1'b1;                   // planted: reset does not reach mem_data
`else
  wire rst_mem = rst_i_n;
`endif

  wire _unused = &{1'b0, ena};

  // ------------------------------------------------------------ nets
  wire [4:0]   ph;
  wire [8:0]   pm_addr;
  wire [31:0]  pm_rdata;
  wire [2:0]   dm_op;
  wire [6:0]   dm_off;
  wire [1:0]   dm_hmask;
  wire [31:0]  dm_wdata, dm_rdata;
  wire [3:0]   qe, qf, qne;
  wire         ch_put, ch_get;
  wire [1:0]   ch_idx;
  wire [31:0]  ch_wdata;
  wire [3:0]   ch_valid;
  wire [127:0] ch_data;
  wire [31:0]  ts;
  wire         hreg_we;
  wire [6:0]   hreg_addr;
  wire [31:0]  hreg_wdata, hreg_rdata;
  wire [31:0]  rd_seq, rd_mem, rd_chan, rd_xbar;
  wire [3:0]   run, attn, lane_run_unused;
  wire         h_pm_en, h_pm_we, h_dm_en, h_dm_we;
  wire [8:0]   h_pm_addr, h_dm_addr;
  wire [31:0]  h_pm_wdata, h_dm_wdata;
  wire         host_sclk, host_csn, host_mosi, host_miso, host_irq;
  wire [15:0]  lane_in, lane_val, lane_oe;
  wire [7:0]   x_uo, x_uio_out, x_uio_oe;

  // sequencer <-> lanes
  wire [3:0]   l_park_we, l_res_ack, l_abort, l_cfg_we, l_data_wait;
  wire [1:0]   l_park_op;
  wire [27:0]  l_park_ctl;
  wire [31:0]  l_park_data, l_cmd_data;
  wire [15:0]  l_ev_clr;
  wire [4:0]   l_cfg_reg;
  wire         l_cfg_hi;
  wire [15:0]  l_cfg_data;
  wire [7:0]   l_cmd;
  wire [3:0]   l_busy, l_park_full, l_res_valid, l_m;
  wire [15:0]  l_res_dst, l_dst_busy, l_ev_pend;
  wire [127:0] l_res_data, l_sr, l_crc;
  wire [63:0]  l_res_ts;

  // ------------------------------------------------------------ TT pins (2)
  wire [7:0] ui_x, uio_x;
`ifdef MUT_PIN_UI_SWAP
  assign ui_x = {ui_in[7:5], ui_in[3], ui_in[4], ui_in[2:0]};   // planted
`elsif MUT_XBAR_UIO_FROM_UI
  assign ui_x = ui_in;
`elsif MUT_EXTRA_SYNC
  logic [7:0] ui_q;                                             // planted
  always_ff @(posedge clk or negedge rst_i_n)
    if (!rst_i_n) ui_q <= 8'h02; else ui_q <= ui_in;
  assign ui_x = ui_q;
`else
  assign ui_x = ui_in;
`endif
`ifdef MUT_XBAR_UIO_FROM_UI
  assign uio_x = ui_in;                                         // planted
`else
  assign uio_x = uio_in;
`endif

`ifdef MUT_ENA_GATE
  assign uo_out  = x_uo & {8{ena}};                             // planted
`elsif MUT_UO01_SWAP
  assign uo_out  = {x_uo[7:2], x_uo[0], x_uo[1]};               // planted
`elsif MUT_COMB_PIN
  assign uo_out  = {x_uo[7:3], x_uo[2] | ui_in[3], x_uo[1:0]};  // planted: input straight to an output
`elsif MUT_UO_PIN_SHIFT
  assign uo_out  = {x_uo[6:2], x_uo[7], x_uo[1:0]};             // planted
`else
  assign uo_out  = x_uo;
`endif
  assign uio_out = x_uio_out;
`ifdef MUT_UIO_OE_POL
  assign uio_oe  = ~x_uio_oe;                                   // planted
`else
  assign uio_oe  = x_uio_oe;
`endif

  // ------------------------------------------------------------ crossbar
  wire [3:0] x_run;
`ifdef MUT_XBAR_RUN_ONES
  assign x_run = 4'hF;                                          // planted
`else
  assign x_run = run;
`endif
  wire x_miso, x_irq;
`ifdef MUT_MISO_IRQ_SWAP
  assign x_miso = host_irq;  assign x_irq = host_miso;          // planted
`else
  assign x_miso = host_miso; assign x_irq = host_irq;
`endif
  wire [15:0] x_lval, x_loe;
`ifdef MUT_LANE_SWAP_BUS
  assign x_lval = {lane_val[15:12], lane_val[7:4], lane_val[11:8], lane_val[3:0]};  // planted
`else
  assign x_lval = lane_val;
`endif
`ifdef MUT_OE_FROM_VAL
  assign x_loe = lane_val;                                      // planted
`else
  assign x_loe = lane_oe;
`endif

  rosetta_xbar u_xbar (
      .clk         (clk),
      .rst_n       (rst_xbar),
      .ui_in_i     (ui_x),
      .uio_in_i    (uio_x),
      .host_sclk_o (host_sclk),
      .host_csn_o  (host_csn),
      .host_mosi_o (host_mosi),
      .host_miso_i (x_miso),
      .host_irq_i  (x_irq),
      .lane_in_o   (lane_in),
      .lane_val_i  (x_lval),
      .lane_oe_i   (x_loe),
      .run_i       (x_run),
      .uo_out_o    (x_uo),
      .uio_out_o   (x_uio_out),
      .uio_oe_o    (x_uio_oe),
      .hreg_we_i   (hreg_we),
      .hreg_addr_i (hreg_addr),
      .hreg_wdata_i(hreg_wdata),
      .hreg_rdata_o(rd_xbar),
      .obs_insel_o (),
      .obs_outsel_o(),
      .obs_loop_o  ()
  );

  // ------------------------------------------------------------ host port
  wire h_sclk;
`ifdef MUT_HOST_SYNC_BYPASS
  assign h_sclk = ui_in[0];                                     // planted
`else
  assign h_sclk = host_sclk;
`endif
  wire [4:0] h_ph;
`ifdef MUT_HOST_PH_ROT
  assign h_ph = {ph[3:0], ph[4]};                               // planted
`else
  assign h_ph = ph;
`endif
  wire [3:0] h_attn, h_qne;
`ifdef MUT_ATTN_ZERO
  assign h_attn = 4'h0;                                         // planted
`else
  assign h_attn = attn;
`endif
`ifdef MUT_QNE_FROM_QE
  assign h_qne = qe;                                            // planted
`else
  assign h_qne = qne;
`endif
`ifdef MUT_RDATA_NO_XBAR
  assign hreg_rdata = rd_seq | rd_mem | rd_chan;                // planted
`else
  assign hreg_rdata = rd_seq | rd_mem | rd_chan | rd_xbar;
`endif

  rosetta_host u_host (
      .clk         (clk),
      .rst_n       (rst_i_n),
      .ph_i        (h_ph),
      .sclk_i      (h_sclk),
      .csn_i       (host_csn),
      .mosi_i      (host_mosi),
      .miso_o      (host_miso),
      .irq_o       (host_irq),
      .h_pm_en_o   (h_pm_en),
      .h_pm_we_o   (h_pm_we),
      .h_pm_addr_o (h_pm_addr),
      .h_pm_wdata_o(h_pm_wdata),
      .pm_rdata_i  (pm_rdata),
      .h_dm_en_o   (h_dm_en),
      .h_dm_we_o   (h_dm_we),
      .h_dm_addr_o (h_dm_addr),
      .h_dm_wdata_o(h_dm_wdata),
      .dm_rdata_i  (dm_rdata),
      .hreg_we_o   (hreg_we),
      .hreg_addr_o (hreg_addr),
      .hreg_wdata_o(hreg_wdata),
      .hreg_rdata_i(hreg_rdata),
      .attn_i      (h_attn),
      .qne_i       (h_qne),
      .obs_sr_o    (),
      .obs_cmd_o   (),
      .obs_bitcnt_o(),
      .obs_maddr_o (),
      .obs_hst_o   (),
      .obs_imask_o ()
  );

  // ------------------------------------------------------------ memories
  wire       mp_en;
`ifdef MUT_HOST_DM_TO_PM
  assign mp_en = h_dm_en;                                       // planted
`else
  assign mp_en = h_pm_en;
`endif
  wire [4:0] mp_ph;
`ifdef MUT_PH_PM_ROT
  assign mp_ph = {ph[0], ph[4:1]};                              // planted
`else
  assign mp_ph = ph;
`endif

  rosetta_mem_prog u_mem_prog (
      .clk          (clk),
      .ph_i         (mp_ph),
      .fetch_addr_i (pm_addr),
      .h_en_i       (mp_en),
      .h_we_i       (h_pm_we),
      .h_addr_i     (h_pm_addr),
      .h_wdata_i    (h_pm_wdata),
      .rdata_o      (pm_rdata)
  );

  wire [3:0] md_run;
`ifdef MUT_MEMDATA_RUN_ZERO
  assign md_run = 4'h0;                                         // planted
`else
  assign md_run = run;
`endif
  wire [6:0] md_off;
`ifdef MUT_DM_OFF_FROM_PM
  assign md_off = pm_addr[6:0];                                 // planted
`else
  assign md_off = dm_off;
`endif

  rosetta_mem_data u_mem_data (
      .clk          (clk),
      .rst_n        (rst_mem),
      .ph_i         (ph),
      .dm_op_i      (dm_op),
      .dm_off_i     (md_off),
      .dm_hmask_i   (dm_hmask),
      .dm_wdata_i   (dm_wdata),
      .h_en_i       (h_dm_en),
      .h_we_i       (h_dm_we),
      .h_addr_i     (h_dm_addr),
      .h_wdata_i    (h_dm_wdata),
      .run_i        (md_run),
      .hreg_we_i    (hreg_we),
      .hreg_addr_i  (hreg_addr),
      .hreg_wdata_i (hreg_wdata),
      .hreg_rdata_o (rd_mem),
      .rdata_o      (dm_rdata),
      .qe_o         (qe),
      .qf_o         (qf),
      .qne_o        (qne),
      .obs_qcfg_o   (),
      .obs_in_wr_o  (),
      .obs_in_rd_o  (),
      .obs_out_wr_o (),
      .obs_out_rd_o ()
  );

  // ------------------------------------------------------------ channels
  wire c_we;
`ifdef MUT_CHAN_NO_HREG
  assign c_we = 1'b0;                                           // planted
`else
  assign c_we = hreg_we;
`endif

  rosetta_chan u_chan (
      .clk          (clk),
      .rst_n        (rst_i_n),
      .ch_put_i     (ch_put),
      .ch_get_i     (ch_get),
      .ch_idx_i     (ch_idx),
      .ch_wdata_i   (ch_wdata),
      .ch_valid_o   (ch_valid),
      .ch_data_o    (ch_data),
      .ts_o         (ts),
      .hreg_we_i    (c_we),
      .hreg_addr_i  (hreg_addr),
      .hreg_wdata_i (hreg_wdata),
      .hreg_rdata_o (rd_chan)
  );

  // ------------------------------------------------------------ sequencer
  wire [31:0] s_pm_rdata, s_dm_rdata;
`ifdef MUT_PM_DM_CROSS
  assign s_pm_rdata = dm_rdata;  assign s_dm_rdata = pm_rdata;  // planted
`else
  assign s_pm_rdata = pm_rdata;  assign s_dm_rdata = dm_rdata;
`endif
  wire [3:0] s_qe, s_qf;
`ifdef MUT_QE_QF_SWAP
  assign s_qe = qf;  assign s_qf = qe;                          // planted
`else
  assign s_qe = qe;  assign s_qf = qf;
`endif
  wire [127:0] s_ch_data;
`ifdef MUT_CH_DATA_ROT
  assign s_ch_data = {ch_data[95:0], ch_data[127:96]};          // planted
`else
  assign s_ch_data = ch_data;
`endif
  wire [127:0] s_res_data;
`ifdef MUT_RES_DATA_SWAP
  assign s_res_data = {l_res_data[127:64], l_res_data[31:0], l_res_data[63:32]};  // planted
`else
  assign s_res_data = l_res_data;
`endif
  wire [15:0] s_ev_pend;
`ifdef MUT_EV_PEND_SWAP
  assign s_ev_pend = {l_ev_pend[11:8], l_ev_pend[15:12], l_ev_pend[7:0]};  // planted
`else
  assign s_ev_pend = l_ev_pend;
`endif

  rosetta_seq #(.N_LANES(4)) u_seq (
      .clk              (clk),
      .rst_n            (rst_seq),
      .ph_o             (ph),
      .pm_addr_o        (pm_addr),
      .pm_rdata_i       (s_pm_rdata),
      .dm_op_o          (dm_op),
      .dm_off_o         (dm_off),
      .dm_hmask_o       (dm_hmask),
      .dm_wdata_o       (dm_wdata),
      .dm_rdata_i       (s_dm_rdata),
      .qe_i             (s_qe),
      .qf_i             (s_qf),
      .ch_put_o         (ch_put),
      .ch_get_o         (ch_get),
      .ch_idx_o         (ch_idx),
      .ch_wdata_o       (ch_wdata),
      .ch_valid_i       (ch_valid),
      .ch_data_i        (s_ch_data),
      .ts_i             (ts),
      .hreg_we_i        (hreg_we),
      .hreg_addr_i      (hreg_addr),
      .hreg_wdata_i     (hreg_wdata),
      .hreg_rdata_o     (rd_seq),
      .run_o            (run),
      .attn_o           (attn),
      .lane_run_o       (lane_run_unused),
      .lane_park_we_o   (l_park_we),
      .lane_park_op_o   (l_park_op),
      .lane_park_ctl_o  (l_park_ctl),
      .lane_park_data_o (l_park_data),
      .lane_res_ack_o   (l_res_ack),
      .lane_abort_o     (l_abort),
      .lane_ev_clr_o    (l_ev_clr),
      .lane_cfg_we_o    (l_cfg_we),
      .lane_cfg_reg_o   (l_cfg_reg),
      .lane_cfg_hi_o    (l_cfg_hi),
      .lane_cfg_data_o  (l_cfg_data),
      .lane_cmd_o       (l_cmd),
      .lane_cmd_data_o  (l_cmd_data),
      .lane_data_wait_o (l_data_wait),
      .lane_busy_i      (l_busy),
      .lane_park_full_i (l_park_full),
      .lane_res_valid_i (l_res_valid),
      .lane_res_dst_i   (l_res_dst),
      .lane_res_data_i  (s_res_data),
      .lane_res_ts_i    (l_res_ts),
      .lane_dst_busy_i  (l_dst_busy),
      .lane_m_i         (l_m),
      .lane_ev_pend_i   (s_ev_pend),
      .lane_sr_i        (l_sr),
      .lane_crc_i       (l_crc),
      .obs_pc_o         (),
      .obs_r_o          (),
      .obs_z_o          (),
      .obs_n_o          (),
      .obs_pause_o      (),
      .obs_step_o       (),
      .obs_armed_o      (),
      .obs_tgt_o        (),
      .obs_wsta_o       (),
      .obs_capw_o       ()
  );

  // ------------------------------------------------------------ lanes
  for (genvar L = 0; L < 4; L++) begin : gen_lane
    // per-lane connections, normally the lane's own slice
    wire        g_rst  = (L == 2) ? rst_lane2 : rst_i_n;
    wire        g_run;
    wire [3:0]  g_in;
    wire        g_pwe;
    wire [15:0] g_ts;
    wire [15:0] g_cfgd;
`ifdef MUT_LANE_RUN_SWAP
    assign g_run = (L == 0) ? run[1] : (L == 1) ? run[0] : run[L];   // planted
`else
    assign g_run = run[L];
`endif
`ifdef MUT_LANE_IN_OFF1
    assign g_in = lane_in[4*((L+1)%4) +: 4];                         // planted
`else
    assign g_in = lane_in[4*L +: 4];
`endif
`ifdef MUT_STROBE_IDX
    assign g_pwe = (L == 3) ? l_park_we[2] : l_park_we[L];           // planted
`else
    assign g_pwe = l_park_we[L];
`endif
`ifdef MUT_TS_HIGH
    assign g_ts = ts[31:16];                                         // planted
`else
    assign g_ts = ts[15:0];
`endif
`ifdef MUT_CFG_DATA_SHIFT
    assign g_cfgd = (L == 1) ? {l_cfg_data[14:0], 1'b0} : l_cfg_data; // planted
`else
    assign g_cfgd = l_cfg_data;
`endif

    rosetta_lane #(
`ifdef MUT_LANE_RTRIP
        .RTRIP (3)                                                   // planted
`else
        .RTRIP (4)
`endif
    ) u_lane (
        .clk         (clk),
        .rst_n       (g_rst),
        .run_i       (g_run),
        .park_we_i   (g_pwe),
        .park_op_i   (l_park_op),
        .park_ctl_i  (l_park_ctl),
        .park_data_i (l_park_data),
        .res_ack_i   (l_res_ack[L]),
        .abort_i     (l_abort[L]),
        .ev_clr_i    (l_ev_clr[4*L +: 4]),
        .cfg_we_i    (l_cfg_we[L]),
        .cfg_reg_i   (l_cfg_reg),
        .cfg_hi_i    (l_cfg_hi),
        .cfg_data_i  (g_cfgd),
        .cmd_i       (l_cmd[2*L +: 2]),
        .cmd_data_i  (l_cmd_data),
        .data_wait_i (l_data_wait[L]),
        .in_i        (g_in),
        .ts_i        (g_ts),
        .busy_o      (l_busy[L]),
        .park_full_o (l_park_full[L]),
        .res_valid_o (l_res_valid[L]),
        .res_dst_o   (l_res_dst[4*L +: 4]),
        .res_data_o  (l_res_data[32*L +: 32]),
        .res_ts_o    (l_res_ts[16*L +: 16]),
        .dst_busy_o  (l_dst_busy[4*L +: 4]),
        .m_o         (l_m[L]),
        .ev_pend_o   (l_ev_pend[4*L +: 4]),
        .sr_o        (l_sr[32*L +: 32]),
        .crc_o       (l_crc[32*L +: 32]),
        .out_val_o   (lane_val[4*L +: 4]),
        .out_oe_o    (lane_oe[4*L +: 4])
    );
  end
endmodule
