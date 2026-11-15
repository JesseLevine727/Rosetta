`default_nettype none

// Rosetta top level (docs/blocks/top.md). Contains no logic beyond the
// two-flop reset synchroniser (ruling 15) and the OR of the register read
// buses. Tiny Tapeout port mapping per top.md section 2.
//
// Interconnect per top.md section 4 and rulings 14, 30, 31:
//  - u_chan.ts_o goes to u_seq.ts_i (32 bits) and its low 16 bits to every
//    lane's ts_i; every lane's res_ts_o goes to u_seq.lane_res_ts_i.
//  - Pin synchronisers live inside u_xbar (three flops on the pin path, two
//    on the host path).
//
// This file defines the same module name as src/project.v; a build uses
// one or the other, never both.

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
  localparam int NL = 4;

  // ---------------------------------------------------------------- reset
  // rst_sync[1:0] of top.md 3, kept as two named flops (rst_s0 = rst_sync[0],
  // rst_i_n = rst_sync[1]) so lint does not see one vector used as both
  // data and asynchronous reset.
  logic rst_s0, rst_i_n;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rst_s0  <= 1'b0;
      rst_i_n <= 1'b0;
    end else begin
      rst_s0  <= 1'b1;
      rst_i_n <= rst_s0;
    end
  end

  // ---------------------------------------------------------------- nets
  // wheel
  logic [4:0]   ph;
  // program memory
  logic [8:0]   pm_addr;
  logic [31:0]  pm_rdata;
  logic         h_pm_en, h_pm_we;
  logic [8:0]   h_pm_addr;
  logic [31:0]  h_pm_wdata;
  // data memory
  logic [2:0]   dm_op;
  logic [6:0]   dm_off;
  logic [1:0]   dm_hmask;
  logic [31:0]  dm_wdata, dm_rdata;
  logic         h_dm_en, h_dm_we;
  logic [8:0]   h_dm_addr;
  logic [31:0]  h_dm_wdata;
  logic [NL-1:0] qe, qf, qne;
  // channels and time
  logic         ch_put, ch_get;
  logic [1:0]   ch_idx;
  logic [31:0]  ch_wdata;
  logic [3:0]   ch_valid;
  logic [127:0] ch_data;
  logic [31:0]  ts;
  // register bus
  logic         hreg_we;
  logic [6:0]   hreg_addr;
  logic [31:0]  hreg_wdata;
  logic [31:0]  rd_seq, rd_mem, rd_chan, rd_xbar, hreg_rdata;
  // run / attention
  logic [NL-1:0] run, attn, lane_run;
  // host pins
  logic         host_sclk, host_csn, host_mosi, host_miso, host_irq;
  // lane pins
  logic [4*NL-1:0] lane_in, lane_val, lane_oe;
  // lane control (sequencer -> lanes)
  logic [NL-1:0]   l_park_we, l_res_ack, l_abort, l_cfg_we, l_data_wait;
  logic [1:0]      l_park_op;
  logic [27:0]     l_park_ctl;
  logic [31:0]     l_park_data;
  logic [4*NL-1:0] l_ev_clr;
  logic [4:0]      l_cfg_reg;
  logic            l_cfg_hi;
  logic [15:0]     l_cfg_data;
  logic [2*NL-1:0] l_cmd;
  logic [31:0]     l_cmd_data;
  // lanes -> sequencer
  logic [NL-1:0]    l_busy, l_park_full, l_res_valid, l_m;
  logic [4*NL-1:0]  l_res_dst, l_dst_busy, l_ev_pend;
  logic [32*NL-1:0] l_res_data, l_sr, l_crc;
  logic [16*NL-1:0] l_res_ts;

  // observation outputs (not used at the top)
  logic [9*NL-1:0]  obs_pc;
  logic [64*NL-1:0] obs_r;
  logic [NL-1:0]    obs_z, obs_n, obs_pause, obs_step, obs_wsta, obs_capw;
  logic [4*NL-1:0]  obs_armed;
  logic [36*NL-1:0] obs_tgt;
  logic [15:0]      obs_qcfg;
  logic [27:0]      obs_in_wr, obs_in_rd, obs_out_wr, obs_out_rd;
  logic [31:0]      obs_sr;
  logic [7:0]       obs_cmd, obs_imask;
  logic [5:0]       obs_bitcnt;
  logic [9:0]       obs_maddr;
  logic [1:0]       obs_hst;
  logic [63:0]      obs_insel;
  logic [69:0]      obs_outsel;
  logic             obs_loop;

  // ---------------------------------------------------------------- xbar
  rosetta_xbar u_xbar (
      .clk          (clk),
      .rst_n        (rst_i_n),
      .ui_in_i      (ui_in),
      .uio_in_i     (uio_in),
      .host_sclk_o  (host_sclk),
      .host_csn_o   (host_csn),
      .host_mosi_o  (host_mosi),
      .host_miso_i  (host_miso),
      .host_irq_i   (host_irq),
      .lane_in_o    (lane_in),
      .lane_val_i   (lane_val),
      .lane_oe_i    (lane_oe),
      .run_i        (run),
      .uo_out_o     (uo_out),
      .uio_out_o    (uio_out),
      .uio_oe_o     (uio_oe),
      .hreg_we_i    (hreg_we),
      .hreg_addr_i  (hreg_addr),
      .hreg_wdata_i (hreg_wdata),
      .hreg_rdata_o (rd_xbar),
      .obs_insel_o  (obs_insel),
      .obs_outsel_o (obs_outsel),
      .obs_loop_o   (obs_loop)
  );

  // ---------------------------------------------------------------- host
  rosetta_host u_host (
      .clk          (clk),
      .rst_n        (rst_i_n),
      .ph_i         (ph),
      .sclk_i       (host_sclk),
      .csn_i        (host_csn),
      .mosi_i       (host_mosi),
      .miso_o       (host_miso),
      .irq_o        (host_irq),
      .h_pm_en_o    (h_pm_en),
      .h_pm_we_o    (h_pm_we),
      .h_pm_addr_o  (h_pm_addr),
      .h_pm_wdata_o (h_pm_wdata),
      .pm_rdata_i   (pm_rdata),
      .h_dm_en_o    (h_dm_en),
      .h_dm_we_o    (h_dm_we),
      .h_dm_addr_o  (h_dm_addr),
      .h_dm_wdata_o (h_dm_wdata),
      .dm_rdata_i   (dm_rdata),
      .hreg_we_o    (hreg_we),
      .hreg_addr_o  (hreg_addr),
      .hreg_wdata_o (hreg_wdata),
      .hreg_rdata_i (hreg_rdata),
      .attn_i       (attn),
      .qne_i        (qne),
      .obs_sr_o     (obs_sr),
      .obs_cmd_o    (obs_cmd),
      .obs_bitcnt_o (obs_bitcnt),
      .obs_maddr_o  (obs_maddr),
      .obs_hst_o    (obs_hst),
      .obs_imask_o  (obs_imask)
  );

  // Address ownership is disjoint (host.md 4), so the OR is exact.
  assign hreg_rdata = rd_seq | rd_mem | rd_chan | rd_xbar;

  // ---------------------------------------------------------------- sequencer
  rosetta_seq #(.N_LANES(NL)) u_seq (
      .clk              (clk),
      .rst_n            (rst_i_n),
      .ph_o             (ph),
      .pm_addr_o        (pm_addr),
      .pm_rdata_i       (pm_rdata),
      .dm_op_o          (dm_op),
      .dm_off_o         (dm_off),
      .dm_hmask_o       (dm_hmask),
      .dm_wdata_o       (dm_wdata),
      .dm_rdata_i       (dm_rdata),
      .qe_i             (qe),
      .qf_i             (qf),
      .ch_put_o         (ch_put),
      .ch_get_o         (ch_get),
      .ch_idx_o         (ch_idx),
      .ch_wdata_o       (ch_wdata),
      .ch_valid_i       (ch_valid),
      .ch_data_i        (ch_data),
      .ts_i             (ts),
      .hreg_we_i        (hreg_we),
      .hreg_addr_i      (hreg_addr),
      .hreg_wdata_i     (hreg_wdata),
      .hreg_rdata_o     (rd_seq),
      .run_o            (run),
      .attn_o           (attn),
      .lane_run_o       (lane_run),
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
      .lane_res_data_i  (l_res_data),
      .lane_res_ts_i    (l_res_ts),
      .lane_dst_busy_i  (l_dst_busy),
      .lane_m_i         (l_m),
      .lane_ev_pend_i   (l_ev_pend),
      .lane_sr_i        (l_sr),
      .lane_crc_i       (l_crc),
      .obs_pc_o         (obs_pc),
      .obs_r_o          (obs_r),
      .obs_z_o          (obs_z),
      .obs_n_o          (obs_n),
      .obs_pause_o      (obs_pause),
      .obs_step_o       (obs_step),
      .obs_armed_o      (obs_armed),
      .obs_tgt_o        (obs_tgt),
      .obs_wsta_o       (obs_wsta),
      .obs_capw_o       (obs_capw)
  );

  // ---------------------------------------------------------------- memories
  rosetta_mem_prog u_mem_prog (
      .clk          (clk),
      .ph_i         (ph),
      .fetch_addr_i (pm_addr),
      .h_en_i       (h_pm_en),
      .h_we_i       (h_pm_we),
      .h_addr_i     (h_pm_addr),
      .h_wdata_i    (h_pm_wdata),
      .rdata_o      (pm_rdata)
  );

  rosetta_mem_data u_mem_data (
      .clk          (clk),
      .rst_n        (rst_i_n),
      .ph_i         (ph),
      .dm_op_i      (dm_op),
      .dm_off_i     (dm_off),
      .dm_hmask_i   (dm_hmask),
      .dm_wdata_i   (dm_wdata),
      .h_en_i       (h_dm_en),
      .h_we_i       (h_dm_we),
      .h_addr_i     (h_dm_addr),
      .h_wdata_i    (h_dm_wdata),
      .run_i        (run),
      .hreg_we_i    (hreg_we),
      .hreg_addr_i  (hreg_addr),
      .hreg_wdata_i (hreg_wdata),
      .hreg_rdata_o (rd_mem),
      .rdata_o      (dm_rdata),
      .qe_o         (qe),
      .qf_o         (qf),
      .qne_o        (qne),
      .obs_qcfg_o   (obs_qcfg),
      .obs_in_wr_o  (obs_in_wr),
      .obs_in_rd_o  (obs_in_rd),
      .obs_out_wr_o (obs_out_wr),
      .obs_out_rd_o (obs_out_rd)
  );

  // ---------------------------------------------------------------- channels
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
      .hreg_we_i    (hreg_we),
      .hreg_addr_i  (hreg_addr),
      .hreg_wdata_i (hreg_wdata),
      .hreg_rdata_o (rd_chan)
  );

  // ---------------------------------------------------------------- lanes
  for (genvar L = 0; L < NL; L++) begin : gen_lane
    rosetta_lane u_lane (
        .clk         (clk),
        .rst_n       (rst_i_n),
        .run_i       (lane_run[L]),
        .park_we_i   (l_park_we[L]),
        .park_op_i   (l_park_op),
        .park_ctl_i  (l_park_ctl),
        .park_data_i (l_park_data),
        .res_ack_i   (l_res_ack[L]),
        .abort_i     (l_abort[L]),
        .ev_clr_i    (l_ev_clr[4*L +: 4]),
        .cfg_we_i    (l_cfg_we[L]),
        .cfg_reg_i   (l_cfg_reg),
        .cfg_hi_i    (l_cfg_hi),
        .cfg_data_i  (l_cfg_data),
        .cmd_i       (l_cmd[2*L +: 2]),
        .cmd_data_i  (l_cmd_data),
        .data_wait_i (l_data_wait[L]),
        .in_i        (lane_in[4*L +: 4]),
        .ts_i        (ts[15:0]),           // chan ts_o is 32 bits; lane takes the low 16
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

  // ---------------------------------------------------------------- unused
  wire _unused = &{1'b0, ena, obs_pc, obs_r, obs_z, obs_n,
                   obs_pause, obs_step, obs_armed, obs_tgt, obs_wsta, obs_capw,
                   obs_qcfg, obs_in_wr, obs_in_rd, obs_out_wr, obs_out_rd,
                   obs_sr, obs_cmd, obs_imask, obs_bitcnt, obs_maddr, obs_hst,
                   obs_insel, obs_outsel, obs_loop};
endmodule
