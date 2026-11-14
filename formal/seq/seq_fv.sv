`default_nettype none
// ===========================================================================
//  Proof top for rosetta_seq: the block and the checker, every input free.
//  Environment rules are assumed inside seq_props (STANDALONE = 1). The
//  design file is chosen in seq.sby (design or the private reference model).
// ===========================================================================

module seq_fv (
    input wire clk,
    input wire rst_n,
    input wire [31:0] pm_rdata_i,
    input wire [31:0] dm_rdata_i,
    input wire [3:0] qe_i,
    input wire [3:0] qf_i,
    input wire [3:0] ch_valid_i,
    input wire [127:0] ch_data_i,
    input wire [31:0] ts_i,
    input wire hreg_we_i,
    input wire [6:0] hreg_addr_i,
    input wire [31:0] hreg_wdata_i,
    input wire [3:0] lane_busy_i,
    input wire [3:0] lane_park_full_i,
    input wire [3:0] lane_res_valid_i,
    input wire [15:0] lane_res_dst_i,
    input wire [127:0] lane_res_data_i,
    input wire [63:0] lane_res_ts_i,
    input wire [15:0] lane_dst_busy_i,
    input wire [3:0] lane_m_i,
    input wire [15:0] lane_ev_pend_i,
    input wire [127:0] lane_sr_i,
    input wire [127:0] lane_crc_i
);
    wire [4:0] ph_o;
    wire [8:0] pm_addr_o;
    wire [2:0] dm_op_o;
    wire [6:0] dm_off_o;
    wire [1:0] dm_hmask_o;
    wire [31:0] dm_wdata_o;
    wire ch_put_o;
    wire ch_get_o;
    wire [1:0] ch_idx_o;
    wire [31:0] ch_wdata_o;
    wire [31:0] hreg_rdata_o;
    wire [3:0] run_o;
    wire [3:0] attn_o;
    wire [3:0] lane_run_o;
    wire [3:0] lane_park_we_o;
    wire [1:0] lane_park_op_o;
    wire [27:0] lane_park_ctl_o;
    wire [31:0] lane_park_data_o;
    wire [3:0] lane_res_ack_o;
    wire [3:0] lane_abort_o;
    wire [15:0] lane_ev_clr_o;
    wire [3:0] lane_cfg_we_o;
    wire [4:0] lane_cfg_reg_o;
    wire lane_cfg_hi_o;
    wire [15:0] lane_cfg_data_o;
    wire [7:0] lane_cmd_o;
    wire [31:0] lane_cmd_data_o;
    wire [3:0] lane_data_wait_o;
    wire [35:0] obs_pc_o;
    wire [255:0] obs_r_o;
    wire [3:0] obs_z_o;
    wire [3:0] obs_n_o;
    wire [3:0] obs_pause_o;
    wire [3:0] obs_step_o;
    wire [15:0] obs_armed_o;
    wire [143:0] obs_tgt_o;
    wire [3:0] obs_wsta_o;
    wire [3:0] obs_capw_o;

    rosetta_seq u_dut (
        .clk(clk),
        .rst_n(rst_n),
        .pm_rdata_i(pm_rdata_i),
        .dm_rdata_i(dm_rdata_i),
        .qe_i(qe_i),
        .qf_i(qf_i),
        .ch_valid_i(ch_valid_i),
        .ch_data_i(ch_data_i),
        .ts_i(ts_i),
        .hreg_we_i(hreg_we_i),
        .hreg_addr_i(hreg_addr_i),
        .hreg_wdata_i(hreg_wdata_i),
        .lane_busy_i(lane_busy_i),
        .lane_park_full_i(lane_park_full_i),
        .lane_res_valid_i(lane_res_valid_i),
        .lane_res_dst_i(lane_res_dst_i),
        .lane_res_data_i(lane_res_data_i),
        .lane_res_ts_i(lane_res_ts_i),
        .lane_dst_busy_i(lane_dst_busy_i),
        .lane_m_i(lane_m_i),
        .lane_ev_pend_i(lane_ev_pend_i),
        .lane_sr_i(lane_sr_i),
        .lane_crc_i(lane_crc_i),
        .ph_o(ph_o),
        .pm_addr_o(pm_addr_o),
        .dm_op_o(dm_op_o),
        .dm_off_o(dm_off_o),
        .dm_hmask_o(dm_hmask_o),
        .dm_wdata_o(dm_wdata_o),
        .ch_put_o(ch_put_o),
        .ch_get_o(ch_get_o),
        .ch_idx_o(ch_idx_o),
        .ch_wdata_o(ch_wdata_o),
        .hreg_rdata_o(hreg_rdata_o),
        .run_o(run_o),
        .attn_o(attn_o),
        .lane_run_o(lane_run_o),
        .lane_park_we_o(lane_park_we_o),
        .lane_park_op_o(lane_park_op_o),
        .lane_park_ctl_o(lane_park_ctl_o),
        .lane_park_data_o(lane_park_data_o),
        .lane_res_ack_o(lane_res_ack_o),
        .lane_abort_o(lane_abort_o),
        .lane_ev_clr_o(lane_ev_clr_o),
        .lane_cfg_we_o(lane_cfg_we_o),
        .lane_cfg_reg_o(lane_cfg_reg_o),
        .lane_cfg_hi_o(lane_cfg_hi_o),
        .lane_cfg_data_o(lane_cfg_data_o),
        .lane_cmd_o(lane_cmd_o),
        .lane_cmd_data_o(lane_cmd_data_o),
        .lane_data_wait_o(lane_data_wait_o),
        .obs_pc_o(obs_pc_o),
        .obs_r_o(obs_r_o),
        .obs_z_o(obs_z_o),
        .obs_n_o(obs_n_o),
        .obs_pause_o(obs_pause_o),
        .obs_step_o(obs_step_o),
        .obs_armed_o(obs_armed_o),
        .obs_tgt_o(obs_tgt_o),
        .obs_wsta_o(obs_wsta_o),
        .obs_capw_o(obs_capw_o)
    );

    seq_props #(.STANDALONE(1)) u_props (
        .clk(clk),
        .rst_n(rst_n),
        .pm_rdata_i(pm_rdata_i),
        .dm_rdata_i(dm_rdata_i),
        .qe_i(qe_i),
        .qf_i(qf_i),
        .ch_valid_i(ch_valid_i),
        .ch_data_i(ch_data_i),
        .ts_i(ts_i),
        .hreg_we_i(hreg_we_i),
        .hreg_addr_i(hreg_addr_i),
        .hreg_wdata_i(hreg_wdata_i),
        .lane_busy_i(lane_busy_i),
        .lane_park_full_i(lane_park_full_i),
        .lane_res_valid_i(lane_res_valid_i),
        .lane_res_dst_i(lane_res_dst_i),
        .lane_res_data_i(lane_res_data_i),
        .lane_res_ts_i(lane_res_ts_i),
        .lane_dst_busy_i(lane_dst_busy_i),
        .lane_m_i(lane_m_i),
        .lane_ev_pend_i(lane_ev_pend_i),
        .lane_sr_i(lane_sr_i),
        .lane_crc_i(lane_crc_i),
        .ph_o(ph_o),
        .pm_addr_o(pm_addr_o),
        .dm_op_o(dm_op_o),
        .dm_off_o(dm_off_o),
        .dm_hmask_o(dm_hmask_o),
        .dm_wdata_o(dm_wdata_o),
        .ch_put_o(ch_put_o),
        .ch_get_o(ch_get_o),
        .ch_idx_o(ch_idx_o),
        .ch_wdata_o(ch_wdata_o),
        .hreg_rdata_o(hreg_rdata_o),
        .run_o(run_o),
        .attn_o(attn_o),
        .lane_run_o(lane_run_o),
        .lane_park_we_o(lane_park_we_o),
        .lane_park_op_o(lane_park_op_o),
        .lane_park_ctl_o(lane_park_ctl_o),
        .lane_park_data_o(lane_park_data_o),
        .lane_res_ack_o(lane_res_ack_o),
        .lane_abort_o(lane_abort_o),
        .lane_ev_clr_o(lane_ev_clr_o),
        .lane_cfg_we_o(lane_cfg_we_o),
        .lane_cfg_reg_o(lane_cfg_reg_o),
        .lane_cfg_hi_o(lane_cfg_hi_o),
        .lane_cfg_data_o(lane_cfg_data_o),
        .lane_cmd_o(lane_cmd_o),
        .lane_cmd_data_o(lane_cmd_data_o),
        .lane_data_wait_o(lane_data_wait_o),
        .obs_pc_o(obs_pc_o),
        .obs_r_o(obs_r_o),
        .obs_z_o(obs_z_o),
        .obs_n_o(obs_n_o),
        .obs_pause_o(obs_pause_o),
        .obs_step_o(obs_step_o),
        .obs_armed_o(obs_armed_o),
        .obs_tgt_o(obs_tgt_o),
        .obs_wsta_o(obs_wsta_o),
        .obs_capw_o(obs_capw_o)
    );
endmodule
