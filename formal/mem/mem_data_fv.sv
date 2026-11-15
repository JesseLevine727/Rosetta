`default_nettype none
// ===========================================================================
//  Proof top for rosetta_mem_data.
//
//  One copy of the block, every input free; rst_n low in cycle 0 (assumed in
//  the checker), free afterwards; flops start at arbitrary values; ph_i
//  one-hot by assumption.
//
//  `MEM_PROBE (task prove): the macro is the probe model mem_probe.sv
//  (src/rosetta_sram512x32.sv read WITHOUT ROSETTA_SRAM_BEHAV instantiates
//  the macro by name); the checker decodes the pins from rdata_o.
//  Otherwise (tasks bmc, cover): src/rosetta_sram512x32.sv with
//  ROSETTA_SRAM_BEHAV, the full behavioural memory (arbitrary initial array
//  and read register).
// ===========================================================================

module mem_data_fv (
    input wire         clk,
    input wire         rst_n,
    input wire [4:0]   ph_i,
    input wire [2:0]   dm_op_i,
    input wire [6:0]   dm_off_i,
    input wire [1:0]   dm_hmask_i,
    input wire [31:0]  dm_wdata_i,
    input wire         h_en_i,
    input wire         h_we_i,
    input wire [8:0]   h_addr_i,
    input wire [31:0]  h_wdata_i,
    input wire [3:0]   run_i,
    input wire         hreg_we_i,
    input wire [6:0]   hreg_addr_i,
    input wire [31:0]  hreg_wdata_i
);
`ifdef MEM_PROBE
    localparam PROBE = 1;
`else
    localparam PROBE = 0;
`endif
    // Marked spec choices (see mem_data_props.sv header and the .sby):
    //   MEM_HREG_UNGATED  assume hreg_we_i only in ph = 1 instead of checking
    //                     that it is ignored elsewhere (HREG_PH_GATED = 0)
    //   MEM_LAX_ZERO      the 0 entries of the drive table are don't-care
`ifdef MEM_HREG_UNGATED
    localparam HREG_PH_GATED = 0;
`else
    localparam HREG_PH_GATED = 1;
`endif
`ifdef MEM_LAX_ZERO
    localparam STRICT_ZERO = 0;
`else
    localparam STRICT_ZERO = 1;
`endif

    wire [31:0] hreg_rdata, rdata;
    wire [3:0]  qe, qf, qne;
    wire [15:0] obs_qcfg;
    wire [27:0] obs_in_wr, obs_in_rd, obs_out_wr, obs_out_rd;

    rosetta_mem_data u_dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .ph_i         (ph_i),
        .dm_op_i      (dm_op_i),
        .dm_off_i     (dm_off_i),
        .dm_hmask_i   (dm_hmask_i),
        .dm_wdata_i   (dm_wdata_i),
        .h_en_i       (h_en_i),
        .h_we_i       (h_we_i),
        .h_addr_i     (h_addr_i),
        .h_wdata_i    (h_wdata_i),
        .run_i        (run_i),
        .hreg_we_i    (hreg_we_i),
        .hreg_addr_i  (hreg_addr_i),
        .hreg_wdata_i (hreg_wdata_i),
        .hreg_rdata_o (hreg_rdata),
        .rdata_o      (rdata),
        .qe_o         (qe),
        .qf_o         (qf),
        .qne_o        (qne),
        .obs_qcfg_o   (obs_qcfg),
        .obs_in_wr_o  (obs_in_wr),
        .obs_in_rd_o  (obs_in_rd),
        .obs_out_wr_o (obs_out_wr),
        .obs_out_rd_o (obs_out_rd)
    );

    mem_data_props #(.PROBE(PROBE), .STRICT_ZERO(STRICT_ZERO), .HREG_PH_GATED(HREG_PH_GATED)) u_props (
        .clk          (clk),
        .rst_n        (rst_n),
        .ph_i         (ph_i),
        .dm_op_i      (dm_op_i),
        .dm_off_i     (dm_off_i),
        .dm_hmask_i   (dm_hmask_i),
        .dm_wdata_i   (dm_wdata_i),
        .h_en_i       (h_en_i),
        .h_we_i       (h_we_i),
        .h_addr_i     (h_addr_i),
        .h_wdata_i    (h_wdata_i),
        .run_i        (run_i),
        .hreg_we_i    (hreg_we_i),
        .hreg_addr_i  (hreg_addr_i),
        .hreg_wdata_i (hreg_wdata_i),
        .hreg_rdata_o (hreg_rdata),
        .rdata_o      (rdata),
        .qe_o         (qe),
        .qf_o         (qf),
        .qne_o        (qne),
        .obs_qcfg_o   (obs_qcfg),
        .obs_in_wr_o  (obs_in_wr),
        .obs_in_rd_o  (obs_in_rd),
        .obs_out_wr_o (obs_out_wr),
        .obs_out_rd_o (obs_out_rd)
    );
endmodule
