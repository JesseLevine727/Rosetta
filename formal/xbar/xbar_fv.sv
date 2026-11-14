`default_nettype none
// ===========================================================================
//  Proof top for rosetta_xbar.
//
//  One rosetta_xbar with every input free, checked by xbar_props (exact
//  behaviour of every output port in every cycle, the required properties of
//  xbar.md section 7 stated on the ports, covers). rst_n is low in the first
//  cycle (assumed in xbar_props) and free after. Flops start at arbitrary
//  values (no initial values anywhere).
//
//  Spec: docs/blocks/xbar.md as amended by docs/decisions.md ruling 14.
// ===========================================================================

module xbar_fv (
    input wire        clk,
    input wire        rst_n,
    input wire [7:0]  ui_in_i,
    input wire [7:0]  uio_in_i,
    input wire        host_miso_i,
    input wire        host_irq_i,
    input wire [15:0] lane_val_i,
    input wire [15:0] lane_oe_i,
    input wire [3:0]  run_i,
    input wire        hreg_we_i,
    input wire [6:0]  hreg_addr_i,
    input wire [31:0] hreg_wdata_i
);
    wire        host_sclk_o, host_csn_o, host_mosi_o;
    wire [15:0] lane_in_o;
    wire [7:0]  uo_out_o, uio_out_o, uio_oe_o;
    wire [31:0] hreg_rdata_o;
    wire [63:0] obs_insel_o;
    wire [69:0] obs_outsel_o;
    wire        obs_loop_o;

    rosetta_xbar u_dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .ui_in_i      (ui_in_i),
        .uio_in_i     (uio_in_i),
        .host_sclk_o  (host_sclk_o),
        .host_csn_o   (host_csn_o),
        .host_mosi_o  (host_mosi_o),
        .host_miso_i  (host_miso_i),
        .host_irq_i   (host_irq_i),
        .lane_in_o    (lane_in_o),
        .lane_val_i   (lane_val_i),
        .lane_oe_i    (lane_oe_i),
        .run_i        (run_i),
        .uo_out_o     (uo_out_o),
        .uio_out_o    (uio_out_o),
        .uio_oe_o     (uio_oe_o),
        .hreg_we_i    (hreg_we_i),
        .hreg_addr_i  (hreg_addr_i),
        .hreg_wdata_i (hreg_wdata_i),
        .hreg_rdata_o (hreg_rdata_o),
        .obs_insel_o  (obs_insel_o),
        .obs_outsel_o (obs_outsel_o),
        .obs_loop_o   (obs_loop_o)
    );

    // OBS_CHECK(1): the obs_* ports are checked with the layout in
    // xbar_props.sv. Use 0 if the design documents another layout.
    xbar_props #(.STANDALONE(1), .OBS_CHECK(1)) u_props (
        .clk          (clk),
        .rst_n        (rst_n),
        .ui_in_i      (ui_in_i),
        .uio_in_i     (uio_in_i),
        .host_sclk_o  (host_sclk_o),
        .host_csn_o   (host_csn_o),
        .host_mosi_o  (host_mosi_o),
        .host_miso_i  (host_miso_i),
        .host_irq_i   (host_irq_i),
        .lane_in_o    (lane_in_o),
        .lane_val_i   (lane_val_i),
        .lane_oe_i    (lane_oe_i),
        .run_i        (run_i),
        .uo_out_o     (uo_out_o),
        .uio_out_o    (uio_out_o),
        .uio_oe_o     (uio_oe_o),
        .hreg_we_i    (hreg_we_i),
        .hreg_addr_i  (hreg_addr_i),
        .hreg_wdata_i (hreg_wdata_i),
        .hreg_rdata_o (hreg_rdata_o),
        .obs_insel_o  (obs_insel_o),
        .obs_outsel_o (obs_outsel_o),
        .obs_loop_o   (obs_loop_o)
    );
endmodule
