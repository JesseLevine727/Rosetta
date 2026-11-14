`default_nettype none
// Synchronisers and pin crossbar (docs/blocks/xbar.md). Per decisions.md
// rulings 2 and 14 both synchroniser flops sit ahead of the crossbar mux:
// pin -> s1 -> s2 -> mux -> lin -> lane, 3 cycles. Host pins: pin -> s1 -> s2.
module rosetta_xbar (
    input  wire         clk, rst_n,
    input  wire  [7:0]  ui_in_i, uio_in_i,
    output logic        host_sclk_o, host_csn_o, host_mosi_o,
    input  wire         host_miso_i, host_irq_i,
    output logic [15:0] lane_in_o,
    input  wire  [15:0] lane_val_i, lane_oe_i,
    input  wire  [3:0]  run_i,
    output logic [7:0]  uo_out_o, uio_out_o, uio_oe_o,
    input  wire         hreg_we_i,
    input  wire  [6:0]  hreg_addr_i,
    input  wire  [31:0] hreg_wdata_i,
    output logic [31:0] hreg_rdata_o,
    output logic [63:0] obs_insel_o,
    output logic [69:0] obs_outsel_o,
    output logic        obs_loop_o
);
  logic [15:0] s1, s2, lin, src, nlin, g, v;
  logic [63:0] insel;              // lane input i = 4L+j: select at [4i +: 4]
  logic [69:0] outsel;             // output p: {en, lane, out} at [5p +: 5]
  logic        loop, hit;
  logic [13:0] val;                // pin value, output index p
  logic [7:0]  oe;                 // uio enable, output index p = k+6

  // Two synchroniser flops per input pin; s2[2:0] is the host second stage.
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin s1 <= 16'h0002; s2 <= 16'h0002; end
    else begin s1 <= {uio_in_i, ui_in_i}; s2 <= s1; end
  assign {host_mosi_o, host_csn_o, host_sclk_o} = s2[2:0];

  // Output pins, combinational. g: lane output enabled and lane running.
  assign g = lane_oe_i & {{4{run_i[3]}}, {4{run_i[2]}}, {4{run_i[1]}}, {4{run_i[0]}}};
  assign v = g & lane_val_i;
  for (genvar p = 0; p < 14; p++) begin : g_val
    assign val[p] = outsel[5*p+4] & v[outsel[5*p +: 4]];
  end
  for (genvar k = 0; k < 8; k++) begin : g_oe
    assign oe[k] = outsel[5*k+34] & g[outsel[5*k+30 +: 4]];
  end
  assign uo_out_o  = {val[5:0], host_irq_i, host_miso_i};
  assign uio_out_o = val[13:6];
  assign uio_oe_o  = oe;

  // Crossbar: lin[i] <= SRC(insel[i]); loopback swaps every source for its
  // output twin (a released uio reads 1).
  assign src = loop ? {~oe | val[13:6], uo_out_o} : s2;
  for (genvar i = 0; i < 16; i++) begin : g_lin
    assign nlin[i] = src[insel[4*i +: 4]];
  end
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) lin <= '0; else lin <= nlin;
  assign lane_in_o = lin;

  // Configuration registers 0x40-0x45; a write replaces the whole register.
  assign hit = hreg_we_i && hreg_addr_i[6:3] == 4'b1000;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      insel <= '0; outsel <= '0; loop <= 1'b0;
    end else if (hit) begin
      if (hreg_addr_i[2:0] == 3'd0) insel[31:0]   <= hreg_wdata_i;
      if (hreg_addr_i[2:0] == 3'd1) insel[63:32]  <= hreg_wdata_i;
      if (hreg_addr_i[2:0] == 3'd2) outsel[29:0]  <= hreg_wdata_i[29:0];
      if (hreg_addr_i[2:0] == 3'd3) outsel[59:30] <= hreg_wdata_i[29:0];
      if (hreg_addr_i[2:0] == 3'd4) outsel[69:60] <= hreg_wdata_i[9:0];
      if (hreg_addr_i[2:0] == 3'd5) loop          <= hreg_wdata_i[0];
    end

  always_comb
    case (hreg_addr_i)
      7'h40:   hreg_rdata_o = insel[31:0];
      7'h41:   hreg_rdata_o = insel[63:32];
      7'h42:   hreg_rdata_o = {2'b0, outsel[29:0]};
      7'h43:   hreg_rdata_o = {2'b0, outsel[59:30]};
      7'h44:   hreg_rdata_o = {22'b0, outsel[69:60]};
      7'h45:   hreg_rdata_o = {31'b0, loop};
      default: hreg_rdata_o = '0;
    endcase

  assign obs_insel_o  = insel;
  assign obs_outsel_o = outsel;
  assign obs_loop_o   = loop;
endmodule
