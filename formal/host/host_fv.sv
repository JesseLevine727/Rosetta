`default_nettype none
// ===========================================================================
//  Proof top for rosetta_host.
//
//  Two independent copies of the block (three in the cover task) share clk;
//  every input is free and
//  each copy has its own reset (low in cycle 0, free afterwards; flops start
//  at arbitrary values).
//
//   u_env   ENV_SPI = 1: the SPI pins are driven by the protocol-legal master
//           of host_props (host.md 2.1 at the synchronised pins, arbitrary
//           phase against the wheel, free command/data bits, aborts and
//           extra clocks allowed). Model check, port properties, end-to-end
//           frame properties (required property 10), covers of every timing
//           rule at its boundary and of a command at the fastest legal SCLK.
//   u_free  ENV_SPI = 0: the SPI pins are completely free (no timing rule).
//           Model check and port properties only: the cycle rules of
//           section 3 hold for any input, so a design that is only right
//           for nicely timed frames fails here. Short-trace covers (every
//           request kind, IRQ sources) live here.
//   u_c     cover task only (FV_COVER): the u_env master with the time scale
//           compressed (HALF = 1, CS gaps 2, MOSI setup/hold 0) and every
//           assertion off, so whole frames, back-to-back frames, aborts and
//           IRQ raise/clear fit a cover trace (a legal frame at real timing
//           is >= 570 cycles; u_env covers each timing rule at its boundary
//           and a command byte + service at the fastest legal SCLK). CS gap
//           2 keeps the scoreboard exact: a request is served (<= 5) and its
//           read data captured (<= 6) before the next frame's first edge
//           (>= HALF + 3 x CS gap = 7 cycles later), as at real timing.
//  All: ph_i one-hot and rotating (sequencer.md property 1); attn_i, qne_i,
//  pm_rdata_i, dm_rdata_i, hreg_rdata_i free in every cycle.
// ===========================================================================

module host_fv (
    input wire         clk,
    // u_env
    input wire         e_rst_n,
    input wire [4:0]   e_ph,
    input wire         e_sclk, e_csn, e_mosi,
    input wire [31:0]  e_pm_rdata, e_dm_rdata, e_hreg_rdata,
    input wire [3:0]   e_attn, e_qne,
    // u_free
    input wire         f_rst_n,
    input wire [4:0]   f_ph,
    input wire         f_sclk, f_csn, f_mosi,
    input wire [31:0]  f_pm_rdata, f_dm_rdata, f_hreg_rdata,
    input wire [3:0]   f_attn, f_qne,
    // u_c (cover task only)
    input wire         c_rst_n,
    input wire [4:0]   c_ph,
    input wire         c_sclk, c_csn, c_mosi,
    input wire [31:0]  c_pm_rdata, c_dm_rdata, c_hreg_rdata,
    input wire [3:0]   c_attn, c_qne
);
`ifndef FV_NO_ENV
    // ---------------------------------------------------------------- u_e
    wire        e_miso, e_irq, e_pm_en, e_pm_we, e_dm_en, e_dm_we, e_hreg_we;
    wire [8:0]  e_pm_addr, e_dm_addr;
    wire [31:0] e_pm_wdata, e_dm_wdata, e_hreg_wdata, e_obs_sr;
    wire [6:0]  e_hreg_addr;
    wire [7:0]  e_obs_cmd, e_obs_imask;
    wire [5:0]  e_obs_bitcnt;
    wire [9:0]  e_obs_maddr;
    wire [1:0]  e_obs_hst;
    rosetta_host u_e (
        .clk(clk), .rst_n(e_rst_n), .ph_i(e_ph),
        .sclk_i(e_sclk), .csn_i(e_csn), .mosi_i(e_mosi),
        .miso_o(e_miso), .irq_o(e_irq),
        .h_pm_en_o(e_pm_en), .h_pm_we_o(e_pm_we), .h_pm_addr_o(e_pm_addr),
        .h_pm_wdata_o(e_pm_wdata), .pm_rdata_i(e_pm_rdata),
        .h_dm_en_o(e_dm_en), .h_dm_we_o(e_dm_we), .h_dm_addr_o(e_dm_addr),
        .h_dm_wdata_o(e_dm_wdata), .dm_rdata_i(e_dm_rdata),
        .hreg_we_o(e_hreg_we), .hreg_addr_o(e_hreg_addr), .hreg_wdata_o(e_hreg_wdata),
        .hreg_rdata_i(e_hreg_rdata), .attn_i(e_attn), .qne_i(e_qne),
        .obs_sr_o(e_obs_sr), .obs_cmd_o(e_obs_cmd), .obs_bitcnt_o(e_obs_bitcnt),
        .obs_maddr_o(e_obs_maddr), .obs_hst_o(e_obs_hst), .obs_imask_o(e_obs_imask)
    );

    host_props #(.STANDALONE(1), .ENV_SPI(1)) u_e_props (
        .clk(clk), .rst_n(e_rst_n), .ph_i(e_ph),
        .sclk_i(e_sclk), .csn_i(e_csn), .mosi_i(e_mosi),
        .miso_o(e_miso), .irq_o(e_irq),
        .h_pm_en_o(e_pm_en), .h_pm_we_o(e_pm_we), .h_pm_addr_o(e_pm_addr),
        .h_pm_wdata_o(e_pm_wdata), .pm_rdata_i(e_pm_rdata),
        .h_dm_en_o(e_dm_en), .h_dm_we_o(e_dm_we), .h_dm_addr_o(e_dm_addr),
        .h_dm_wdata_o(e_dm_wdata), .dm_rdata_i(e_dm_rdata),
        .hreg_we_o(e_hreg_we), .hreg_addr_o(e_hreg_addr), .hreg_wdata_o(e_hreg_wdata),
        .hreg_rdata_i(e_hreg_rdata), .attn_i(e_attn), .qne_i(e_qne),
        .obs_sr_o(e_obs_sr), .obs_cmd_o(e_obs_cmd), .obs_bitcnt_o(e_obs_bitcnt),
        .obs_maddr_o(e_obs_maddr), .obs_hst_o(e_obs_hst), .obs_imask_o(e_obs_imask)
    );
`endif

`ifdef FV_COVER
    // ---------------------------------------------------------------- u_c
    wire        c_miso, c_irq, c_pm_en, c_pm_we, c_dm_en, c_dm_we, c_hreg_we;
    wire [8:0]  c_pm_addr, c_dm_addr;
    wire [31:0] c_pm_wdata, c_dm_wdata, c_hreg_wdata, c_obs_sr;
    wire [6:0]  c_hreg_addr;
    wire [7:0]  c_obs_cmd, c_obs_imask;
    wire [5:0]  c_obs_bitcnt;
    wire [9:0]  c_obs_maddr;
    wire [1:0]  c_obs_hst;
    rosetta_host u_c (
        .clk(clk), .rst_n(c_rst_n), .ph_i(c_ph),
        .sclk_i(c_sclk), .csn_i(c_csn), .mosi_i(c_mosi),
        .miso_o(c_miso), .irq_o(c_irq),
        .h_pm_en_o(c_pm_en), .h_pm_we_o(c_pm_we), .h_pm_addr_o(c_pm_addr),
        .h_pm_wdata_o(c_pm_wdata), .pm_rdata_i(c_pm_rdata),
        .h_dm_en_o(c_dm_en), .h_dm_we_o(c_dm_we), .h_dm_addr_o(c_dm_addr),
        .h_dm_wdata_o(c_dm_wdata), .dm_rdata_i(c_dm_rdata),
        .hreg_we_o(c_hreg_we), .hreg_addr_o(c_hreg_addr), .hreg_wdata_o(c_hreg_wdata),
        .hreg_rdata_i(c_hreg_rdata), .attn_i(c_attn), .qne_i(c_qne),
        .obs_sr_o(c_obs_sr), .obs_cmd_o(c_obs_cmd), .obs_bitcnt_o(c_obs_bitcnt),
        .obs_maddr_o(c_obs_maddr), .obs_hst_o(c_obs_hst), .obs_imask_o(c_obs_imask)
    );

    host_props #(.STANDALONE(1), .ENV_SPI(1), .COVER_ONLY(1),
                 .HALF(1), .CS_GAP(2), .MOSI_SU(0), .MOSI_HD(0)) u_c_props (
        .clk(clk), .rst_n(c_rst_n), .ph_i(c_ph),
        .sclk_i(c_sclk), .csn_i(c_csn), .mosi_i(c_mosi),
        .miso_o(c_miso), .irq_o(c_irq),
        .h_pm_en_o(c_pm_en), .h_pm_we_o(c_pm_we), .h_pm_addr_o(c_pm_addr),
        .h_pm_wdata_o(c_pm_wdata), .pm_rdata_i(c_pm_rdata),
        .h_dm_en_o(c_dm_en), .h_dm_we_o(c_dm_we), .h_dm_addr_o(c_dm_addr),
        .h_dm_wdata_o(c_dm_wdata), .dm_rdata_i(c_dm_rdata),
        .hreg_we_o(c_hreg_we), .hreg_addr_o(c_hreg_addr), .hreg_wdata_o(c_hreg_wdata),
        .hreg_rdata_i(c_hreg_rdata), .attn_i(c_attn), .qne_i(c_qne),
        .obs_sr_o(c_obs_sr), .obs_cmd_o(c_obs_cmd), .obs_bitcnt_o(c_obs_bitcnt),
        .obs_maddr_o(c_obs_maddr), .obs_hst_o(c_obs_hst), .obs_imask_o(c_obs_imask)
    );
`endif

`ifndef FV_NO_FREE
    // ---------------------------------------------------------------- u_f
    wire        f_miso, f_irq, f_pm_en, f_pm_we, f_dm_en, f_dm_we, f_hreg_we;
    wire [8:0]  f_pm_addr, f_dm_addr;
    wire [31:0] f_pm_wdata, f_dm_wdata, f_hreg_wdata, f_obs_sr;
    wire [6:0]  f_hreg_addr;
    wire [7:0]  f_obs_cmd, f_obs_imask;
    wire [5:0]  f_obs_bitcnt;
    wire [9:0]  f_obs_maddr;
    wire [1:0]  f_obs_hst;
    rosetta_host u_f (
        .clk(clk), .rst_n(f_rst_n), .ph_i(f_ph),
        .sclk_i(f_sclk), .csn_i(f_csn), .mosi_i(f_mosi),
        .miso_o(f_miso), .irq_o(f_irq),
        .h_pm_en_o(f_pm_en), .h_pm_we_o(f_pm_we), .h_pm_addr_o(f_pm_addr),
        .h_pm_wdata_o(f_pm_wdata), .pm_rdata_i(f_pm_rdata),
        .h_dm_en_o(f_dm_en), .h_dm_we_o(f_dm_we), .h_dm_addr_o(f_dm_addr),
        .h_dm_wdata_o(f_dm_wdata), .dm_rdata_i(f_dm_rdata),
        .hreg_we_o(f_hreg_we), .hreg_addr_o(f_hreg_addr), .hreg_wdata_o(f_hreg_wdata),
        .hreg_rdata_i(f_hreg_rdata), .attn_i(f_attn), .qne_i(f_qne),
        .obs_sr_o(f_obs_sr), .obs_cmd_o(f_obs_cmd), .obs_bitcnt_o(f_obs_bitcnt),
        .obs_maddr_o(f_obs_maddr), .obs_hst_o(f_obs_hst), .obs_imask_o(f_obs_imask)
    );

    host_props #(.STANDALONE(1), .ENV_SPI(0)) u_f_props (
        .clk(clk), .rst_n(f_rst_n), .ph_i(f_ph),
        .sclk_i(f_sclk), .csn_i(f_csn), .mosi_i(f_mosi),
        .miso_o(f_miso), .irq_o(f_irq),
        .h_pm_en_o(f_pm_en), .h_pm_we_o(f_pm_we), .h_pm_addr_o(f_pm_addr),
        .h_pm_wdata_o(f_pm_wdata), .pm_rdata_i(f_pm_rdata),
        .h_dm_en_o(f_dm_en), .h_dm_we_o(f_dm_we), .h_dm_addr_o(f_dm_addr),
        .h_dm_wdata_o(f_dm_wdata), .dm_rdata_i(f_dm_rdata),
        .hreg_we_o(f_hreg_we), .hreg_addr_o(f_hreg_addr), .hreg_wdata_o(f_hreg_wdata),
        .hreg_rdata_i(f_hreg_rdata), .attn_i(f_attn), .qne_i(f_qne),
        .obs_sr_o(f_obs_sr), .obs_cmd_o(f_obs_cmd), .obs_bitcnt_o(f_obs_bitcnt),
        .obs_maddr_o(f_obs_maddr), .obs_hst_o(f_obs_hst), .obs_imask_o(f_obs_imask)
    );
`endif
endmodule
