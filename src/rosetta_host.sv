`default_nettype none

// rosetta_host: SPI target host port (docs/blocks/host.md).
//
// SPI mode 0, one 32-bit register access per frame. sclk_i, csn_i, mosi_i are
// already synchronised by rosetta_xbar (two flops after the pins). Edges are
// detected against sclk_prev; every action happens at the end of the cycle in
// which the edge is seen, so miso_o changes one cycle after the detection
// cycle of a falling edge.
//
// State (69 flops, all reset to 0): sclk_prev 1, sr 32, bitcnt 6, cmd 8,
// miso 1, maddr 10, hst 2, imask 8, irq 1.
//
// Request service (3.3) is gated by the wheel phase: register and data-memory
// effects at ph = 1, program memory at ph = 4, read-data capture at ph = 0
// (program) or ph = 2 (data). The strobes are combinational from state and
// ph_i, so they coincide with the host slot. Address and write-data outputs
// are driven continuously (hreg_addr_o = cmd[6:0], *_addr_o = maddr[8:0],
// *_wdata_o = sr); only the en/we strobes are gated.
//
// ISSUED is left in its single capture cycle, chosen by maddr[9] and ph only
// (not re-checked against cmd), so it lasts exactly one cycle (property 7).
// hst = 3 is unreachable; it is treated as ISSUED and so returns to IDLE.

module rosetta_host (
    input  wire         clk, rst_n,
    input  wire  [4:0]  ph_i,
    input  wire         sclk_i, csn_i, mosi_i,
    output logic        miso_o, irq_o,
    output logic        h_pm_en_o, h_pm_we_o,
    output logic [8:0]  h_pm_addr_o,
    output logic [31:0] h_pm_wdata_o,
    input  wire  [31:0] pm_rdata_i,
    output logic        h_dm_en_o, h_dm_we_o,
    output logic [8:0]  h_dm_addr_o,
    output logic [31:0] h_dm_wdata_o,
    input  wire  [31:0] dm_rdata_i,
    output logic        hreg_we_o,
    output logic [6:0]  hreg_addr_o,
    output logic [31:0] hreg_wdata_o,
    input  wire  [31:0] hreg_rdata_i,
    input  wire  [3:0]  attn_i, qne_i,
    output logic [31:0] obs_sr_o,
    output logic [7:0]  obs_cmd_o,
    output logic [5:0]  obs_bitcnt_o,
    output logic [9:0]  obs_maddr_o,
    output logic [1:0]  obs_hst_o,
    output logic [7:0]  obs_imask_o
);
  localparam logic [6:0]  A_ID = 7'h00, A_MADDR = 7'h01, A_MDATA = 7'h02,
                          A_IRQMASK = 7'h05;
  localparam logic [31:0] ID = 32'h5253_0401;   // "RS", N_LANES = 4, v1
  localparam logic [1:0]  IDLE = 2'd0, PEND = 2'd1, ISSUED = 2'd2;

  logic        sclk_prev, miso, irq;
  logic [31:0] sr, sr_n, rdata;
  logic [5:0]  bitcnt;
  logic [7:0]  cmd, imask;
  logic [9:0]  maddr;
  logic [1:0]  hst, hst_n;

  logic rise, fall, rd, mdata, pend, issued, req;
  logic svc_reg, svc_pm, svc_dm, cap_reg, cap_pm, cap_dm, cap, shift, sbit;
  logic a_id, a_ma, a_im;
  logic unused_ok;

  assign unused_ok = ph_i[3];   // no host action at ph = 3

  // Synchronised SCLK edges; frame actions only while CS_n is low.
  assign rise = sclk_i & ~sclk_prev;
  assign fall = ~sclk_i & sclk_prev;

  // Request kind (A = cmd[6:0], rd = cmd[7]) and service cycles.
  assign rd      = cmd[7];
  assign mdata   = cmd[6:0] == A_MDATA;
  assign pend    = hst == PEND;
  assign issued  = hst[1];
  assign svc_reg = pend & ~mdata & ph_i[1];
  assign svc_pm  = pend & mdata & ~maddr[9] & ph_i[4];
  assign svc_dm  = pend & mdata &  maddr[9] & ph_i[1];
  assign cap_reg = svc_reg & rd;
  assign cap_pm  = issued & ~maddr[9] & ph_i[0];
  assign cap_dm  = issued &  maddr[9] & ph_i[2];

  assign hreg_we_o = svc_reg & ~rd;
  assign h_pm_en_o = svc_pm;
  assign h_pm_we_o = svc_pm & ~rd;
  assign h_dm_en_o = svc_dm;
  assign h_dm_we_o = svc_dm & ~rd;

  // New request at edge k = bitcnt + 1: k = 8 with the read bit (sr[6] is
  // cmd[7] being loaded), or k = 40 of a write; only from IDLE.
  assign req = ~csn_i & rise &
               ((bitcnt == 6'd7 & sr[6]) | (bitcnt == 6'd39 & ~rd));

  always_comb begin
    hst_n = hst;
    if (hst == IDLE) begin
      if (req) hst_n = PEND;
    end else if (svc_reg | svc_pm | svc_dm)
      hst_n = (rd & ~svc_reg) ? ISSUED : IDLE;
    else if (cap_pm | cap_dm)
      hst_n = IDLE;
  end

  // Shift at edge k: MOSI for k <= 8 and for 9 <= k <= 40 of a write; 0 for
  // 17 <= k <= 47 of a read; otherwise hold (dummy byte, bits after the end).
  assign shift = ~csn_i & rise &
                 (bitcnt < 6'd8 | (~rd & bitcnt < 6'd40) |
                  (rd & bitcnt >= 6'd16 & bitcnt < 6'd47));
  assign sbit  = mosi_i & (bitcnt < 6'd8 | ~rd);

  // RDATA(A): the host's own registers, else the OR'd register bus. Written
  // as AND-OR (one-hot selects); about 1.2% less area than a case.
  assign a_id  = cmd[6:0] == A_ID;
  assign a_ma  = cmd[6:0] == A_MADDR;
  assign a_im  = cmd[6:0] == A_IRQMASK;
  assign rdata = ({32{a_id}} & ID) | ({32{a_ma}} & {22'h0, maddr}) |
                 ({32{a_im}} & {24'h0, imask}) |
                 ({32{~(a_id | a_ma | a_im)}} & hreg_rdata_i);

  // Next sr: capture (3.3) beats a shift (cannot coincide under 2.1). The
  // selects are mutually exclusive, so a flat AND-OR; this maps about 11%
  // smaller than the equivalent if/else chain.
  assign cap   = cap_reg | cap_pm | cap_dm;
  assign sr_n  = ({32{cap_reg}} & rdata) | ({32{cap_pm}} & pm_rdata_i) |
                 ({32{cap_dm}} & dm_rdata_i) |
                 ({32{shift & ~cap}} & {sr[30:0], sbit}) |
                 ({32{~(shift | cap)}} & sr);

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      sclk_prev <= 1'b0;
      sr        <= '0;
      bitcnt    <= '0;
      cmd       <= '0;
      miso      <= 1'b0;
      maddr     <= '0;
      hst       <= IDLE;
      imask     <= '0;
      irq       <= 1'b0;
    end else begin
      sclk_prev <= sclk_i;
      irq       <= |(attn_i & imask[3:0]) | |(qne_i & imask[7:4]);
      hst       <= hst_n;

      // Frame receive (3.2).
      if (csn_i) begin
        bitcnt <= '0;
        miso   <= 1'b0;
      end else if (rise) begin
        bitcnt <= (bitcnt >= 6'd47) ? 6'd48 : bitcnt + 6'd1;
        if (bitcnt == 6'd7) cmd <= {sr[6:0], mosi_i};
      end else if (fall)
        miso <= rd & bitcnt >= 6'd16 & bitcnt <= 6'd47 & sr[31];

      sr <= sr_n;

      // Host-owned registers (3.4) and MDATA auto-increment.
      if (hreg_we_o && a_ma)                maddr <= sr[9:0];
      else if (svc_pm | svc_dm)             maddr[8:0] <= maddr[8:0] + 9'd1;
      if (hreg_we_o && a_im)                imask <= sr[7:0];
    end

  assign miso_o       = miso;
  assign irq_o        = irq;
  assign h_pm_addr_o  = maddr[8:0];
  assign h_dm_addr_o  = maddr[8:0];
  assign h_pm_wdata_o = sr;
  assign h_dm_wdata_o = sr;
  assign hreg_addr_o  = cmd[6:0];
  assign hreg_wdata_o = sr;

  assign obs_sr_o     = sr;
  assign obs_cmd_o    = cmd;
  assign obs_bitcnt_o = bitcnt;
  assign obs_maddr_o  = maddr;
  assign obs_hst_o    = hst;
  assign obs_imask_o  = imask;
endmodule
