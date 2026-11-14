`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/host.md written by the formal
//  property writer, only to show that formal/host/*.sv can PASS and that the
//  properties are non-vacuous (the planted bugs below make tasks FAIL).
//  The real block is src/rosetta_host.sv, written independently.
//  Do not integrate, synthesise or copy this file.
//
//  Planted bugs (mutation testing), enabled one at a time with a define:
//    MUT_BIT_ORDER     write data shifted in LSB first (bit order reversed)
//    MUT_WR_EARLY      write request at rising edge 39 (one bit early)
//    MUT_WR_PHASE      REG write strobe in ph = 2 instead of ph = 1
//    MUT_WR_ABORT      CS_n rising after edge 32 of a write frame still writes
//    MUT_ADDR_BIT      hreg_addr_o = cmd[7:1] (address off by one bit)
//    MUT_RD_LATE       REG read data sampled one cycle after the service cycle
//    MUT_MISO_RISE     MISO updated on the SCLK rising edge
//    MUT_IRQ_NOMASK    irq ignores IRQMASK
//    MUT_STROBE2       hreg_we_o lasts two cycles
//    MUT_CS_NOCLR      bitcnt not cleared while CS_n is high
//    MUT_MDATA_NOINC   MDATA read does not increment MADDR
//    MUT_CAP_PRIO      a shift beats a read-data capture in the same cycle
//    MUT_IMASK_NORST   imask flop has no reset
//    MUT_SCLKP_RST1    sclk_prev resets to 1
// ===========================================================================
module rosetta_host (
    input  wire         clk,
    input  wire         rst_n,
    input  wire [4:0]   ph_i,
    input  wire         sclk_i,
    input  wire         csn_i,
    input  wire         mosi_i,
    output logic        miso_o,
    output logic        irq_o,
    output logic        h_pm_en_o,
    output logic        h_pm_we_o,
    output logic [8:0]  h_pm_addr_o,
    output logic [31:0] h_pm_wdata_o,
    input  wire  [31:0] pm_rdata_i,
    output logic        h_dm_en_o,
    output logic        h_dm_we_o,
    output logic [8:0]  h_dm_addr_o,
    output logic [31:0] h_dm_wdata_o,
    input  wire  [31:0] dm_rdata_i,
    output logic        hreg_we_o,
    output logic [6:0]  hreg_addr_o,
    output logic [31:0] hreg_wdata_o,
    input  wire  [31:0] hreg_rdata_i,
    input  wire  [3:0]  attn_i,
    input  wire  [3:0]  qne_i,
    output logic [31:0] obs_sr_o,
    output logic [7:0]  obs_cmd_o,
    output logic [5:0]  obs_bitcnt_o,
    output logic [9:0]  obs_maddr_o,
    output logic [1:0]  obs_hst_o,
    output logic [7:0]  obs_imask_o
);
    logic        sclk_prev;
    logic [31:0] sr;
    logic [5:0]  bitcnt;
    logic [7:0]  cmd;
    logic        miso;
    logic [9:0]  maddr;
    logic [1:0]  hst;
    logic [7:0]  imask;
    logic        irq;
`ifdef MUT_RD_LATE
    logic        rd_late;
`endif
`ifdef MUT_STROBE2
    logic        we_d;
`endif

    wire rise = sclk_i & ~sclk_prev;
    wire fall = ~sclk_i & sclk_prev;
    wire [6:0] k = {1'b0, bitcnt} + 7'd1;
    wire rd    = cmd[7];
    wire is_md = (cmd[6:0] == 7'h02);
    wire kprog = is_md & ~maddr[9];
    wire kdata = is_md &  maddr[9];
    wire kreg  = ~is_md;
`ifdef MUT_WR_PHASE
    wire serve = (hst == 2'd1) & (kprog ? ph_i[4] : (kreg & ~rd) ? ph_i[2] : ph_i[1]);
`else
    wire serve = (hst == 2'd1) & (kprog ? ph_i[4] : ph_i[1]);
`endif
    wire cap   = (hst == 2'd2) & ((kprog & ph_i[0]) | (kdata & ph_i[2]));
    wire wrreg = serve & kreg & ~rd;
`ifdef MUT_RD_LATE
    wire rdreg = rd_late;
`else
    wire rdreg = serve & kreg & rd;
`endif
    wire [31:0] rdata = (cmd[6:0] == 7'h00) ? 32'h52530401 :
                        (cmd[6:0] == 7'h01) ? {22'h0, maddr} :
                        (cmd[6:0] == 7'h05) ? {24'h0, imask} : hreg_rdata_i;
    wire edge_ok = ~csn_i & rise;
`ifdef MUT_WR_EARLY
    localparam [6:0] KW = 7'd39;
`else
    localparam [6:0] KW = 7'd40;
`endif
    wire shin = edge_ok & ((k <= 7'd8) | ((k >= 7'd9) & (k <= 7'd40) & ~rd));
    wire sh0  = edge_ok & (k >= 7'd17) & (k <= 7'd47) & rd;
`ifdef MUT_WR_ABORT
    wire req  = (edge_ok & (((k == 7'd8) & sr[6]) | ((k == KW) & ~rd))) |
                (csn_i & ~rd & (bitcnt >= 6'd33) & (bitcnt < 6'd40));
`else
    wire req  = edge_ok & (((k == 7'd8) & sr[6]) | ((k == KW) & ~rd));
`endif
`ifdef MUT_MISO_RISE
    wire miso_ev = rise;
`else
    wire miso_ev = fall;
`endif

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
`ifdef MUT_SCLKP_RST1
            sclk_prev <= 1'b1;
`else
            sclk_prev <= 1'b0;
`endif
            sr     <= 32'h0;
            bitcnt <= 6'd0;
            cmd    <= 8'h0;
            miso   <= 1'b0;
            maddr  <= 10'h0;
            hst    <= 2'd0;
            irq    <= 1'b0;
`ifdef MUT_RD_LATE
            rd_late <= 1'b0;
`endif
`ifdef MUT_STROBE2
            we_d   <= 1'b0;
`endif
        end else begin
            sclk_prev <= sclk_i;
`ifdef MUT_RD_LATE
            rd_late <= serve & kreg & rd;
`endif
`ifdef MUT_STROBE2
            we_d   <= wrreg;
`endif
`ifdef MUT_CS_NOCLR
            if (edge_ok)
                bitcnt <= (k > 7'd48) ? 6'd48 : k[5:0];
`else
            if (csn_i)
                bitcnt <= 6'd0;
            else if (rise)
                bitcnt <= (k > 7'd48) ? 6'd48 : k[5:0];
`endif
            if (edge_ok & (k == 7'd8))
                cmd <= {sr[6:0], mosi_i};
`ifdef MUT_CAP_PRIO
            if (shin)
                sr <= {sr[30:0], mosi_i};
            else if (sh0)
                sr <= {sr[30:0], 1'b0};
            else if (rdreg)
                sr <= rdata;
            else if (cap)
                sr <= kprog ? pm_rdata_i : dm_rdata_i;
`else
            if (rdreg)
                sr <= rdata;
            else if (cap)
                sr <= kprog ? pm_rdata_i : dm_rdata_i;
  `ifdef MUT_BIT_ORDER
            else if (shin & (k >= 7'd9))
                sr <= {mosi_i, sr[31:1]};
  `endif
            else if (shin)
                sr <= {sr[30:0], mosi_i};
            else if (sh0)
                sr <= {sr[30:0], 1'b0};
`endif
            if (csn_i)
                miso <= 1'b0;
            else if (miso_ev)
                miso <= (rd & (bitcnt >= 6'd16) & (bitcnt <= 6'd47)) ? sr[31] : 1'b0;
            if (wrreg & (cmd[6:0] == 7'h01))
                maddr <= sr[9:0];
`ifdef MUT_MDATA_NOINC
            else if (serve & is_md & ~rd)
`else
            else if (serve & is_md)
`endif
                maddr <= {maddr[9], maddr[8:0] + 9'd1};
            if ((hst == 2'd0) & req)
                hst <= 2'd1;
            else if (serve)
                hst <= (rd & is_md) ? 2'd2 : 2'd0;
            else if (cap)
                hst <= 2'd0;
`ifdef MUT_IRQ_NOMASK
            irq <= (|attn_i) | (|qne_i);
`else
            irq <= (|(attn_i & imask[3:0])) | (|(qne_i & imask[7:4]));
`endif
        end
    end

`ifdef MUT_IMASK_NORST
    always_ff @(posedge clk) begin
        if (rst_n && wrreg && (cmd[6:0] == 7'h05))
            imask <= sr[7:0];
    end
`else
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            imask <= 8'h0;
        else if (wrreg & (cmd[6:0] == 7'h05))
            imask <= sr[7:0];
    end
`endif

    always_comb begin
        miso_o       = miso;
        irq_o        = irq;
        h_pm_en_o    = serve & kprog;
        h_pm_we_o    = serve & kprog & ~rd;
        h_pm_addr_o  = maddr[8:0];
        h_pm_wdata_o = sr;
        h_dm_en_o    = serve & kdata;
        h_dm_we_o    = serve & kdata & ~rd;
        h_dm_addr_o  = maddr[8:0];
        h_dm_wdata_o = sr;
`ifdef MUT_STROBE2
        hreg_we_o    = wrreg | we_d;
`else
        hreg_we_o    = wrreg;
`endif
`ifdef MUT_ADDR_BIT
        hreg_addr_o  = cmd[7:1];
`else
        hreg_addr_o  = cmd[6:0];
`endif
        hreg_wdata_o = sr;
        obs_sr_o     = sr;
        obs_cmd_o    = cmd;
        obs_bitcnt_o = bitcnt;
        obs_maddr_o  = maddr;
        obs_hst_o    = hst;
        obs_imask_o  = imask;
    end
endmodule
