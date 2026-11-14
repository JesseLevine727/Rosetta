`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/xbar.md as amended by
//  docs/decisions.md ruling 14 (both synchroniser flops ahead of the
//  crossbar multiplexer; the lane-input register after it: the "fallback"
//  of xbar.md X1). Written by the formal property writer only to show that
//  formal/xbar/*.sv can PASS and that every property is non-vacuous
//  (planted bugs below make tasks FAIL). The real block is
//  src/rosetta_xbar.sv, written independently. Do not integrate, synthesise
//  or copy this file.
//
//  Planted bugs (mutation testing), enabled one at a time with a define:
//    MUT_ONE_SYNC        lane path: the mux reads s1 (one synchroniser flop
//                        missing)
//    MUT_HOST_ONE_SYNC   host path: host_*_o taken from s1
//    MUT_NO_LIN          lane-input register missing (lane_in_o = mux)
//    MUT_SEL_OFF1        input select decoded off by one (source insel+1)
//    MUT_OE_NORST        outsel en bit of uio[1] (p = 7) has no reset
//    MUT_UIO_OE_INPUT    uio_oe ignores en: an unmapped (input-only) uio pin
//                        is driven when lane 0 out0 enables
//    MUT_LOOP_PREGATE    loopback taps lane_val_i of the mapped output,
//                        before the en/run/oe gating
//    MUT_LOOP_SYNC       loopback enters ahead of the two synchroniser flops
//                        (two cycles late)
//    MUT_HOST_XBAR       host SCLK/CS_n/MOSI routed through the crossbar
//                        (taken from lane 0 inputs 0-2)
//    MUT_CFG_WRONG_REG   a write to 0x41 (XIN1) lands in XIN0
//    MUT_CFG_ALIAS       addr[6] not decoded: writes to 0x00-0x05 hit 0x40-0x45
//    MUT_EN_UNSELECTED   the enable of a pin is taken from lane 0's output oi
//                        whatever lane is selected
//    MUT_RUN_UNGATED     drive ignores run_i
//    MUT_CSN_RST0        s1 resets to 0 (CS_n not idle high after reset)
//    MUT_RDATA_UNUSED    XOUT0 bits [31:30] kept and read back
//    MUT_LOOP_REL0       loopback of a released uio pin reads 0
// ===========================================================================
module rosetta_xbar (
    input  wire         clk,
    input  wire         rst_n,
    input  wire  [7:0]  ui_in_i,
    input  wire  [7:0]  uio_in_i,
    output logic        host_sclk_o,
    output logic        host_csn_o,
    output logic        host_mosi_o,
    input  wire         host_miso_i,
    input  wire         host_irq_i,
    output logic [15:0] lane_in_o,
    input  wire  [15:0] lane_val_i,
    input  wire  [15:0] lane_oe_i,
    input  wire  [3:0]  run_i,
    output logic [7:0]  uo_out_o,
    output logic [7:0]  uio_out_o,
    output logic [7:0]  uio_oe_o,
    input  wire         hreg_we_i,
    input  wire  [6:0]  hreg_addr_i,
    input  wire  [31:0] hreg_wdata_i,
    output logic [31:0] hreg_rdata_o,
    output logic [63:0] obs_insel_o,
    output logic [69:0] obs_outsel_o,
    output logic        obs_loop_o
);
    wire [15:0] pins = {uio_in_i, ui_in_i};

    logic [15:0] s1, s2, lin;
    logic [63:0] insel;
    logic [69:0] outsel;
    logic        loop;
`ifdef MUT_RDATA_UNUSED
    logic [1:0]  xout0_hi;
`endif

    // ---------------- synchronisers: two flops per pin ----------------
`ifdef MUT_CSN_RST0
    localparam [15:0] S_RST = 16'h0000;
`else
    localparam [15:0] S_RST = 16'h0002;
`endif
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1 <= S_RST;
            s2 <= S_RST;
        end else begin
            s1 <= pins;
            s2 <= s1;
        end
    end

`ifdef MUT_HOST_ONE_SYNC
    assign {host_mosi_o, host_csn_o, host_sclk_o} = s1[2:0];
`elsif MUT_HOST_XBAR
    assign {host_mosi_o, host_csn_o, host_sclk_o} = lin[2:0];
`else
    assign {host_mosi_o, host_csn_o, host_sclk_o} = s2[2:0];
`endif

    // ---------------- output pins (combinational) ----------------
    logic [13:0] drive, pval;
    logic [13:0] en_eff;
    integer p;
    always_comb begin
        for (p = 0; p < 14; p = p + 1) begin
            en_eff[p] = outsel[5*p+4];
`ifdef MUT_RUN_UNGATED
            drive[p] = en_eff[p] && lane_oe_i[outsel[5*p +: 4]];
`elsif MUT_EN_UNSELECTED
            drive[p] = en_eff[p] && run_i[outsel[5*p+2 +: 2]] && lane_oe_i[{2'b00, outsel[5*p +: 2]}];
`else
            drive[p] = en_eff[p] && run_i[outsel[5*p+2 +: 2]] && lane_oe_i[outsel[5*p +: 4]];
`endif
            pval[p]  = drive[p] && lane_val_i[outsel[5*p +: 4]];
        end
    end

    assign uo_out_o  = {pval[5:0], host_irq_i, host_miso_i};
    assign uio_out_o = pval[13:6];
`ifdef MUT_UIO_OE_INPUT
    always_comb begin
        for (p = 6; p < 14; p = p + 1)
            uio_oe_o[p-6] = run_i[outsel[5*p+2 +: 2]] && lane_oe_i[outsel[5*p +: 4]];
    end
`else
    assign uio_oe_o  = drive[13:6];
`endif

    // ---------------- crossbar and lane-input register ----------------
    logic [15:0] lsrc, src, nlin;
`ifdef MUT_LOOP_REL0
    assign lsrc = {uio_oe_o & uio_out_o, uo_out_o};
`elsif MUT_LOOP_PREGATE
    always_comb begin
        lsrc[1:0] = uo_out_o[1:0];
        for (p = 0; p < 14; p = p + 1)
            lsrc[p+2] = lane_val_i[outsel[5*p +: 4]];
    end
`else
    assign lsrc = {~uio_oe_o | uio_out_o, uo_out_o};
`endif

`ifdef MUT_LOOP_SYNC
    logic [15:0] ls1, ls2;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ls1 <= 16'h0;
            ls2 <= 16'h0;
        end else begin
            ls1 <= lsrc;
            ls2 <= ls1;
        end
    end
    wire [15:0] lsrc_eff = ls2;
`else
    wire [15:0] lsrc_eff = lsrc;
`endif

`ifdef MUT_ONE_SYNC
    assign src = loop ? lsrc_eff : s1;
`else
    assign src = loop ? lsrc_eff : s2;
`endif

    integer i;
    always_comb begin
        for (i = 0; i < 16; i = i + 1) begin
`ifdef MUT_SEL_OFF1
            nlin[i] = src[insel[4*i +: 4] + 4'd1];
`else
            nlin[i] = src[insel[4*i +: 4]];
`endif
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) lin <= 16'h0;
        else        lin <= nlin;
    end

`ifdef MUT_NO_LIN
    assign lane_in_o = nlin;
`else
    assign lane_in_o = lin;
`endif

    // ---------------- configuration registers ----------------
`ifdef MUT_CFG_ALIAS
    wire addr_hit = 1'b1;
`else
    wire addr_hit = hreg_addr_i[6];
`endif
    wire [5:0] a = hreg_addr_i[5:0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            insel       <= 64'h0;
`ifdef MUT_OE_NORST
            outsel[34:0]  <= 35'h0;
            outsel[38:35] <= 4'h0;
            outsel[69:40] <= 30'h0;
`else
            outsel      <= 70'h0;
`endif
            loop        <= 1'b0;
`ifdef MUT_RDATA_UNUSED
            xout0_hi    <= 2'b00;
`endif
        end else if (hreg_we_i && addr_hit) begin
            case (a)
                6'h00: insel[31:0]  <= hreg_wdata_i;
`ifdef MUT_CFG_WRONG_REG
                6'h01: insel[31:0]  <= hreg_wdata_i;
`else
                6'h01: insel[63:32] <= hreg_wdata_i;
`endif
                6'h02: begin
                    outsel[29:0] <= hreg_wdata_i[29:0];
`ifdef MUT_RDATA_UNUSED
                    xout0_hi <= hreg_wdata_i[31:30];
`endif
                end
`ifdef MUT_OE_NORST
                6'h03: {outsel[59:40], outsel[38:30]} <= {hreg_wdata_i[29:10], hreg_wdata_i[8:0]};
`else
                6'h03: outsel[59:30] <= hreg_wdata_i[29:0];
`endif
                6'h04: outsel[69:60] <= hreg_wdata_i[9:0];
                6'h05: loop          <= hreg_wdata_i[0];
                default: ;
            endcase
        end
    end

`ifdef MUT_OE_NORST
    // outsel[39] = en of p = 7 (uio[1]): no reset (powers up at any value)
    always_ff @(posedge clk) begin
        if (hreg_we_i && addr_hit && a == 6'h03) outsel[39] <= hreg_wdata_i[9];
    end
`endif

    always_comb begin
        hreg_rdata_o = 32'h0;
        if (hreg_addr_i[6]) begin
            case (a)
                6'h00: hreg_rdata_o = insel[31:0];
                6'h01: hreg_rdata_o = insel[63:32];
`ifdef MUT_RDATA_UNUSED
                6'h02: hreg_rdata_o = {xout0_hi, outsel[29:0]};
`else
                6'h02: hreg_rdata_o = {2'b00, outsel[29:0]};
`endif
                6'h03: hreg_rdata_o = {2'b00, outsel[59:30]};
                6'h04: hreg_rdata_o = {22'h0, outsel[69:60]};
                6'h05: hreg_rdata_o = {31'h0, loop};
                default: ;
            endcase
        end
    end

    assign obs_insel_o  = insel;
    assign obs_outsel_o = outsel;
    assign obs_loop_o   = loop;
endmodule
