`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/tick.md section 9 (rosetta_delay)
//  written by the formal property writer, only to show that formal/tick/
//  delay_*.sv can PASS and that the properties are non-vacuous. The real
//  block is src/rosetta_delay.sv, written independently. Do not integrate,
//  synthesise or copy this file.
//
//  Planted bugs (mutation testing), enabled one at a time with a define:
//    MUT_D_OFF1       matures one cycle/strobe late (pcnt + 1 > D)
//    MUT_D_PIN_ANY    any PIN cancels the pending change (unrelated output)
//    MUT_D_W1_IGN     a PIN on out1 cancels a data change that does not
//                     write out1 (w1 ignored)
//    MUT_D_PS_RAW     prescaled delay counts cycles
//    MUT_D_ZERO       D = 0 still delays (by one cycle)
//    MUT_D_FLUSH_LOST a new delayed request drops the pending change
//    MUT_D_DCAP_ANY   dcap_o for every delayed clock change
//  Ruling 58 (tick.md 9.2 rule 4 amended, 15 Nov: a delayed request made in
//  the cycle a PIN writes an output it would write is dropped):
//    MUT_DROP_NONE     no drop at all (the old rule 4)
//    MUT_DROP_NOPINWE  drop decided from pin_mask_i without pin_we_i
//    MUT_DROP_CLKBIT   clock stream tested against pin_mask_i[2], not [3]
//    MUT_DROP_NOW1     an out1-only PIN drops a data request without w1_new
//    MUT_DROP_NOFLUSH  a dropped request does not flush a pending change the
//                      PIN does not cancel (rule 3); it stays pending
//    MUT_DROP_CLRPEND  a drop also clears a pending change the PIN does not
//                      hit, without outputting it
// ===========================================================================
module rosetta_delay (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       run_i,
    input  wire       abort_i,
    input  wire       wr_tick_i,
    input  wire [7:0] cfg_dly_i,
    input  wire       cfg_dlyclk_i,
    input  wire       cfg_dlyps_i,
    input  wire       pt_i,
    input  wire       dreq_i,
    input  wire [4:0] dpay_i,
    input  wire       dreq_dly_i,
    input  wire       creq_i,
    input  wire [1:0] cpay_i,
    input  wire       creq_dly_i,
    input  wire       creq_cap_i,
    input  wire       pin_we_i,
    input  wire [3:0] pin_mask_i,
    output logic      d_we_o,
    output logic [4:0] d_pay_o,
    output logic      c_we_o,
    output logic [1:0] c_pay_o,
    output logic      dcap_o
);
    logic       pv, pclk, pcap;
    logic [4:0] ppay;
    logic [7:0] pcnt;

`ifdef MUT_D_ZERO
    wire dnz   = 1'b1;
`else
    wire dnz   = |cfg_dly_i;
`endif
    wire ddel  = dreq_i & dreq_dly_i & ~cfg_dlyclk_i & dnz;
    wire cdel  = creq_i & creq_dly_i &  cfg_dlyclk_i & dnz;
    wire dpass = dreq_i & ~ddel;
    wire cpass = creq_i & ~cdel;
    wire nreq  = ddel | cdel;
    wire w1n   = dpay_i[2];
`ifdef MUT_DROP_NONE
    wire drop  = 1'b0;
`elsif MUT_DROP_NOPINWE
    wire drop  = nreq & (cdel ? pin_mask_i[3] : (pin_mask_i[0] | (pin_mask_i[1] & w1n)));
`elsif MUT_DROP_CLKBIT
    wire drop  = nreq & pin_we_i & (cdel ? pin_mask_i[2] : (pin_mask_i[0] | (pin_mask_i[1] & w1n)));
`elsif MUT_DROP_NOW1
    wire drop  = nreq & pin_we_i & (cdel ? pin_mask_i[3] : (pin_mask_i[0] | pin_mask_i[1]));
`else
    wire drop  = nreq & pin_we_i & (cdel ? pin_mask_i[3] : (pin_mask_i[0] | (pin_mask_i[1] & w1n)));
`endif

`ifdef MUT_D_PS_RAW
    wire [7:0] step = 8'd1;
`else
    wire [7:0] step = cfg_dlyps_i ? {7'd0, pt_i} : 8'd1;
`endif
    wire [8:0] nxt  = {1'b0, pcnt} + {1'b0, step};
    wire [7:0] nsat = nxt[8] ? 8'hFF : nxt[7:0];
`ifdef MUT_D_OFF1
    wire mat = pv & ({1'b0, nsat} > {1'b0, cfg_dly_i});
`else
    wire mat = pv & (nsat >= cfg_dly_i);
`endif
`ifdef MUT_D_PIN_ANY
    wire phit = pin_we_i;
`elsif MUT_D_W1_IGN
    wire phit = pin_we_i & (pclk ? pin_mask_i[3] : (pin_mask_i[0] | pin_mask_i[1]));
`else
    wire phit = pin_we_i & (pclk ? pin_mask_i[3] : (pin_mask_i[0] | (pin_mask_i[1] & ppay[2])));
`endif
    wire can  = abort_i | wr_tick_i | (pclk ? cpass : dpass) | phit;
`ifdef MUT_D_FLUSH_LOST
    wire fire = pv & ~can & mat & ~nreq;
`elsif MUT_DROP_NOFLUSH
    wire fire = pv & ~can & (mat | (nreq & ~drop));
`elsif MUT_DROP_CLRPEND
    wire fire = pv & ~can & (mat | (nreq & ~drop));
`else
    wire fire = pv & ~can & (mat | nreq);
`endif

    always_comb begin
        d_we_o  = run_i & (dpass | (fire & ~pclk));
        d_pay_o = run_i ? (dpass ? dpay_i : ppay) : 5'd0;
        c_we_o  = run_i & (cpass | (fire & pclk));
        c_pay_o = run_i ? (cpass ? cpay_i : ppay[1:0]) : 2'd0;
`ifdef MUT_D_DCAP_ANY
        dcap_o  = run_i & fire & pclk;
`else
        dcap_o  = run_i & fire & pclk & pcap;
`endif
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pv <= 1'b0; pclk <= 1'b0; pcap <= 1'b0; ppay <= 5'd0; pcnt <= 8'd0;
        end else if (!run_i) begin
            pv <= 1'b0; pclk <= 1'b0; pcap <= 1'b0; ppay <= 5'd0; pcnt <= 8'd0;
        end else if (nreq & ~drop & ~abort_i & ~wr_tick_i) begin
            pv <= 1'b1; pclk <= cdel; pcap <= creq_cap_i; pcnt <= 8'd0;
            ppay <= cdel ? {3'b000, cpay_i} : dpay_i;
`ifdef MUT_DROP_CLRPEND
        end else if (fire | can | drop) begin
`else
        end else if (fire | can) begin
`endif
            pv <= 1'b0;
        end else begin
            pcnt <= nsat;
        end
    end
endmodule
