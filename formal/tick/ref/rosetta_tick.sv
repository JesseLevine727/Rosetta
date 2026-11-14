`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/tick.md (rosetta_tick) written
//  by the formal property writer, only to show that formal/tick/*.sv can
//  PASS and that the properties are non-vacuous (the planted bugs below
//  make tasks FAIL). The real block is src/rosetta_tick.sv, written
//  independently. Do not integrate, synthesise or copy this file.
//
//  The operation timer follows the spec's implementation note (interval
//  counter + strobe counter), unlike the checker's single N countdown.
//
//  Planted bugs (mutation testing), enabled one at a time with a define:
//    MUT_PERIOD_OFF1     BAUD capture when the count exceeds Pd (one late)
//    MUT_PERIOD2_IGN     end of bit counted against Pd instead of P2d
//    MUT_PRE_FLDRST      the prescaler phase restarts at a FIELD start
//    MUT_EDGE_SWAP       EDGE captures on the launch edge (polarity swap)
//    MUT_REC_NOSYNC      RECOVER: a data edge gives lt but does not restart
//    MUT_HONOUR_EARLY    honour wait ends the cycle after cte, cin ignored
//    MUT_RTRIP3          BAUD sample 3 cycles after its reference, not 4
//    MUT_DLYREF_IGN      sample referenced to ct even with a delayed clock
//    MUT_NO_POSTPONE     capture not postponed while a sample is pending
//    MUT_THR_OFF1        ge0 = d8 > THR0
//    MUT_ECNT_WRAP       edge interval counter wraps instead of saturating
//    MUT_OT_N1           timer with N = 1 never finishes (no start-cycle case)
//    MUT_REST_WRONG      stop requests the idle level even with rest
//    MUT_TWQ_DLY         TICK-write idle request marked delayed
//    MUT_STR_NOWAIT      stretch start ignores data_wait
//    MUT_ABORT_RECCNT    abort clears the RECOVER count
//    MUT_CT_REPEAT       BAUD capture without honour stays in phase 0 (a
//                        second capture without a launch)
//    MUT_ST_HON_ANY      with honour, st = ct even without cu_smp
// ===========================================================================
module rosetta_tick #(
    parameter RTRIP = 4
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run_i,
    input  wire        abort_i,
    input  wire        wr_tick_i,
    input  wire [1:0]  cfg_mode_i,
    input  wire [1:0]  cfg_ps_i,
    input  wire [1:0]  cfg_clkin_i,
    input  wire        cfg_capfall_i,
    input  wire        cfg_clkout_i,
    input  wire        cfg_idle_i,
    input  wire        cfg_clkod_i,
    input  wire        cfg_rest_i,
    input  wire        cfg_honour_i,
    input  wire        cfg_hold_i,
    input  wire [15:0] cfg_period_i,
    input  wire [15:0] cfg_period2_i,
    input  wire [7:0]  cfg_thr0_i,
    input  wire [7:0]  cfg_pw0_i,
    input  wire [7:0]  cfg_pw1_i,
    input  wire        cfg_dlyclk_i,
    input  wire [7:0]  cfg_dly_i,
    input  wire [3:0]  in_i,
    input  wire [3:0]  in_q_i,
    input  wire        dline_i,
    input  wire        dedge_i,
    input  wire        pw_bit_i,
    input  wire        cu_v_i,
    input  wire        cu_fld_i,
    input  wire        cu_tx_i,
    input  wire        cu_smp_i,
    input  wire        pk_v_i,
    input  wire        launch_req_i,
    input  wire        fld_next_i,
    input  wire        fld_first_i,
    input  wire        data_wait_i,
    input  wire        ot_start_i,
    input  wire        ot_raw_i,
    input  wire [15:0] ot_n_i,
    input  wire        dcap_i,
    output logic       pt_o,
    output logic       lt_o,
    output logic       ct_o,
    output logic       st_o,
    output logic       cte_o,
    output logic       hte_o,
    output logic       spend_o,
    output logic       bt_run_o,
    output logic       bt_stop_o,
    output logic       ot_done_o,
    output logic [15:0] d16_o,
    output logic       ge0_o,
    output logic       creq_o,
    output logic [1:0] cpay_o,
    output logic       creq_dly_o,
    output logic       creq_cap_o,
    output logic       stretch_o
);
    localparam [1:0] BAUD = 2'd0, EDGE = 2'd1, REC = 2'd2, PULSE = 2'd3;

    // ---------------- state
    logic [8:0]  pre;
    logic        bt_run, bt_ph, lr, hw;
    logic [15:0] cnt;
    logic        spend, sarm, rwait;
    logic [2:0]  sdn;
    logic        stretch, srel, twq;
    logic [15:0] ecnt;
    logic        ot_act;
    logic [11:0] ot_iv;
    logic [15:0] ot_sc;

    // ---------------- prescaler
    function automatic pt_of(input [8:0] p, input [1:0] ps);
        case (ps)
            2'd0:    pt_of = 1'b1;
            2'd1:    pt_of = &p[2:0];
            2'd2:    pt_of = &p[5:0];
            default: pt_of = &p;
        endcase
    endfunction
    wire [8:0] pre1 = pre + 9'd1;
    wire pt   = pt_of(pre, cfg_ps_i);
    wire ptn  = pt_of(pre1, cfg_ps_i);

    // ---------------- common
    wire G     = run_i && !abort_i;
    wire CL    = ~cfg_capfall_i;
    wire LL    = cfg_capfall_i;
    wire cin   = in_i[cfg_clkin_i];
    wire cinq  = in_q_i[cfg_clkin_i];
    wire [15:0] Pd  = (cfg_period_i == 16'd0)  ? 16'd1 : cfg_period_i;
    wire [15:0] P2d = (cfg_period2_i == 16'd0) ? 16'd1 : cfg_period2_i;
    wire [15:0] Pr  = (cfg_period_i[15:1] == 15'd0) ? 16'd2 : cfg_period_i;
    wire [15:0] Hh  = {1'b0, Pr[15:1]};
    wire [15:0] inc = (cnt == 16'hFFFF) ? cnt : cnt + {15'd0, pt};
    wire baudm  = (cfg_mode_i == BAUD) || (cfg_mode_i == PULSE);
    wire baudt  = (cfg_mode_i == BAUD) || ((cfg_mode_i == PULSE) && cu_tx_i);
    wire chg    = cin ^ cinq;
`ifdef MUT_EDGE_SWAP
    wire el     = chg && (cin == CL);
    wire ec     = chg && (cin == LL);
`else
    wire el     = chg && (cin == LL);
    wire ec     = chg && (cin == CL);
`endif

    // ---------------- BAUD timer
    wire st_stop = ~bt_run;
    wire st_p0   = bt_run & ~bt_ph & ~hw;
    wire st_hw   = bt_run & ~bt_ph & hw;
    wire st_p1   = bt_run & bt_ph;
`ifdef MUT_PERIOD_OFF1
    wire due     = inc > Pd;
`else
    wire due     = inc >= Pd;
`endif
`ifdef MUT_NO_POSTPONE
    wire bcte    = st_p0 & due;
`else
    wire bcte    = st_p0 & due & ~spend;
`endif
`ifdef MUT_HONOUR_EARLY
    wire hcap    = st_hw;
`else
    wire hcap    = st_hw & (cin == CL);
`endif
    wire bct     = (bcte & ~cfg_honour_i) | hcap;
`ifdef MUT_PERIOD2_IGN
    wire eob     = lr | (inc >= Pd);
`else
    wire eob     = lr | (inc >= P2d);
`endif
    wire blt     = (st_stop & launch_req_i) | (st_p1 & eob & launch_req_i);
    wire bstop   = st_p1 & eob & ~launch_req_i & ~fld_next_i;
    wire [7:0] H = pw_bit_i ? cfg_pw1_i : cfg_pw0_i;
    wire bhte    = st_p0 & pt & (inc == {8'd0, H}) & (H != 8'd0);
    wire bon     = G & baudm;

    // ---------------- EDGE stretch
    wire eon     = G & (cfg_mode_i == EDGE);
`ifdef MUT_STR_NOWAIT
    wire sstart  = eon & cfg_hold_i & cfg_clkout_i & ~stretch & el & ~cu_v_i & ~pk_v_i;
`else
    wire sstart  = eon & cfg_hold_i & cfg_clkout_i & ~stretch & el & ~cu_v_i & ~pk_v_i & data_wait_i;
`endif
    wire sfirst  = eon & stretch & fld_first_i & ~srel;   // ruling 41
    wire srelq   = eon & srel & (inc >= Pd);

    // ---------------- RECOVER
    wire rwrap   = pt & (inc >= Pr);
    wire rlt     = dedge_i | rwrap;
    wire rct     = ~dedge_i & pt & (inc == Hh);
`ifdef MUT_REC_NOSYNC
    wire [15:0] rcnt_n = rwrap ? 16'd0 : inc;
`else
    wire [15:0] rcnt_n = (dedge_i | rwrap) ? 16'd0 : inc;
`endif

    // ---------------- sample
    wire dlyc    = cfg_dlyclk_i & (cfg_dly_i != 8'd0) & cfg_clkout_i;
    wire samp    = sarm & (sdn == 3'd0) & spend;
`ifdef MUT_ST_HON_ANY
    wire bst     = cfg_honour_i ? bct : samp;
`else
    wire bst     = cfg_honour_i ? (bct & cu_smp_i) : samp;
`endif
    wire sset    = G & baudt & ~cfg_honour_i & bct & cu_smp_i;
    wire sarmev  = G & baudm & bct;                      // ruling 42
`ifdef MUT_DLYREF_IGN
    wire refimm  = 1'b1;
`else
    wire refimm  = ~dlyc | dcap_i;
`endif
`ifdef MUT_RTRIP3
    localparam [2:0] SLOAD = RTRIP - 2;
`else
    localparam [2:0] SLOAD = RTRIP - 1;
`endif

    // ---------------- operation timer
    wire        o_w0   = ot_start_i & ~ot_raw_i & (ot_n_i[11:0] == 12'd0);   // ruling 43
    wire        o_eact = (ot_start_i & ~o_w0) | (ot_act & ~ot_start_i);
    wire [15:0] o_esc  = ot_start_i ? (ot_raw_i ? ((ot_n_i == 16'd0) ? 16'd1 : ot_n_i) : Pd) : ot_sc;
    wire [11:0] o_eiv  = ot_start_i ? (ot_raw_i ? 12'd0 : ot_n_i[11:0] - 12'd1) : ot_iv;
`ifdef MUT_OT_N1
    wire        o_done = G & ot_act & ~ot_start_i & ptn & (ot_sc == 16'd1) & (ot_iv == 12'd0);
`else
    wire        o_done = G & o_eact & ptn & (o_esc == 16'd1) & (o_eiv == 12'd0);
`endif

    // ---------------- outputs
    logic lvl;
    wire mct = (cfg_mode_i == EDGE) ? ec : (cfg_mode_i == REC) ? rct : dedge_i;
    always_comb begin
        pt_o      = pt;
        lt_o      = 1'b0; ct_o = 1'b0; st_o = 1'b0; cte_o = 1'b0; hte_o = 1'b0;
        if (G) begin
            if (baudt) begin
                lt_o = blt; ct_o = bct; cte_o = bcte; hte_o = bhte; st_o = bst;
            end else begin
                ct_o = mct; st_o = mct;
                case (cfg_mode_i)
                    EDGE:    begin lt_o = el | (stretch & fld_first_i); cte_o = ec; end
                    REC:     begin lt_o = rlt; cte_o = rct; end
                    default: ;
                endcase
            end
        end
        bt_stop_o = bon & bstop;
        ot_done_o = o_done;
        spend_o   = spend;
        bt_run_o  = bt_run;
        stretch_o = stretch;
        d16_o     = ecnt;
`ifdef MUT_THR_OFF1
        ge0_o     = ((ecnt[15:8] != 8'd0) ? 8'hFF : ecnt[7:0]) > cfg_thr0_i;
`else
        ge0_o     = ((ecnt[15:8] != 8'd0) ? 8'hFF : ecnt[7:0]) >= cfg_thr0_i;
`endif
        // clock request
        creq_o = 1'b0; cpay_o = 2'b00; creq_dly_o = 1'b0; creq_cap_o = 1'b0;
        begin
            lvl = cfg_idle_i;
            if (G & twq) begin
                creq_o = 1'b1; lvl = cfg_idle_i;
`ifdef MUT_TWQ_DLY
                creq_dly_o = 1'b1;
`endif
            end else if (bon & (blt | bcte | bstop)) begin
                creq_o = 1'b1; creq_dly_o = 1'b1; creq_cap_o = bcte;
`ifdef MUT_REST_WRONG
                lvl = bcte ? CL : blt ? LL : cfg_idle_i;
`else
                lvl = bcte ? CL : blt ? LL : (cfg_rest_i ? LL : cfg_idle_i);
`endif
            end else if (sstart) begin
                creq_o = 1'b1; lvl = 1'b0;
            end else if (srelq | (abort_i & stretch)) begin
                creq_o = 1'b1; lvl = cfg_idle_i;
            end
            creq_o = creq_o & cfg_clkout_i;
            cpay_o = cfg_clkod_i ? {1'b0, ~lvl} : {lvl, 1'b1};
        end
    end

    // ---------------- registers
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pre <= 9'd0; bt_run <= 1'b0; bt_ph <= 1'b0; lr <= 1'b0; hw <= 1'b0; cnt <= 16'd0;
            spend <= 1'b0; sarm <= 1'b0; rwait <= 1'b0; sdn <= 3'd0;
            stretch <= 1'b0; srel <= 1'b0; twq <= 1'b0; ecnt <= 16'd0;
            ot_act <= 1'b0; ot_iv <= 12'd0; ot_sc <= 16'd0;
        end else if (!run_i) begin
            pre <= 9'd0; bt_run <= 1'b0; bt_ph <= 1'b0; lr <= 1'b0; hw <= 1'b0; cnt <= 16'd0;
            spend <= 1'b0; sarm <= 1'b0; rwait <= 1'b0; sdn <= 3'd0;
            stretch <= 1'b0; srel <= 1'b0; twq <= 1'b0; ecnt <= 16'd0;
            ot_act <= 1'b0; ot_iv <= 12'd0; ot_sc <= 16'd0;
        end else begin
`ifdef MUT_PRE_FLDRST
            pre <= fld_first_i ? 9'd0 : pre1;
`else
            pre <= pre1;
`endif
            twq <= wr_tick_i;
`ifdef MUT_ECNT_WRAP
            ecnt <= dedge_i ? {15'd0, pt} : ecnt + {15'd0, pt};
`else
            ecnt <= dedge_i ? {15'd0, pt} : ((ecnt == 16'hFFFF) ? ecnt : ecnt + {15'd0, pt});
`endif
            // operation timer
            if (!G | o_w0) begin
                ot_act <= 1'b0;
            end else if (o_eact) begin
                if (o_done) begin
                    ot_act <= 1'b0;
                end else begin
                    ot_act <= 1'b1;
                    if (ptn) begin
                        if (o_esc == 16'd1) begin ot_iv <= o_eiv - 12'd1; ot_sc <= Pd; end
                        else begin ot_iv <= o_eiv; ot_sc <= o_esc - 16'd1; end
                    end else begin
                        ot_iv <= o_eiv; ot_sc <= o_esc;
                    end
                end
            end
            if (abort_i | wr_tick_i) begin
                bt_run <= 1'b0; bt_ph <= 1'b0; lr <= 1'b0; hw <= 1'b0;
                spend <= 1'b0; sarm <= 1'b0; rwait <= 1'b0; sdn <= 3'd0;
                stretch <= 1'b0; srel <= 1'b0;
`ifdef MUT_ABORT_RECCNT
                cnt <= 16'd0;
`else
                cnt <= (abort_i & ~wr_tick_i & (cfg_mode_i == REC)) ? rcnt_n : 16'd0;
`endif
            end else begin
                case (cfg_mode_i)
                    REC: cnt <= rcnt_n;
                    EDGE: begin
                        if (sstart) stretch <= 1'b1;
                        if (sfirst) begin
                            srel <= 1'b1; cnt <= 16'd0;
                        end else if (srel) begin
                            if (inc >= Pd) begin stretch <= 1'b0; srel <= 1'b0; end
                            else cnt <= inc;
                        end
                    end
                    default: begin  // BAUD, PULSE
                        if (st_stop) begin
                            if (launch_req_i) begin
                                bt_run <= 1'b1; bt_ph <= 1'b0; cnt <= 16'd0; lr <= 1'b0; hw <= 1'b0;
                            end
                        end else if (st_p0) begin
                            if (bcte & cfg_honour_i)       hw <= 1'b1;
`ifdef MUT_CT_REPEAT
                            else if (bcte)                 begin cnt <= 16'd0; end
`else
                            else if (bcte)                 begin bt_ph <= 1'b1; cnt <= 16'd0; end
`endif
                            else                           cnt <= inc;
                        end else if (st_hw) begin
                            if (hcap) begin bt_ph <= 1'b1; hw <= 1'b0; cnt <= 16'd0; end
                        end else begin
                            if (~eob)              cnt <= inc;
                            else if (launch_req_i) begin bt_ph <= 1'b0; lr <= 1'b0; cnt <= 16'd0; end
                            else if (fld_next_i)   lr <= 1'b1;
                            else                   begin bt_run <= 1'b0; lr <= 1'b0; end
                        end
                    end
                endcase
                // sample countdown
                if (sset)      spend <= 1'b1;
                else if (samp) spend <= 1'b0;
                if (sarmev) begin
                    sarm  <= refimm;
                    rwait <= ~refimm;
                    sdn   <= SLOAD;
                end else begin
                    if (rwait & dcap_i) begin
                        rwait <= 1'b0; sarm <= 1'b1; sdn <= SLOAD;
                    end else if (sarm) begin
                        if (sdn == 3'd0) sarm <= 1'b0;
                        else sdn <= sdn - 3'd1;
                    end
                end
            end
        end
    end
endmodule
