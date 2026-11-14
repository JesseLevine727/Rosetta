`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/events.md written by the formal
//  property writer, only to show that formal/events/*.sv can PASS and that
//  the properties are non-vacuous (the planted bugs below make tasks FAIL).
//  The real block is src/rosetta_events.sv, written independently.
//  Do not integrate, synthesise or copy this file.
//
//  Planted bugs (mutation testing), enabled one at a time with a define:
//    MUT_K_OFF_BY_ONE     a window is loaded with K+1: fires one prescaled
//                         cycle late (K = 63 never fires)
//    MUT_EDGE_NO_RESTART  a trigger while the window is open does not
//                         restart it (window not restarted by a level change)
//    MUT_LVL_NO_CANCEL    a level window is not closed when cond drops
//    MUT_LVL_REFIRE       done is never set: a level trigger refires (every
//                         cycle of the episode with K = 0, every K strobes)
//    MUT_EDGE_POL         trigger 2 fires on the edge to !L (polarity)
//    MUT_B_IGNORED        the B qualifier is ignored (bok = 1, bm = 3 too)
//    MUT_T3_NO_LV         trigger 3 holds on bok only (lv not compared)
//    MUT_T4_USES_BM       trigger 4 honours bm = 3 (never fires)
//    MUT_PT_AT_TRIGGER    a prescale strobe in the trigger cycle is counted
//    MUT_WR_NO_RESTART    a configuration write does not clear win/done
//    MUT_UNDEF_FIRES      trigger code 5 acts as trigger 1 (decodes [4:3])
//    MUT_CLR_WINS         a clear beats a fire in the same cycle
//    MUT_VIOL_WRONG_EV    the stuffing violation raises event 2, not 3
//    MUT_VIOL_NO_CFG      the stuffing violation raises event 3 even with
//                         cfg_stuffev_i = 0
//    MUT_PEND_NO_RESET    the pending flops have no reset
//    MUT_HALT_KEEPS_PEND  pending bits are not cleared while halted
//    MUT_FIRE_LATE        every fire one cycle late (fire_o and pending)
//    MUT_CROSS_CFG        detector 1 takes K from EVC0 (independence)
// ===========================================================================
module rosetta_events (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run_i,
    input  wire [3:0]  in_i,
    input  wire [3:0]  in_q_i,
    input  wire        pt_i,
    input  wire [63:0] cfg_evc_i,
    input  wire [3:0]  evc_wr_i,
    input  wire        cfg_stuffev_i,
    input  wire        stuff_viol_i,
    input  wire [3:0]  ev_clr_i,
    output logic [3:0] ev_pend_o,
    output logic [3:0] fire_o
);
    logic [3:0]  win, done, lv;
    logic [23:0] cnt;
    logic [3:0]  win_d, done_d, lv_d;
    logic [23:0] cnt_d;
    logic [3:0]  fire;

    logic [15:0] c;
    logic [1:0]  fa, fb, fbm;
    logic        fl;
    logic [2:0]  ftrig;
    logic [5:0]  fk;
    logic        a, aq, b, bok, edge_t, lvl_t, trg, hold, cnd;
    logic [5:0]  cur;
    integer      k;

    always_comb begin
        win_d  = win;
        done_d = done;
        lv_d   = lv;
        cnt_d  = cnt;
        fire   = 4'h0;
        for (k = 0; k < 4; k = k + 1) begin
            c     = cfg_evc_i[16*k +: 16];
            fa    = c[1:0];
            fl    = c[2];
            ftrig = c[5:3];
            fb    = c[7:6];
            fbm   = c[9:8];
            fk    = c[15:10];
`ifdef MUT_CROSS_CFG
            if (k == 1) fk = cfg_evc_i[15:10];
`endif
`ifdef MUT_UNDEF_FIRES
            if (ftrig == 3'd5) ftrig = 3'd1;
`endif
            a   = in_i[fa];
            aq  = in_q_i[fa];
            b   = in_i[fb];
            bok = (fbm == 2'd0) || (fbm == 2'd1 && b) || (fbm == 2'd2 && !b);
`ifdef MUT_B_IGNORED
            bok = 1'b1;
`endif
            edge_t = (ftrig == 3'd2) || (ftrig == 3'd3);
            lvl_t  = (ftrig == 3'd1) || (ftrig == 3'd4);
`ifdef MUT_EDGE_POL
            trg = edge_t && (a != aq) && bok && ((ftrig == 3'd3) || (a != fl));
`else
            trg = edge_t && (a != aq) && bok && ((ftrig == 3'd3) || (a == fl));
`endif
`ifdef MUT_T3_NO_LV
            hold = ((ftrig == 3'd3) ? 1'b1 : (a == fl)) && bok;
`else
            hold = ((ftrig == 3'd3) ? (a == lv[k]) : (a == fl)) && bok;
`endif
`ifdef MUT_T4_USES_BM
            cnd  = (ftrig == 3'd1) ? ((a == fl) && bok) : (!a && !b && fbm != 2'd3);
`else
            cnd  = (ftrig == 3'd1) ? ((a == fl) && bok) : (!a && !b);
`endif
            cur  = cnt[6*k +: 6];
            if (!run_i) begin
                win_d[k]  = 1'b0;
                done_d[k] = 1'b0;
                lv_d[k]   = 1'b0;
                cnt_d[6*k +: 6] = 6'd0;
            end else begin
                if (edge_t) begin
`ifdef MUT_EDGE_NO_RESTART
                    if (trg && !win[k]) begin
`else
                    if (trg) begin
`endif
                        lv_d[k] = a;
                        if (fk == 6'd0) begin
                            fire[k]  = 1'b1;
                            win_d[k] = 1'b0;
                        end else begin
                            win_d[k] = 1'b1;
`ifdef MUT_K_OFF_BY_ONE
                            cnt_d[6*k +: 6] = fk + 6'd1;
`elsif MUT_PT_AT_TRIGGER
                            cnt_d[6*k +: 6] = (pt_i && fk != 6'd1) ? fk - 6'd1 : fk;
`else
                            cnt_d[6*k +: 6] = fk;
`endif
                        end
                    end else if (win[k]) begin
                        if (!hold) begin
                            win_d[k] = 1'b0;
                        end else if (pt_i) begin
                            if (cur == 6'd1) begin
                                fire[k]  = 1'b1;
                                win_d[k] = 1'b0;
                            end else begin
                                cnt_d[6*k +: 6] = cur - 6'd1;
                            end
                        end
                    end
                end else if (lvl_t) begin
                    if (!cnd) begin
`ifndef MUT_LVL_NO_CANCEL
                        win_d[k]  = 1'b0;
`endif
                        done_d[k] = 1'b0;
                    end else if (!win[k] && !done[k]) begin
                        if (fk == 6'd0) begin
                            fire[k]   = 1'b1;
`ifndef MUT_LVL_REFIRE
                            done_d[k] = 1'b1;
`endif
                        end else begin
                            win_d[k] = 1'b1;
`ifdef MUT_K_OFF_BY_ONE
                            cnt_d[6*k +: 6] = fk + 6'd1;
`elsif MUT_PT_AT_TRIGGER
                            cnt_d[6*k +: 6] = (pt_i && fk != 6'd1) ? fk - 6'd1 : fk;
`else
                            cnt_d[6*k +: 6] = fk;
`endif
                        end
                    end else if (win[k] && pt_i) begin
                        if (cur == 6'd1) begin
                            fire[k]   = 1'b1;
                            win_d[k]  = 1'b0;
`ifndef MUT_LVL_REFIRE
                            done_d[k] = 1'b1;
`endif
                        end else begin
                            cnt_d[6*k +: 6] = cur - 6'd1;
                        end
                    end
                end
`ifndef MUT_WR_NO_RESTART
                if (evc_wr_i[k]) begin
                    win_d[k]  = 1'b0;
                    done_d[k] = 1'b0;
                end
`endif
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            win  <= 4'h0;
            done <= 4'h0;
            lv   <= 4'h0;
            cnt  <= 24'h0;
        end else begin
            win  <= win_d;
            done <= done_d;
            lv   <= lv_d;
            cnt  <= cnt_d;
        end
    end

    // fire -> fire_o (combinational, or one cycle late in MUT_FIRE_LATE)
    logic [3:0] fire_late;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) fire_late <= 4'h0;
        else        fire_late <= fire;
    end
`ifdef MUT_FIRE_LATE
    assign fire_o = fire_late;
`else
    assign fire_o = fire;
`endif

    // pending bits
    wire viol = stuff_viol_i && cfg_stuffev_i;
    logic [3:0] f;
    always_comb begin
        f = fire_o;
`ifdef MUT_VIOL_WRONG_EV
        f[2] = f[2] | viol;
`elsif MUT_VIOL_NO_CFG
        f[3] = f[3] | stuff_viol_i;
`else
        f[3] = f[3] | viol;
`endif
    end

    logic [3:0] pend_d;
`ifdef MUT_CLR_WINS
    assign pend_d = (f | ev_pend_o) & ~ev_clr_i;
`else
    assign pend_d = f | (ev_pend_o & ~ev_clr_i);
`endif

`ifdef MUT_PEND_NO_RESET
    always_ff @(posedge clk) begin
        ev_pend_o <= run_i ? pend_d : 4'h0;
    end
`elsif MUT_HALT_KEEPS_PEND
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) ev_pend_o <= 4'h0;
        else        ev_pend_o <= pend_d;
    end
`else
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) ev_pend_o <= 4'h0;
        else        ev_pend_o <= run_i ? pend_d : 4'h0;
    end
`endif
endmodule
