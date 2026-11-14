`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/coding.md (with docs/decisions.md
//  rulings 20, 21, 27, 46-48) written by the formal property writer, only to
//  show that formal/coding/*.sv can PASS and that the properties are
//  non-vacuous (each planted bug below makes a task FAIL). The real block is
//  src/rosetta_coding.sv, written independently. Do not integrate,
//  synthesise or copy this file.
//
//  Planted bugs (mutation testing), enabled one at a time with a define:
//    MUT_NRZI_POL        NRZI transmit polarity: a 1 changes, a 0 holds
//    MUT_MAN_ORDER       Manchester half-bit order swapped (x first, !x second)
//    MUT_INV_TWICE       invert applied twice on transmit (level coding net
//                        effect: no invert)
//    MUT_OD_HIGH         open-drain ignored on out0: drives high for a 1
//    MUT_DIF_NOT_COMPL   differential out1 carries the same level as out0
//    MUT_DFI_VALID       invalid differential pair still reports rxv = 1
//    MUT_NRZI_PIN_LOST   a PIN write of out0 does not update the NRZI level
//    MUT_PW_OFF1         pulse width: the low change comes one cycle after
//                        hte (high time H + 1)
//    MUT_MPH_NO_WR       a CODE/TICK write does not reset the Manchester phase
//    MUT_DL_HALT_RST     dl_q forced to 0 while halted (spurious edge at start)
//    MUT_NZ_NRZI_ONLY    nz_q follows samples only when receiving NRZI
//    MUT_ABORT_KEEPS_HP  abort does not cancel a pending second change
//    MUT_REL_NO_DLY      a release request is not subject to the launch delay
//    MUT_PS_RISE         pulse symbol: a rising edge carries the bit
//    MUT_PW_ZERO_PULSE   pulse width with a zero high time still pulses
//    MUT_TL_NO_RESET     tl_q has no reset (arbitrary power-up value)
//    MUT_REL_SETS_TL     a release updates tl_q (ruling 47: it must not)
//    MUT_WR_LOSES_MPH    a Manchester sample beats a CODE write (ruling 47)
//    MUT_DREQ_ABORT      dreq gated by abort as well (ruling 48: run only)
// ===========================================================================
module rosetta_coding (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run_i,
    input  wire        abort_i,
    input  wire        wr_code_i,
    input  wire        wr_tick_i,
    input  wire  [1:0] cfg_mode_i,
    input  wire  [1:0] cfg_rx_i,
    input  wire  [1:0] cfg_tx_i,
    input  wire  [1:0] cfg_din_i,
    input  wire        cfg_inv_i,
    input  wire        cfg_od_i,
    input  wire        cfg_dif_i,
    input  wire        cfg_dfi_i,
    input  wire  [7:0] cfg_pw0_i,
    input  wire  [7:0] cfg_pw1_i,
    input  wire  [3:0] in_i,
    input  wire        st_i,
    input  wire        cte_i,
    input  wire        hte_i,
    input  wire        ge0_i,
    input  wire        tx_go_i,
    input  wire        tx_bit_i,
    input  wire        tx_rel_i,
    input  wire        raw_go_i,
    input  wire        raw_lvl_i,
    input  wire        pin_we_i,
    input  wire        pin_mask0_i,
    input  wire        pin_val0_i,
    input  wire        pin_oe0_i,
    input  wire  [1:0] out_val_i,
    output logic       dline_o,
    output logic       dedge_o,
    output logic       rxb_o,
    output logic       rxv_o,
    output logic       pw_bit_o,
    output logic       dreq_o,
    output logic [4:0] dpay_o,
    output logic       dreq_dly_o
);
    logic dl_q, nz_q, mph, tl_q, hp, hv, pwb;
`ifdef MUT_PW_OFF1
    logic hte_q;
`endif

    // ---------------- receive ----------------
    wire P  = in_i[cfg_din_i];
    wire N  = in_i[cfg_din_i ^ 2'd1];
    wire ok = !cfg_dfi_i || (P != N);
    wire dline = ok ? P : dl_q;
    wire lvl   = dline ^ cfg_inv_i;
    wire pm    = (cfg_mode_i == 2'd3);

    always_comb begin
        dline_o = dline;
        dedge_o = ok && (P != dl_q);
        if (cfg_rx_i == 2'd1) begin
            rxb_o = (lvl == nz_q);
            rxv_o = 1'b1;
        end else if (pm && cfg_rx_i == 2'd2) begin
            rxb_o = lvl;
            rxv_o = ge0_i || !mph;
        end else if (pm && cfg_rx_i == 2'd3) begin
            rxb_o = ge0_i;
`ifdef MUT_PS_RISE
            rxv_o = (lvl == 1'b1);
`else
            rxv_o = (lvl == 1'b0);
`endif
        end else begin
            rxb_o = lvl;
            rxv_o = 1'b1;
        end
`ifndef MUT_DFI_VALID
        rxv_o = rxv_o && ok;
`endif
    end

    // ---------------- transmit ----------------
    function automatic [4:0] map_l(input logic l, input logic inv,
                                   input logic od, input logic dif);
        logic p, q;
        begin
            p = l ^ inv;
`ifdef MUT_DIF_NOT_COMPL
            q = p;
`else
            q = !p;
`endif
            map_l = {od ? 1'b0 : p, od ? !p : 1'b1, dif,
                     od ? 1'b0 : q, od ? !q : 1'b1};
`ifdef MUT_OD_HIGH
            map_l[4:3] = {p, 1'b1};
`endif
        end
    endfunction

    wire [7:0] hsel = tx_bit_i ? cfg_pw1_i : cfg_pw0_i;
`ifdef MUT_PW_ZERO_PULSE
    wire       hnz  = 1'b1;
`else
    wire       hnz  = (hsel != 8'd0);
`endif
    logic go_l;
    always_comb begin
        case (cfg_tx_i)
`ifdef MUT_INV_TWICE
            2'd0: go_l = tx_bit_i ^ cfg_inv_i;
`else
            2'd0: go_l = tx_bit_i;
`endif
`ifdef MUT_NRZI_POL
            2'd1: go_l = tx_bit_i ? !tl_q : tl_q;
`else
            2'd1: go_l = tx_bit_i ? tl_q : !tl_q;
`endif
`ifdef MUT_MAN_ORDER
            2'd2: go_l = tx_bit_i;
`else
            2'd2: go_l = !tx_bit_i;
`endif
            default: go_l = hnz;
        endcase
    end

`ifdef MUT_PW_OFF1
    wire sc_c = hp && ((cfg_tx_i == 2'd3) ? (hte_q || cte_i) : cte_i);
`else
    wire sc_c = hp && ((cfg_tx_i == 2'd3) ? (hte_i || cte_i) : cte_i);
`endif
    wire r_go  = run_i && tx_go_i;
    wire r_rel = run_i && !tx_go_i && tx_rel_i;
    wire r_raw = run_i && !tx_go_i && !tx_rel_i && raw_go_i;
    wire r_sc  = run_i && !tx_go_i && !tx_rel_i && !raw_go_i && sc_c;

    always_comb begin
        dreq_o = r_go || r_rel || r_raw || r_sc;
`ifdef MUT_DREQ_ABORT
        dreq_o = dreq_o && !abort_i;
`endif
`ifdef MUT_REL_NO_DLY
        dreq_dly_o = !(r_raw || r_rel);
`else
        dreq_dly_o = !r_raw;
`endif
        if (tx_go_i)
            dpay_o = map_l(go_l, cfg_inv_i, cfg_od_i, cfg_dif_i);
        else if (tx_rel_i)
            dpay_o = {out_val_i[0], 1'b0, cfg_dif_i, out_val_i[1], 1'b0};
        else if (raw_go_i)
            dpay_o = map_l(raw_lvl_i, cfg_inv_i, cfg_od_i, cfg_dif_i);
        else
            dpay_o = map_l(hv, cfg_inv_i, cfg_od_i, cfg_dif_i);
    end

    assign pw_bit_o = pwb;

    // ---------------- state ----------------
    wire wr = wr_code_i || wr_tick_i;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dl_q <= 1'b0;
        end else begin
`ifdef MUT_DL_HALT_RST
            dl_q <= run_i ? dline : 1'b0;
`else
            dl_q <= dline;
`endif
        end
    end

`ifdef MUT_TL_NO_RESET
    always_ff @(posedge clk) begin
        if (!run_i) tl_q <= 1'b0;
        else if (pin_we_i && pin_mask0_i
  `ifdef MUT_NRZI_PIN_LOST
                 && 1'b0
  `endif
                 ) tl_q <= (pin_oe0_i ? pin_val0_i : 1'b1) ^ cfg_inv_i;
        else if (r_go)  tl_q <= go_l;
        else if (r_raw) tl_q <= raw_lvl_i;
        else if (r_sc)  tl_q <= hv;
    end
`else
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) tl_q <= 1'b0;
        else if (!run_i) tl_q <= 1'b0;
        else if (pin_we_i && pin_mask0_i
  `ifdef MUT_NRZI_PIN_LOST
                 && 1'b0
  `endif
                 ) tl_q <= (pin_oe0_i ? pin_val0_i : 1'b1) ^ cfg_inv_i;
        else if (r_go)  tl_q <= go_l;
  `ifdef MUT_REL_SETS_TL
        else if (r_rel) tl_q <= out_val_i[0] ^ cfg_inv_i;
  `endif
        else if (r_raw) tl_q <= raw_lvl_i;
        else if (r_sc)  tl_q <= hv;
    end
`endif

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nz_q <= 1'b1;
            mph  <= 1'b1;
            hp   <= 1'b0;
            hv   <= 1'b0;
            pwb  <= 1'b0;
`ifdef MUT_PW_OFF1
            hte_q <= 1'b0;
`endif
        end else if (!run_i) begin
            nz_q <= 1'b1;
            mph  <= 1'b1;
            hp   <= 1'b0;
            hv   <= 1'b0;
            pwb  <= 1'b0;
`ifdef MUT_PW_OFF1
            hte_q <= 1'b0;
`endif
        end else begin
`ifdef MUT_PW_OFF1
            hte_q <= hte_i;
`endif
`ifdef MUT_NZ_NRZI_ONLY
            if (st_i && ok && cfg_rx_i == 2'd1) nz_q <= lvl;
`else
            if (st_i && ok) nz_q <= lvl;
`endif
`ifdef MUT_WR_LOSES_MPH
            if (st_i && pm && cfg_rx_i == 2'd2) mph <= ge0_i ? 1'b1 : !mph;
            else if (wr) mph <= 1'b1;
`elsif MUT_MPH_NO_WR
            if (st_i && pm && cfg_rx_i == 2'd2) mph <= ge0_i ? 1'b1 : !mph;
`else
            if (wr) mph <= 1'b1;
            else if (st_i && pm && cfg_rx_i == 2'd2) mph <= ge0_i ? 1'b1 : !mph;
`endif
            if (r_go) begin
                case (cfg_tx_i)
                    2'd2: begin hp <= 1'b1; hv <= tx_bit_i; end
                    2'd3: begin hp <= hnz;  hv <= 1'b0; pwb <= tx_bit_i; end
                    default: hp <= 1'b0;
                endcase
            end else if (r_rel || r_raw || r_sc) begin
                hp <= 1'b0;
            end
`ifdef MUT_ABORT_KEEPS_HP
            if (wr) hp <= 1'b0;
`else
            if (abort_i || wr) hp <= 1'b0;
`endif
        end
    end
endmodule
