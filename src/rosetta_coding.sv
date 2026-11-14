`default_nettype none
// Line coding, receive and transmit (docs/blocks/coding.md; lane.md 6.3-6.5).
//
// Receive: data input select, differential-pair check (decisions.md ruling
// 20: an invalid pair holds dline and carries no bit, so rxv is gated by ok
// in every coding; ruling 27, EXCHANGE takes the sample anyway, is done in
// rosetta_field), level / NRZI / Manchester / pulse-symbol decode.
// Transmit: one request per cycle, priority wire bit > release > RAW DRIVE >
// second change (4.2), mapped by MAP (4.1).
//
// State (section 2, 7 flops): dl_q, nz_q, mph, tl_q, hp, hv, pwb. While
// run_i = 0 all but dl_q take their reset values. Same-cycle precedence where
// the spec gives none: abort/CODE/TICK writes win over the hp and mph
// updates of a request or a sample; a PIN write of out0 wins over a request
// for tl_q (as in rosetta_pins).
module rosetta_coding (
    input  wire        clk, rst_n,
    input  wire        run_i, abort_i,
    input  wire        wr_code_i, wr_tick_i,
    input  wire  [1:0] cfg_mode_i,
    input  wire  [1:0] cfg_rx_i, cfg_tx_i, cfg_din_i,
    input  wire        cfg_inv_i, cfg_od_i, cfg_dif_i, cfg_dfi_i,
    input  wire  [7:0] cfg_pw0_i, cfg_pw1_i,
    input  wire  [3:0] in_i,
    input  wire        st_i, cte_i, hte_i, ge0_i,
    input  wire        tx_go_i, tx_bit_i, tx_rel_i, raw_go_i, raw_lvl_i,
    input  wire        pin_we_i, pin_mask0_i, pin_val0_i, pin_oe0_i,
    input  wire  [1:0] out_val_i,
    output logic       dline_o, dedge_o,
    output logic       rxb_o, rxv_o,
    output logic       pw_bit_o,
    output logic       dreq_o,
    output logic [4:0] dpay_o,              // {v0, e0, w1, v1, e1}
    output logic       dreq_dly_o
);
  localparam logic [1:0] PULSE = 2'd3;

  logic dl_q, nz_q, mph, tl_q, hp, hv, pwb;
  logic p, n, ok, dline, lvl, pm, man, sc, rel, raw, lreq, lgo, l, pp, wr;

  // ---- 3.1 Line cleaning ---------------------------------------------------
  assign p     = in_i[cfg_din_i];
  assign n     = in_i[cfg_din_i ^ 2'd1];
  assign ok    = !cfg_dfi_i || (p != n);
  assign dline = ok ? p : dl_q;
  assign lvl   = dline ^ cfg_inv_i;
  assign dline_o = dline;
  assign dedge_o = ok && (p != dl_q);

  // ---- 3.2 Decoding (meaningful with st_i; not gated by it) ----------------
  assign pm  = cfg_mode_i == PULSE;
  assign man = pm && cfg_rx_i == 2'd2;
  always_comb begin
    if (cfg_rx_i == 2'd1)       begin rxb_o = lvl == nz_q; rxv_o = ok;                    end
    else if (man)               begin rxb_o = lvl;         rxv_o = ok && (ge0_i || !mph); end
    else if (pm && cfg_rx_i[0]) begin rxb_o = ge0_i;       rxv_o = ok && !lvl;            end
    else                        begin rxb_o = lvl;         rxv_o = ok;                    end
  end

  // ---- 4.2 Requests --------------------------------------------------------
  // rel/raw/sc: the request selected in priority order (only with run_i).
  assign rel = !tx_go_i && tx_rel_i;
  assign raw = !tx_go_i && !tx_rel_i && raw_go_i;
  assign sc  = !tx_go_i && !tx_rel_i && !raw_go_i && hp &&
               (cte_i || (cfg_tx_i == 2'd3 && hte_i));
  assign pp  = |(tx_bit_i ? cfg_pw1_i : cfg_pw0_i);         // H(x) != 0

  always_comb
    case (cfg_tx_i)
      2'd0:    lgo = tx_bit_i;
      2'd1:    lgo = !(tx_bit_i ^ tl_q);                     // x ? tl_q : !tl_q
      2'd2:    lgo = !tx_bit_i;
      default: lgo = pp;
    endcase
  assign l    = tx_go_i ? lgo : raw_go_i ? raw_lvl_i : hv;
  assign lreq = tx_go_i || raw || sc;                        // requests a level l

  // 4.1 MAP(l), or the release payload (values kept, enables cleared).
  always_comb begin
    if (rel)           dpay_o = {out_val_i[0], 1'b0, cfg_dif_i, out_val_i[1], 1'b0};
    else if (cfg_od_i) dpay_o = {1'b0, !(l ^ cfg_inv_i), cfg_dif_i, 1'b0, l ^ cfg_inv_i};
    else               dpay_o = {l ^ cfg_inv_i, 1'b1, cfg_dif_i, !(l ^ cfg_inv_i), 1'b1};
  end
  assign dreq_o     = run_i && (tx_go_i || tx_rel_i || raw_go_i || sc);
  assign dreq_dly_o = !(run_i && raw);                       // 0 exactly for RAW DRIVE
  assign pw_bit_o   = pwb;

  // ---- State ---------------------------------------------------------------
  assign wr = wr_code_i || wr_tick_i;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      dl_q <= 1'b0; nz_q <= 1'b1; mph <= 1'b1;
      tl_q <= 1'b0; hp <= 1'b0; hv <= 1'b0; pwb <= 1'b0;
    end else begin
      dl_q <= dline;
      if (!run_i) begin
        nz_q <= 1'b1; mph <= 1'b1;
        tl_q <= 1'b0; hp <= 1'b0; hv <= 1'b0; pwb <= 1'b0;
      end else begin
        if (st_i && ok) nz_q <= lvl;
        if (wr)                mph <= 1'b1;
        else if (st_i && man)  mph <= ge0_i || !mph;
        if (pin_we_i && pin_mask0_i) tl_q <= (!pin_oe0_i || pin_val0_i) ^ cfg_inv_i;
        else if (lreq)               tl_q <= l;
        if (abort_i || wr)     hp <= 1'b0;
        else if (tx_go_i)      hp <= cfg_tx_i == 2'd2 || (cfg_tx_i == 2'd3 && pp);
        else if (rel || raw || sc) hp <= 1'b0;
        if (tx_go_i && cfg_tx_i[1]) hv  <= !cfg_tx_i[0] && tx_bit_i;   // 2: x, 3: 0
        if (tx_go_i && cfg_tx_i == 2'd3) pwb <= tx_bit_i;
      end
    end
endmodule
