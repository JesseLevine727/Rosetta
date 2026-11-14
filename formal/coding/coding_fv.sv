`default_nettype none
// ===========================================================================
//  Proof top for rosetta_coding.
//
//   u_dut + u_props   one rosetta_coding with every input free (rst_n low in
//                     cycle 0, then free; flops start at arbitrary values).
//                     coding_props checks every output against the exact
//                     spec model in every cycle, plus the port-level
//                     statements of coding.md 5 and covers. ENV_SEQ = 1:
//                     the section 5 assumptions (set FV_ENV_FREE to prove
//                     with free inputs, ENV_SEQ = 0; FV_NO_UNIT drops the
//                     checker, leaving only the round trips).
//   u_rt_ln           round trip, level and NRZI (two more instances)
//   u_rt_man          round trip, Manchester    (two more instances)
//   u_rt_pw           round trip, pulse width   (two more instances)
//                     (coding_rt.sv; `define FV_NO_RT drops all three,
//                     FV_NO_LN / FV_NO_MAN / FV_NO_PW one of them)
//
//  Spec: docs/blocks/coding.md as amended by docs/decisions.md rulings 20,
//  21, 27, 46, 47, 48.
// ===========================================================================

module coding_fv (
    input wire       clk,
    input wire       rst_n,
    input wire       run_i,
    input wire       abort_i,
    input wire       wr_code_i,
    input wire       wr_tick_i,
    input wire [1:0] cfg_mode_i,
    input wire [1:0] cfg_rx_i,
    input wire [1:0] cfg_tx_i,
    input wire [1:0] cfg_din_i,
    input wire       cfg_inv_i,
    input wire       cfg_od_i,
    input wire       cfg_dif_i,
    input wire       cfg_dfi_i,
    input wire [7:0] cfg_pw0_i,
    input wire [7:0] cfg_pw1_i,
    input wire [3:0] in_i,
    input wire       st_i,
    input wire       cte_i,
    input wire       hte_i,
    input wire       ge0_i,
    input wire       tx_go_i,
    input wire       tx_bit_i,
    input wire       tx_rel_i,
    input wire       raw_go_i,
    input wire       raw_lvl_i,
    input wire       pin_we_i,
    input wire       pin_mask0_i,
    input wire       pin_val0_i,
    input wire       pin_oe0_i,
    input wire [1:0] out_val_i
);
    wire       dline_o, dedge_o, rxb_o, rxv_o, pw_bit_o, dreq_o, dreq_dly_o;
    wire [4:0] dpay_o;

    rosetta_coding u_dut (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(abort_i),
        .wr_code_i(wr_code_i), .wr_tick_i(wr_tick_i),
        .cfg_mode_i(cfg_mode_i), .cfg_rx_i(cfg_rx_i), .cfg_tx_i(cfg_tx_i), .cfg_din_i(cfg_din_i),
        .cfg_inv_i(cfg_inv_i), .cfg_od_i(cfg_od_i), .cfg_dif_i(cfg_dif_i), .cfg_dfi_i(cfg_dfi_i),
        .cfg_pw0_i(cfg_pw0_i), .cfg_pw1_i(cfg_pw1_i),
        .in_i(in_i), .st_i(st_i), .cte_i(cte_i), .hte_i(hte_i), .ge0_i(ge0_i),
        .tx_go_i(tx_go_i), .tx_bit_i(tx_bit_i), .tx_rel_i(tx_rel_i),
        .raw_go_i(raw_go_i), .raw_lvl_i(raw_lvl_i),
        .pin_we_i(pin_we_i), .pin_mask0_i(pin_mask0_i), .pin_val0_i(pin_val0_i), .pin_oe0_i(pin_oe0_i),
        .out_val_i(out_val_i),
        .dline_o(dline_o), .dedge_o(dedge_o), .rxb_o(rxb_o), .rxv_o(rxv_o),
        .pw_bit_o(pw_bit_o), .dreq_o(dreq_o), .dpay_o(dpay_o), .dreq_dly_o(dreq_dly_o)
    );

`ifdef FV_ENV_FREE
    localparam ENV = 0;
`else
    localparam ENV = 1;
`endif
`ifndef FV_NO_UNIT
    coding_props #(.STANDALONE(1), .ENV_SEQ(ENV)) u_props (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(abort_i),
        .wr_code_i(wr_code_i), .wr_tick_i(wr_tick_i),
        .cfg_mode_i(cfg_mode_i), .cfg_rx_i(cfg_rx_i), .cfg_tx_i(cfg_tx_i), .cfg_din_i(cfg_din_i),
        .cfg_inv_i(cfg_inv_i), .cfg_od_i(cfg_od_i), .cfg_dif_i(cfg_dif_i), .cfg_dfi_i(cfg_dfi_i),
        .cfg_pw0_i(cfg_pw0_i), .cfg_pw1_i(cfg_pw1_i),
        .in_i(in_i), .st_i(st_i), .cte_i(cte_i), .hte_i(hte_i), .ge0_i(ge0_i),
        .tx_go_i(tx_go_i), .tx_bit_i(tx_bit_i), .tx_rel_i(tx_rel_i),
        .raw_go_i(raw_go_i), .raw_lvl_i(raw_lvl_i),
        .pin_we_i(pin_we_i), .pin_mask0_i(pin_mask0_i), .pin_val0_i(pin_val0_i), .pin_oe0_i(pin_oe0_i),
        .out_val_i(out_val_i),
        .dline_o(dline_o), .dedge_o(dedge_o), .rxb_o(rxb_o), .rxv_o(rxv_o),
        .pw_bit_o(pw_bit_o), .dreq_o(dreq_o), .dpay_o(dpay_o), .dreq_dly_o(dreq_dly_o)
    );
`endif

`ifndef FV_NO_RT
  `ifndef FV_NO_LN
    coding_rt #(.KIND(0)) u_rt_ln  (.clk(clk));
  `endif
  `ifndef FV_NO_MAN
    coding_rt #(.KIND(1)) u_rt_man (.clk(clk));
  `endif
  `ifndef FV_NO_PW
    coding_rt #(.KIND(2)) u_rt_pw  (.clk(clk));
  `endif
`endif
endmodule
