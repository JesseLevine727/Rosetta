`default_nettype none
// ===========================================================================
//  Proof top for rosetta_delay (tick.md section 9).
//
//  One rosetta_delay with every input free, checked by delay_props (exact
//  model of 9.2 on every output in every cycle, required properties 1-5 of
//  tick.md 10 on the ports, covers). rst_n is low in the first cycle
//  (assumed in delay_props) and free after. Flops start at arbitrary values.
// ===========================================================================
module delay_fv (
    input wire       clk,
    input wire       rst_n,
    input wire       run_i,
    input wire       abort_i,
    input wire       wr_tick_i,
    input wire [7:0] cfg_dly_i,
    input wire       cfg_dlyclk_i,
    input wire       cfg_dlyps_i,
    input wire       pt_i,
    input wire       dreq_i,
    input wire [4:0] dpay_i,
    input wire       dreq_dly_i,
    input wire       creq_i,
    input wire [1:0] cpay_i,
    input wire       creq_dly_i,
    input wire       creq_cap_i,
    input wire       pin_we_i,
    input wire [3:0] pin_mask_i
);
    wire       d_we_o, c_we_o, dcap_o;
    wire [4:0] d_pay_o;
    wire [1:0] c_pay_o;

`define DELAY_PORTS \
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(abort_i), .wr_tick_i(wr_tick_i), \
        .cfg_dly_i(cfg_dly_i), .cfg_dlyclk_i(cfg_dlyclk_i), .cfg_dlyps_i(cfg_dlyps_i), \
        .pt_i(pt_i), .dreq_i(dreq_i), .dpay_i(dpay_i), .dreq_dly_i(dreq_dly_i), \
        .creq_i(creq_i), .cpay_i(cpay_i), .creq_dly_i(creq_dly_i), .creq_cap_i(creq_cap_i), \
        .pin_we_i(pin_we_i), .pin_mask_i(pin_mask_i), \
        .d_we_o(d_we_o), .d_pay_o(d_pay_o), .c_we_o(c_we_o), .c_pay_o(c_pay_o), .dcap_o(dcap_o)

    rosetta_delay u_dut (`DELAY_PORTS);
    delay_props #(.STANDALONE(1)) u_props (`DELAY_PORTS);
endmodule
