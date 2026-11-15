`default_nettype none
// ===========================================================================
//  Proof top for rosetta_cfg (lane.md section 5: u_cfg / rosetta_lane_cfg).
//
//   u_a   the block with every input free (rst_n low in cycle 0, free after;
//         flops start at arbitrary values), checked by cfg_props.
//   u_b   (unless FV_NO_BUS) a second copy for lane.md property 3, the
//         shared-bus lemma: same clk, rst_n, run_i, cfg_we_i as u_a;
//         cfg_reg_i / cfg_hi_i / cfg_data_i equal to u_a's in cycles with
//         cfg_we_i = 1 and independently free otherwise. cfg_bus_props
//         asserts that every output of the two copies is equal in every
//         cycle.
// ===========================================================================

module cfg_fv (
    input wire         clk,
    input wire         rst_n,
    input wire         run,
    input wire         we,
    input wire [4:0]   reg_a,
    input wire         hi_a,
    input wire [15:0]  data_a,
    // u_b's shared-bus values in cycles with we = 0
    input wire [4:0]   reg_b,
    input wire         hi_b,
    input wire [15:0]  data_b
);
    // ---------------------------------------------------------------- u_a
    wire [12:0] a_tick;
    wire [15:0] a_period, a_period2, a_pulse;
    wire [7:0]  a_thr0, a_dly;
    wire [11:0] a_code;
    wire [5:0]  a_stuff;
    wire [31:0] a_crcpoly;
    wire [6:0]  a_crccfg;
    wire [63:0] a_evc;
    wire        a_wr_tick, a_wr_code, a_wr_stuff, a_wr_crc;
    wire [3:0]  a_wr_evc;
    rosetta_cfg u_a (
        .clk(clk), .rst_n(rst_n), .run_i(run),
        .cfg_we_i(we), .cfg_reg_i(reg_a), .cfg_hi_i(hi_a), .cfg_data_i(data_a),
        .tick_o(a_tick), .period_o(a_period), .period2_o(a_period2),
        .thr0_o(a_thr0), .dly_o(a_dly), .code_o(a_code), .pulse_o(a_pulse),
        .stuff_o(a_stuff), .crcpoly_o(a_crcpoly), .crccfg_o(a_crccfg),
        .evc_o(a_evc),
        .wr_tick_o(a_wr_tick), .wr_code_o(a_wr_code), .wr_stuff_o(a_wr_stuff),
        .wr_crc_o(a_wr_crc), .wr_evc_o(a_wr_evc)
    );

    cfg_props #(.STANDALONE(1)) u_a_props (
        .clk(clk), .rst_n(rst_n), .run_i(run),
        .cfg_we_i(we), .cfg_reg_i(reg_a), .cfg_hi_i(hi_a), .cfg_data_i(data_a),
        .tick_o(a_tick), .period_o(a_period), .period2_o(a_period2),
        .thr0_o(a_thr0), .dly_o(a_dly), .code_o(a_code), .pulse_o(a_pulse),
        .stuff_o(a_stuff), .crcpoly_o(a_crcpoly), .crccfg_o(a_crccfg),
        .evc_o(a_evc),
        .wr_tick_o(a_wr_tick), .wr_code_o(a_wr_code), .wr_stuff_o(a_wr_stuff),
        .wr_crc_o(a_wr_crc), .wr_evc_o(a_wr_evc)
    );

`ifndef FV_NO_BUS
    // ---------------------------------------------------------------- u_b
    wire [4:0]  b_reg  = we ? reg_a  : reg_b;
    wire        b_hi   = we ? hi_a   : hi_b;
    wire [15:0] b_data = we ? data_a : data_b;
    wire [12:0] b_tick;
    wire [15:0] b_period, b_period2, b_pulse;
    wire [7:0]  b_thr0, b_dly;
    wire [11:0] b_code;
    wire [5:0]  b_stuff;
    wire [31:0] b_crcpoly;
    wire [6:0]  b_crccfg;
    wire [63:0] b_evc;
    wire        b_wr_tick, b_wr_code, b_wr_stuff, b_wr_crc;
    wire [3:0]  b_wr_evc;
    rosetta_cfg u_b (
        .clk(clk), .rst_n(rst_n), .run_i(run),
        .cfg_we_i(we), .cfg_reg_i(b_reg), .cfg_hi_i(b_hi), .cfg_data_i(b_data),
        .tick_o(b_tick), .period_o(b_period), .period2_o(b_period2),
        .thr0_o(b_thr0), .dly_o(b_dly), .code_o(b_code), .pulse_o(b_pulse),
        .stuff_o(b_stuff), .crcpoly_o(b_crcpoly), .crccfg_o(b_crccfg),
        .evc_o(b_evc),
        .wr_tick_o(b_wr_tick), .wr_code_o(b_wr_code), .wr_stuff_o(b_wr_stuff),
        .wr_crc_o(b_wr_crc), .wr_evc_o(b_wr_evc)
    );

    cfg_bus_props u_bus_props (
        .a_obs({a_evc, a_crccfg, a_crcpoly, a_stuff, a_pulse, a_code, a_dly,
                a_thr0, a_period2, a_period, a_tick}),
        .a_pulse({a_wr_evc, a_wr_crc, a_wr_stuff, a_wr_code, a_wr_tick}),
        .b_obs({b_evc, b_crccfg, b_crcpoly, b_stuff, b_pulse, b_code, b_dly,
                b_thr0, b_period2, b_period, b_tick}),
        .b_pulse({b_wr_evc, b_wr_crc, b_wr_stuff, b_wr_code, b_wr_tick})
    );
`endif
endmodule
