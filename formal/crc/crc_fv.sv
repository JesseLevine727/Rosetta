`default_nettype none
// ============================================================================
//  Proof top for rosetta_crc. Every port of this module is a free input
//  (chosen by the solver each cycle); the only assumption is the initial reset
//  inside rosetta_crc_props.
//
//    u_dut   (copy A) fully free inputs; checked by rosetta_crc_props
//                     (update rule, upper bits, init/hold/reset, value_o,
//                     known answers)
//    u_dut_b (copy B) \  linearity (rosetta_crc_lin_props): share everything
//    u_dut_c (copy C) /  with A except
//                          bit:      B free,  C = A ^ B
//                          cfg_init: B free,  C = A ^ B
// ============================================================================
module crc_fv (
    input wire        clk,
    input wire        rst_n,
    input wire [31:0] cfg_poly_i,
    input wire [31:0] cfg_init_i,
    input wire [4:0]  cfg_width_m1_i,
    input wire        cfg_xorout_i,
    input wire        init_i,
    input wire        en_i,
    input wire        bit_i,
    // second copy's free data and initial value
    input wire [31:0] cfg_init_b,
    input wire        bit_b
);
    wire [31:0] state_a, value_a, state_b, value_b, state_c, value_c;

    rosetta_crc u_dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_poly_i(cfg_poly_i), .cfg_init_i(cfg_init_i),
        .cfg_width_m1_i(cfg_width_m1_i), .cfg_xorout_i(cfg_xorout_i),
        .init_i(init_i), .en_i(en_i), .bit_i(bit_i),
        .state_o(state_a), .value_o(value_a)
    );

    rosetta_crc_props u_props (
        .clk(clk), .rst_n(rst_n),
        .cfg_poly_i(cfg_poly_i), .cfg_init_i(cfg_init_i),
        .cfg_width_m1_i(cfg_width_m1_i), .cfg_xorout_i(cfg_xorout_i),
        .init_i(init_i), .en_i(en_i), .bit_i(bit_i),
        .state_o(state_a), .value_o(value_a)
    );

    rosetta_crc u_dut_b (
        .clk(clk), .rst_n(rst_n),
        .cfg_poly_i(cfg_poly_i), .cfg_init_i(cfg_init_b),
        .cfg_width_m1_i(cfg_width_m1_i), .cfg_xorout_i(cfg_xorout_i),
        .init_i(init_i), .en_i(en_i), .bit_i(bit_b),
        .state_o(state_b), .value_o(value_b)
    );

    rosetta_crc u_dut_c (
        .clk(clk), .rst_n(rst_n),
        .cfg_poly_i(cfg_poly_i), .cfg_init_i(cfg_init_i ^ cfg_init_b),
        .cfg_width_m1_i(cfg_width_m1_i), .cfg_xorout_i(cfg_xorout_i),
        .init_i(init_i), .en_i(en_i), .bit_i(bit_i ^ bit_b),
        .state_o(state_c), .value_o(value_c)
    );

    rosetta_crc_lin_props u_lin (
        .clk(clk), .rst_n(rst_n),
        .cfg_width_m1_i(cfg_width_m1_i), .cfg_xorout_i(cfg_xorout_i),
        .init_i(init_i), .en_i(en_i),
        .cfg_init_a(cfg_init_i), .cfg_init_b(cfg_init_b),
        .bit_a(bit_i), .bit_b(bit_b),
        .state_a(state_a), .state_b(state_b), .state_c(state_c),
        .value_a(value_a), .value_b(value_b), .value_c(value_c)
    );
endmodule

`default_nettype wire
