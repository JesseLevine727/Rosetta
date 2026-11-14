`default_nettype none
// ===========================================================================
//  Proof top for rosetta_pins.
//
//  One rosetta_pins with every input free, checked by pins_props (exact
//  behaviour of both outputs in every cycle, the required properties of
//  pins.md section 5 on the ports, section 4 enabling rules, covers).
//  rst_n is low in the first cycle (assumed in pins_props) and free after.
//  Flops start at arbitrary values (no initial values anywhere).
//
//  Spec: docs/blocks/pins.md (field.md 7.1 for when pin_we_i is raised).
// ===========================================================================

module pins_fv (
    input wire       clk,
    input wire       rst_n,
    input wire       run_i,
    input wire       d_we_i,
    input wire [4:0] d_pay_i,
    input wire       c_we_i,
    input wire [1:0] c_pay_i,
    input wire       pin_we_i,
    input wire [3:0] pin_mask_i,
    input wire [3:0] pin_val_i,
    input wire [3:0] pin_oe_i
);
    wire [3:0] out_val_o, out_oe_o;

    rosetta_pins u_dut (
        .clk(clk), .rst_n(rst_n), .run_i(run_i),
        .d_we_i(d_we_i), .d_pay_i(d_pay_i), .c_we_i(c_we_i), .c_pay_i(c_pay_i),
        .pin_we_i(pin_we_i), .pin_mask_i(pin_mask_i), .pin_val_i(pin_val_i), .pin_oe_i(pin_oe_i),
        .out_val_o(out_val_o), .out_oe_o(out_oe_o)
    );

    pins_props #(.STANDALONE(1)) u_props (
        .clk(clk), .rst_n(rst_n), .run_i(run_i),
        .d_we_i(d_we_i), .d_pay_i(d_pay_i), .c_we_i(c_we_i), .c_pay_i(c_pay_i),
        .pin_we_i(pin_we_i), .pin_mask_i(pin_mask_i), .pin_val_i(pin_val_i), .pin_oe_i(pin_oe_i),
        .out_val_o(out_val_o), .out_oe_o(out_oe_o)
    );
endmodule
