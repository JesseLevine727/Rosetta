`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/pins.md written by the formal
//  property writer, only to show that formal/pins/*.sv can PASS and that the
//  properties are non-vacuous (each planted bug below makes a task FAIL).
//  The real block is src/rosetta_pins.sv, written independently. Do not
//  integrate, synthesise or copy this file.
//
//  Planted bugs (mutation testing), enabled one at a time with a define:
//    MUT_HALT_KEEPS_OE    enables not cleared on halt (row 1 missing; the
//                         run_i gate still hides them while halted)
//    MUT_DATA_BEATS_PIN   a data request beats a PIN on out0/out1
//    MUT_CLK_BEATS_PIN    a clock request overrides a PIN on out3
//    MUT_PIN_BYPASS       a PIN's value shows on out_val_o in its own cycle
//                         (takes effect "immediately" instead of at the end
//                         of the cycle)
//    MUT_PIN_LATE         a PIN takes effect one cycle late
//    MUT_W1_IGNORED       a data request writes out1 even without w1 (a
//                         non-differential request can enable out1)
//    MUT_PIN_OE_NOMASK    a PIN writes the enables of unselected outputs
//    MUT_CLK_OUT2         a clock request also writes out2
//    MUT_REL_KEEPS_VAL    a data request with e0 = 0 (release) keeps the old
//                         out0 value instead of taking v0
//    MUT_HALT_CLEARS_VAL  halt clears the values as well
//    MUT_NO_GATE          out_oe_o not gated by run_i
//    MUT_OE_NO_RESET      enable flops without reset
// ===========================================================================
module rosetta_pins (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run_i,
    input  wire        d_we_i,
    input  wire  [4:0] d_pay_i,
    input  wire        c_we_i,
    input  wire  [1:0] c_pay_i,
    input  wire        pin_we_i,
    input  wire  [3:0] pin_mask_i,
    input  wire  [3:0] pin_val_i,
    input  wire  [3:0] pin_oe_i,
    output logic [3:0] out_val_o,
    output logic [3:0] out_oe_o
);
    logic [3:0] val, oe;

    // PIN as seen by the update logic
`ifdef MUT_PIN_LATE
    logic       pw_q;
    logic [3:0] pm_q, pv_q, po_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin pw_q <= 1'b0; pm_q <= 4'h0; pv_q <= 4'h0; po_q <= 4'h0; end
        else begin pw_q <= pin_we_i; pm_q <= pin_mask_i; pv_q <= pin_val_i; po_q <= pin_oe_i; end
    end
    wire       pw = pw_q;
    wire [3:0] pm = pm_q, pv = pv_q, po = po_q;
`else
    wire       pw = pin_we_i;
    wire [3:0] pm = pin_mask_i, pv = pin_val_i, po = pin_oe_i;
`endif

    wire v0 = d_pay_i[4], e0 = d_pay_i[3], w1 = d_pay_i[2], v1 = d_pay_i[1], e1 = d_pay_i[0];
    wire v3 = c_pay_i[1], e3 = c_pay_i[0];

    logic [3:0] val_n, oe_n;
    always_comb begin
        val_n = val;
        oe_n  = oe;
        if (!run_i) begin
`ifndef MUT_HALT_KEEPS_OE
            oe_n = 4'h0;
`endif
`ifdef MUT_HALT_CLEARS_VAL
            val_n = 4'h0;
`endif
        end else begin
            // lowest priority first; later assignments win
            if (c_we_i) begin val_n[3] = v3; oe_n[3] = e3; end
`ifdef MUT_CLK_OUT2
            if (c_we_i) begin val_n[2] = v3; oe_n[2] = e3; end
`endif
`ifdef MUT_W1_IGNORED
            if (d_we_i) begin val_n[1] = v1; oe_n[1] = e1; end
`else
            if (d_we_i && w1) begin val_n[1] = v1; oe_n[1] = e1; end
`endif
`ifdef MUT_REL_KEEPS_VAL
            if (d_we_i) begin if (e0) val_n[0] = v0; oe_n[0] = e0; end
`else
            if (d_we_i) begin val_n[0] = v0; oe_n[0] = e0; end
`endif
            if (pw) begin
                for (int j = 0; j < 4; j++) begin
                    if (pm[j]) begin val_n[j] = pv[j]; oe_n[j] = po[j]; end
`ifdef MUT_PIN_OE_NOMASK
                    oe_n[j] = po[j];
`endif
                end
            end
`ifdef MUT_DATA_BEATS_PIN
            if (d_we_i && w1) begin val_n[1] = v1; oe_n[1] = e1; end
            if (d_we_i) begin val_n[0] = v0; oe_n[0] = e0; end
`endif
`ifdef MUT_CLK_BEATS_PIN
            if (c_we_i) begin val_n[3] = v3; oe_n[3] = e3; end
`endif
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) val <= 4'h0;
        else val <= val_n;
    end
`ifdef MUT_OE_NO_RESET
    always_ff @(posedge clk) oe <= oe_n;
`else
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) oe <= 4'h0;
        else oe <= oe_n;
    end
`endif

`ifdef MUT_PIN_BYPASS
    always_comb begin
        out_val_o = val;
        if (run_i && pin_we_i) out_val_o = (val & ~pin_mask_i) | (pin_val_i & pin_mask_i);
    end
`else
    assign out_val_o = val;
`endif
`ifdef MUT_NO_GATE
    assign out_oe_o = oe;
`else
    assign out_oe_o = oe & {4{run_i}};
`endif
endmodule
