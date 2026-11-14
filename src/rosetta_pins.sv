`default_nettype none
// Lane output registers out0-out3 (docs/blocks/pins.md). Per output, first
// matching row of section 2: halt clears the enable (value kept), then PIN,
// then the data request (out0; out1 with w1), then the clock request (out3).
// out_oe_o is the only output not purely registered: oe & run_i.
module rosetta_pins (
    input  wire        clk, rst_n,
    input  wire        run_i,
    input  wire        d_we_i,
    input  wire  [4:0] d_pay_i,             // {v0, e0, w1, v1, e1}
    input  wire        c_we_i,
    input  wire  [1:0] c_pay_i,             // {v3, e3}
    input  wire        pin_we_i,
    input  wire  [3:0] pin_mask_i, pin_val_i, pin_oe_i,
    output logic [3:0] out_val_o,
    output logic [3:0] out_oe_o
);
  logic [3:0] val, oe, rq, rv, re;

  // Request (rows 3-5) per output: write enable, value, enable.
  assign rq = {c_we_i, 1'b0, d_we_i && d_pay_i[2], d_we_i};
  assign rv = {c_pay_i[1], 1'b0, d_pay_i[1], d_pay_i[4]};
  assign re = {c_pay_i[0], 1'b0, d_pay_i[0], d_pay_i[3]};

  for (genvar j = 0; j < 4; j++) begin : g_out
    always_ff @(posedge clk or negedge rst_n)
      if (!rst_n)                          begin val[j] <= 1'b0;         oe[j] <= 1'b0;         end
      else if (!run_i)                                                   oe[j] <= 1'b0;
      else if (pin_we_i && pin_mask_i[j])  begin val[j] <= pin_val_i[j]; oe[j] <= pin_oe_i[j]; end
      else if (rq[j])                      begin val[j] <= rv[j];        oe[j] <= re[j];        end
  end

  assign out_val_o = val;
  assign out_oe_o  = oe & {4{run_i}};
endmodule
