`default_nettype none
module rosetta_crc (
    input  wire         clk, rst_n,
    input  wire  [31:0] cfg_poly_i, cfg_init_i,
    input  wire  [4:0]  cfg_width_m1_i,
    input  wire         cfg_xorout_i, init_i, en_i, bit_i,
    output logic [31:0] state_o, value_o
);
  logic [31:0] s, mask, upd, nxt;
  logic fb;
  assign mask = ~(32'hFFFF_FFFE << cfg_width_m1_i);
  assign fb = s[cfg_width_m1_i] ^ bit_i;
  assign upd = {s[30:0],1'b0} ^ ({32{fb}} & cfg_poly_i);
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) s <= '0; else s <= nxt;

  assign nxt = init_i ? cfg_init_i : en_i ? upd : s;
  assign state_o = s & mask;
  assign value_o = mask & (s ^ {32{cfg_xorout_i}});
endmodule
