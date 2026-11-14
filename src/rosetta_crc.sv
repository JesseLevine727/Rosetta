`default_nettype none

// rosetta_crc: CRC check unit, any width W = 1..32, one wire bit per update.
// Spec: docs/blocks/crc.md
//
// The only state is crc_q, which holds the W-bit state s zero-extended to
// 32 bits. The width mask is decoded combinationally from cfg_width_m1_i and
// is applied once on the next-state value; that single mask implements the
// "(s << 1)[W-1:0]", "cfg_poly_i[W-1:0]" and "cfg_init_i[W-1:0]" truncations
// of the spec. The outputs are masked again so that bits W and above read as
// 0 for every cfg value, not only when the configuration has been stable
// since the last write. The outputs depend on crc_q and cfg_* only, never on
// bit_i, en_i or init_i.

module rosetta_crc (
    input  wire         clk,
    input  wire         rst_n,
    input  wire  [31:0] cfg_poly_i,      // generator without top term, low W bits
    input  wire  [31:0] cfg_init_i,      // value loaded by init_i, low W bits
    input  wire  [4:0]  cfg_width_m1_i,  // W - 1
    input  wire         cfg_xorout_i,    // 1: value_o is the complemented state
    input  wire         init_i,          // load cfg_init_i (wins over en_i)
    input  wire         en_i,            // consume bit_i this cycle
    input  wire         bit_i,           // next bit, in wire order
    output logic [31:0] state_o,         // raw state, bits W and above are 0
    output logic [31:0] value_o          // state ^ xorout, bits W and above are 0
);

  // mask[i] = 1 iff i < W, i.e. i <= cfg_width_m1_i.
  logic [31:0] mask;
  assign mask = ~(32'hFFFF_FFFE << cfg_width_m1_i);

  logic [31:0] crc_q;    // state s, zero-extended
  logic [31:0] shifted;  // s << 1, xor polynomial when feedback is set
  logic [31:0] crc_sel;  // next state before width masking
  logic        fb;       // s[W-1] ^ bit_i

  assign fb      = crc_q[cfg_width_m1_i] ^ bit_i;
  assign shifted = {crc_q[30:0], 1'b0} ^ ({32{fb}} & cfg_poly_i);

  always_comb begin
    if (init_i) begin
      crc_sel = cfg_init_i;
    end else if (en_i) begin
      crc_sel = shifted;
    end else begin
      crc_sel = crc_q;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crc_q <= '0;
    end else begin
      crc_q <= crc_sel & mask;
    end
  end

  assign state_o = crc_q & mask;
  assign value_o = mask & (crc_q ^ {32{cfg_xorout_i}});

endmodule

`default_nettype wire
