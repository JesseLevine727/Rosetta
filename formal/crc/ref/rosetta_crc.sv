`default_nettype none
// ============================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Written from docs/blocks/crc.md only, solely to show that the properties in
//  ../crc_props.sv pass on a correct implementation and fail on broken ones
//  (non-vacuity). The real block is src/rosetta_crc.sv, written independently.
//  Never integrate this file.
//
//  Planted bugs for the mutation runs (see mutate.sh, next to this file), one `define each:
//    MUT_FB_TAP      feedback taken from s[31] instead of s[W-1]
//    MUT_NO_MASK     no width mask anywhere (register and outputs)
//    MUT_PRIO        en_i wins over init_i
//    MUT_XOR_STATE   final XOR also applied to state_o
//    MUT_WIDTH_OB1   width off by one (computes a (W-1)-bit CRC)
//    MUT_HOLD        idle cycles with bit_i=1 shift (hold broken)
//    MUT_SYNC_RST    synchronous instead of asynchronous reset
//    MUT_RST_VAL     reset loads all ones instead of zero
//    MUT_OUT_UNMASK  register masked, outputs not masked by the current W
//    MUT_COMB_EN     state_o shows the next state while en_i is high
//    MUT_XOR_UNMASK  final XOR complements all 32 bits of value_o
//    MUT_POLY_SHIFT  polynomial XORed before the shift instead of after
//    MUT_W1          W=1 special case ignores the state (parity broken)
//    MUT_W32_MASK    mask built as (1 << (m1+1)) - 1 in 5 bits: W=32 gives 0
// ============================================================================
module rosetta_crc (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [31:0] cfg_poly_i,
    input  logic [31:0] cfg_init_i,
    input  logic [4:0]  cfg_width_m1_i,
    input  logic        cfg_xorout_i,
    input  logic        init_i,
    input  logic        en_i,
    input  logic        bit_i,
    output logic [31:0] state_o,
    output logic [31:0] value_o
);
    logic [31:0] s, s_next, mask, poly_m;
    logic [4:0]  top;     // index of the top state bit, W-1
    logic        fb;

`ifdef MUT_WIDTH_OB1
    assign top  = cfg_width_m1_i - 5'd1;
`else
    assign top  = cfg_width_m1_i;
`endif

`ifdef MUT_W32_MASK
    logic [4:0] w5;
    assign w5   = cfg_width_m1_i + 5'd1;          // wraps to 0 for W=32
    assign mask = (32'd1 << w5) - 32'd1;
`elsif MUT_NO_MASK
    assign mask = 32'hFFFF_FFFF;
`else
    assign mask = 32'hFFFF_FFFF >> (5'd31 - top);  // W ones, right-aligned
`endif

    assign poly_m = cfg_poly_i & mask;

`ifdef MUT_FB_TAP
    assign fb = s[31] ^ bit_i;
`elsif MUT_W1
    assign fb = (cfg_width_m1_i == 5'd0) ? bit_i : (s[top] ^ bit_i);
`else
    assign fb = s[top] ^ bit_i;
`endif

    always_comb begin
`ifdef MUT_PRIO
        if (en_i)
`ifdef MUT_POLY_SHIFT
            s_next = ((s ^ (fb ? poly_m : 32'd0)) << 1) & mask;
`else
            s_next = ((s << 1) & mask) ^ (fb ? poly_m : 32'd0);
`endif
        else if (init_i)
            s_next = cfg_init_i & mask;
`else
        if (init_i)
            s_next = cfg_init_i & mask;
        else if (en_i)
`ifdef MUT_POLY_SHIFT
            s_next = ((s ^ (fb ? poly_m : 32'd0)) << 1) & mask;
`else
            s_next = ((s << 1) & mask) ^ (fb ? poly_m : 32'd0);
`endif
`endif
`ifdef MUT_HOLD
        else if (bit_i)
            s_next = ((s << 1) & mask) ^ (fb ? poly_m : 32'd0);
`endif
        else
            s_next = s;
    end

`ifdef MUT_SYNC_RST
    always_ff @(posedge clk)
`else
    always_ff @(posedge clk or negedge rst_n)
`endif
`ifdef MUT_RST_VAL
        if (!rst_n) s <= 32'hFFFF_FFFF;
`else
        if (!rst_n) s <= 32'd0;
`endif
        else        s <= s_next;

    logic [31:0] s_vis;   // the state the outputs are built from
`ifdef MUT_COMB_EN
    assign s_vis = en_i ? s_next : s;
`else
    assign s_vis = s;
`endif

`ifdef MUT_OUT_UNMASK
    assign state_o = s_vis;
    assign value_o = s_vis ^ (cfg_xorout_i ? mask : 32'd0);
`elsif MUT_XOR_STATE
    assign state_o = (s_vis ^ {32{cfg_xorout_i}}) & mask;
    assign value_o = (s_vis ^ {32{cfg_xorout_i}}) & mask;
`elsif MUT_XOR_UNMASK
    assign state_o = s_vis & mask;
    assign value_o = (s_vis & mask) ^ {32{cfg_xorout_i}};
`else
    assign state_o = s_vis & mask;
    assign value_o = (s_vis ^ {32{cfg_xorout_i}}) & mask;
`endif

endmodule

`default_nettype wire
