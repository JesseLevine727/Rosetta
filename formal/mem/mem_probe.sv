`default_nettype none
// ===========================================================================
//  PROBE model of the SRAM macro, for the unbounded proofs (task prove).
//
//  mem.md section 4: in proofs a model with the macro's name and ports
//  replaces RM_IHPSG13_1P_512x32_c2_bm_bist (ROSETTA_SRAM_BEHAV undefined, so
//  src/rosetta_sram512x32.sv instantiates the macro by name).
//
//  Why a probe and not a memory: the properties of mem.md 2.3 and 3.6 are
//  about the macro pins, which are not ports of the wrappers, and the
//  open-source Yosys front end has neither hierarchical references nor
//  `bind`. The wrappers pass A_DOUT straight to rdata_o (mem.md: "rdata_o =
//  A_DOUT", no added latency), so this model makes A_DOUT a combinational,
//  self-checking ENCODING of the pins it is driven with. The checker decodes
//  rdata_o and checks the macro drive bit-exactly in every cycle against the
//  tables of mem.md 2.2 / 3.4, and runs the tracked-word memory model of
//  mem.md 4.2 itself on the decoded pins. A wrapper that registers, gates or
//  alters rdata_o breaks the encoding and fails (a_probe_*).
//
//  The 32-bit data and mask buses do not fit next to the address, so one bit
//  position pb (anyconst, any of 0..31) of A_DIN and A_BM is exposed; a proof
//  for an arbitrary pb is a proof for every bit.
//
//  Encoding of A_DOUT:
//    [8:0]   A_ADDR        [9]  A_REN   [10] A_WEN   [11] A_MEN
//    [12]    A_DIN[pb]     [13] A_BM[pb]
//    [18:14] pb (constant)
//    [30:19] ~[11:0]       (complement check)
//    [31]    ^[18:0]       (parity check)
//
//  Pin rules that need no context are asserted here (mem.md 1, 2.3 item 2,
//  3.6 item 3, 4.1): A_DLY = 1; every BIST pin 0; A_WEN and A_REN never
//  both 1 (also without A_MEN: write-through is never used).
// ===========================================================================
module RM_IHPSG13_1P_512x32_c2_bm_bist (
    input  wire         A_CLK,
    input  wire         A_MEN,
    input  wire         A_WEN,
    input  wire         A_REN,
    input  wire  [8:0]  A_ADDR,
    input  wire  [31:0] A_DIN,
    input  wire         A_DLY,
    output wire  [31:0] A_DOUT,
    input  wire  [31:0] A_BM,
    input  wire         A_BIST_CLK,
    input  wire         A_BIST_EN,
    input  wire         A_BIST_MEN,
    input  wire         A_BIST_WEN,
    input  wire         A_BIST_REN,
    input  wire  [8:0]  A_BIST_ADDR,
    input  wire  [31:0] A_BIST_DIN,
    input  wire  [31:0] A_BIST_BM
);
    (* anyconst *) logic [4:0] pb;

    wire [11:0] lo = {A_MEN, A_WEN, A_REN, A_ADDR};
    wire [18:0] body = {pb, A_BM[pb], A_DIN[pb], lo};
    assign A_DOUT = {^body, ~lo, body};

    always @* begin
        a_pin_dly:      assert (A_DLY == 1'b1);
        a_pin_bist:     assert ({A_BIST_CLK, A_BIST_EN, A_BIST_MEN, A_BIST_WEN, A_BIST_REN} == 5'b0);
        a_pin_bist_bus: assert (A_BIST_ADDR == 9'h0 && A_BIST_DIN == 32'h0 && A_BIST_BM == 32'h0);
        a_pin_wr_rd:    assert (!(A_WEN && A_REN));
    end
endmodule
