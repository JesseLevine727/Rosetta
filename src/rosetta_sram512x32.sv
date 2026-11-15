`default_nettype none

// rosetta_sram512x32: one 512 x 32 single-port SRAM (docs/blocks/mem.md 1).
//
// Default (ASIC flow): the IHP hard macro RM_IHPSG13_1P_512x32_c2_bm_bist,
// instance u_macro, with A_DLY = 1 and every BIST pin tied to 0. The macro
// declaration for synthesis is src/rosetta_sram_bb.v. A proof may also leave
// ROSETTA_SRAM_BEHAV undefined and supply its own model under the macro's name
// (mem.md 4).
//
// ROSETTA_SRAM_BEHAV (FPGA, simulation, formal): a behavioural synchronous
// memory with the macro's timing, written to the Vivado byte-write NO_CHANGE
// block-RAM template:
//   en & we : write, rdata holds (the macro with A_REN = 0 does not read);
//   en & !we: rdata <= mem[addr] at the clock edge (one-cycle read latency);
//   !en     : array and rdata hold.
// The single port never sees a read and a write in one cycle: the macro's
// write-through mode (A_WEN = A_REN = 1, the PDK model then returns the newly
// written word) is never used. Neither the array nor rdata has a reset, as in
// the macro, and nothing may rely on a word before it is written.
// The mask is honoured per byte (byte b written when bmask[8b] = 1), the
// granularity block RAM supports; the memory wrappers drive only the masks
// 0, 0000FFFF, FFFF0000 and FFFFFFFF, for which this equals the macro's
// per-bit mask.
//
// Every input goes straight to the macro pins (no registers: the wrappers add
// no latency). The macro needs up to 0.92 ns hold on address, data and mask
// at the slow corner; P&R inserts the hold buffers.
module rosetta_sram512x32 (
    input  wire         clk,
    input  wire         en, we,
    input  wire  [8:0]  addr,
    input  wire  [31:0] wdata, bmask,
    output logic [31:0] rdata
);
`ifdef ROSETTA_SRAM_BEHAV
  logic [31:0] mem [0:511];
  logic        unused_ok;

  assign unused_ok = &{1'b0, bmask[31:25], bmask[23:17], bmask[15:9],
                       bmask[7:1]};

  always_ff @(posedge clk)
    if (en) begin
      if (we) begin
        for (int b = 0; b < 4; b++)
          if (bmask[8*b]) mem[addr][8*b +: 8] <= wdata[8*b +: 8];
      end else
        rdata <= mem[addr];
    end
`else
  RM_IHPSG13_1P_512x32_c2_bm_bist u_macro (
      .A_CLK       (clk),
      .A_MEN       (en),
      .A_WEN       (en & we),
      .A_REN       (en & ~we),
      .A_ADDR      (addr),
      .A_DIN       (wdata),
      .A_DLY       (1'b1),
      .A_DOUT      (rdata),
      .A_BM        (bmask),
      .A_BIST_CLK  (1'b0),
      .A_BIST_EN   (1'b0),
      .A_BIST_MEN  (1'b0),
      .A_BIST_WEN  (1'b0),
      .A_BIST_REN  (1'b0),
      .A_BIST_ADDR (9'h0),
      .A_BIST_DIN  (32'h0),
      .A_BIST_BM   (32'h0)
  );
`endif
endmodule
