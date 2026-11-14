`default_nettype none
// Lane mode-register file (docs/blocks/lane.md 5.1-5.2; decisions.md rulings
// 21 and 23). lane.md 3 names this module rosetta_lane_cfg; it is
// rosetta_cfg here to follow conventions.md (file src/rosetta_<block>.sv,
// module rosetta_<block>).
//
// Registers 0-13, all reset to 0, kept while halted, written at the end of a
// cycle with run_i && cfg_we_i. A low write (cfg_hi_i = 0) loads the stored
// bits of register cfg_reg_i from cfg_data_i; a high write loads
// CRCPOLY[31:16] and changes nothing else (model D42). Bits not listed in
// lane.md table 5.1 are not stored. THR1 and THR2 do not exist (ruling 23):
// register 3 stores THR0 = data[7:0], register 4 the launch delay =
// data[15:8]. EVCk is stored as written (new 3-bit trigger layout, I17);
// undefined trigger and B-must codes are decoded as off/never by
// rosetta_events (ruling 21).
//
// Write pulses (5.2) are combinational from the sequencer flops cfg_we_i,
// cfg_reg_i, cfg_hi_i and run_i in the write cycle. The second CRC init
// (crcw_q, 5.3) is a top-level flop (lane.md 3), not part of this module.
//
// 198 flops.
module rosetta_cfg (
    input  wire         clk, rst_n,
    input  wire         run_i,
    input  wire         cfg_we_i,
    input  wire  [4:0]  cfg_reg_i,
    input  wire         cfg_hi_i,
    input  wire  [15:0] cfg_data_i,
    // Stored registers (lane.md table 5.1).
    output logic [12:0] tick_o,         // 0  TICK[12:0]
    output logic [15:0] period_o,       // 1  PERIOD
    output logic [15:0] period2_o,      // 2  PERIOD2
    output logic [7:0]  thr0_o,         // 3  THR01[7:0] = THR0
    output logic [7:0]  dly_o,          // 4  THR2L[15:8] = launch delay
    output logic [11:0] code_o,         // 5  CODE[11:0]
    output logic [15:0] pulse_o,        // 6  PULSE
    output logic [5:0]  stuff_o,        // 7  STUFF[5:0]
    output logic [31:0] crcpoly_o,      // 8  CRCPOLY
    output logic [6:0]  crccfg_o,       // 9  CRCCFG[6:0]
    output logic [63:0] evc_o,          // 10-13 EVCk at [16k +: 16]
    // Write side-effect pulses (lane.md 5.2), valid in the write cycle.
    output logic        wr_tick_o,
    output logic        wr_code_o,
    output logic        wr_stuff_o,
    output logic        wr_crc_o,
    output logic [3:0]  wr_evc_o
);
  logic        we, wlo;
  logic [13:0] sel;                     // one-hot register select, low write

  assign we  = run_i && cfg_we_i;
  assign wlo = we && !cfg_hi_i;
  always_comb
    for (int r = 0; r < 14; r++)
      sel[r] = wlo && cfg_reg_i == 5'(r);

  // 5.2: "write to r" is a low write; a high write is a write only to 8.
  assign wr_tick_o  = sel[0];
  assign wr_code_o  = sel[5];
  assign wr_stuff_o = sel[7];
  assign wr_crc_o   = (we && cfg_reg_i == 5'd8) || sel[9];
  assign wr_evc_o   = sel[13:10];

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      tick_o <= '0; period_o <= '0; period2_o <= '0; thr0_o <= '0; dly_o <= '0;
      code_o <= '0; pulse_o <= '0; stuff_o <= '0; crcpoly_o <= '0; crccfg_o <= '0;
      evc_o <= '0;
    end else begin
      if (sel[0])  tick_o    <= cfg_data_i[12:0];
      if (sel[1])  period_o  <= cfg_data_i;
      if (sel[2])  period2_o <= cfg_data_i;
      if (sel[3])  thr0_o    <= cfg_data_i[7:0];
      if (sel[4])  dly_o     <= cfg_data_i[15:8];
      if (sel[5])  code_o    <= cfg_data_i[11:0];
      if (sel[6])  pulse_o   <= cfg_data_i;
      if (sel[7])  stuff_o   <= cfg_data_i[5:0];
      if (sel[8])  crcpoly_o[15:0]  <= cfg_data_i;
      if (we && cfg_hi_i && cfg_reg_i == 5'd8)
                   crcpoly_o[31:16] <= cfg_data_i;
      if (sel[9])  crccfg_o  <= cfg_data_i[6:0];
      for (int k = 0; k < 4; k++)
        if (sel[10 + k]) evc_o[16*k +: 16] <= cfg_data_i;
    end
endmodule
