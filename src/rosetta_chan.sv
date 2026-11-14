`default_nettype none

// rosetta_chan: lane channels and timestamp (docs/blocks/channels.md).
//
// State: data (4 x 32), valid (4), ts (32). 164 flops, all reset to 0.
// ch_data_o, ch_valid_o and ts_o are the state flops themselves.
//
// Put and get share ch_idx_i, so a put and a get in one cycle name the same
// channel and are both judged on its valid bit at the start of the cycle: at
// most one of them succeeds (the put if empty, the get if valid).
// A host clear of CHSTAT beats a put on the valid bit only; the put's data
// write (put && !valid) still happens. They never coincide: host writes are
// at ph = 1, which has no lane D cycle.
//
// ts + 1 is left to the mapper: with the CMOS5L library, ABC mapped every
// hand-built carry-select tried (2x16, 4x8, 8x4) to the same 11-level,
// ~2.5 ns (typ) network as the plain increment, so the RTL keeps the latter.

module rosetta_chan (
    input  wire          clk, rst_n,
    input  wire          ch_put_i, ch_get_i,
    input  wire  [1:0]   ch_idx_i,
    input  wire  [31:0]  ch_wdata_i,
    output logic [3:0]   ch_valid_o,
    output logic [127:0] ch_data_o,
    output logic [31:0]  ts_o,
    input  wire          hreg_we_i,
    input  wire  [6:0]   hreg_addr_i,
    input  wire  [31:0]  hreg_wdata_i,
    output logic [31:0]  hreg_rdata_o
);
  localparam logic [6:0] A_TIME = 7'h03, A_CHSTAT = 7'h06;

  logic [3:0] sel, put_ok, get_ok, clr;
  logic       unused_ok;

  // Lane operations: one-hot channel select, each judged on valid at t.
  assign sel    = 4'b0001 << ch_idx_i;
  assign put_ok = sel & ~ch_valid_o & {4{ch_put_i}};
  assign get_ok = sel &  ch_valid_o & {4{ch_get_i}};
  // Host write-1-to-clear on CHSTAT[3:0]; CHSTAT[31:4] are read-only zero.
  assign clr       = hreg_wdata_i[3:0] & {4{hreg_we_i && hreg_addr_i == A_CHSTAT}};
  assign unused_ok = &{1'b0, hreg_wdata_i[31:4]};

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      ch_valid_o <= '0;
      ts_o       <= '0;
    end else begin
      ch_valid_o <= (ch_valid_o | put_ok) & ~get_ok & ~clr;
      ts_o       <= ts_o + 32'd1;
    end

  // data[c] changes only by a successful put to c (no enable flop: a mux).
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) ch_data_o <= '0;
    else for (int c = 0; c < 4; c++)
      if (put_ok[c]) ch_data_o[32*c +: 32] <= ch_wdata_i;

  // Register reads, combinational; full 7-bit decode, 0 elsewhere.
  always_comb
    case (hreg_addr_i)
      A_TIME:   hreg_rdata_o = ts_o;
      A_CHSTAT: hreg_rdata_o = {28'h0, ch_valid_o};
      7'h08:    hreg_rdata_o = ch_data_o[31:0];
      7'h09:    hreg_rdata_o = ch_data_o[63:32];
      7'h0A:    hreg_rdata_o = ch_data_o[95:64];
      7'h0B:    hreg_rdata_o = ch_data_o[127:96];
      default:  hreg_rdata_o = '0;
    endcase
endmodule
