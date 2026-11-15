`default_nettype none

// rosetta_mem_prog: program-memory wrapper (docs/blocks/mem.md 2).
//
// No flops (so no reset). The macro is driven combinationally in every cycle
// as a pure function of the wheel phase: lanes fetch in ph = 0..3, the host
// owns ph = 4. rdata_o is the macro's registered read data, one cycle after
// the address; the wrapper adds no latency.
//
// ph_i is the sequencer's one-hot ring: "ph in 0..3" is taken as !ph_i[4].
// Macro inputs that are not used in a cycle are driven to 0 (mem.md 2.2).
// Timing: fetch_addr_i reaches A_ADDR through one AND-OR level; the host
// inputs (flops in rosetta_host) through one more AND. None of them can be
// registered here without adding a cycle; the host paths are short and need
// P&R hold buffers (0.92 ns hold at the slow corner).
//
// The hard macro is u_sram.u_macro (rosetta_sram512x32).
module rosetta_mem_prog (
    input  wire         clk,
    input  wire  [4:0]  ph_i,
    input  wire  [8:0]  fetch_addr_i,
    input  wire         h_en_i, h_we_i,
    input  wire  [8:0]  h_addr_i,
    input  wire  [31:0] h_wdata_i,
    output logic [31:0] rdata_o
);
  logic        lane, hacc, en, we;
  logic [8:0]  addr;
  logic [31:0] wdata;
  logic        unused_ok;

  assign unused_ok = &{1'b0, ph_i[3:0]};   // one-hot: ph_i[4] decides

  assign lane  = ~ph_i[4];                  // ph 0..3: fetch read
  assign hacc  = ph_i[4] & h_en_i;          // ph 4: host access
  assign en    = lane | hacc;
  assign we    = hacc & h_we_i;
  assign addr  = ({9{lane}} & fetch_addr_i) | ({9{hacc}} & h_addr_i);
  assign wdata = {32{we}} & h_wdata_i;

  rosetta_sram512x32 u_sram (
      .clk   (clk),
      .en    (en),
      .we    (we),
      .addr  (addr),
      .wdata (wdata),
      .bmask ({32{we}}),
      .rdata (rdata_o)
  );
endmodule
