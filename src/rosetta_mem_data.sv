`default_nettype none

// rosetta_mem_data: data-memory wrapper and host queues (docs/blocks/mem.md 3).
//
// State (128 flops, all reset to 0): per lane in_wr, in_rd, out_wr, out_rd
// (7 each), isz, osz (2 each). Lane L's fields are at [7L +: 7] / [2L +: 2].
//
// The macro is driven combinationally in every cycle (mem.md 3.4, first
// matching row): the host owns ph = 1, otherwise the D owner o of ph
// (ph 0 -> L3, 2 -> L0, 3 -> L1, 4 -> L2). A lane address is always
// {o, offset} with o taken from ph only, so a lane cannot leave its region.
// rdata_o is the macro's registered read data; the wrapper adds no latency.
// ph_i is the sequencer's one-hot ring.
//
// Queue arithmetic (3.2), per lane, from flops only (ahead of the late dm_*
// inputs): m = 2S - 1 = {sz == 3, sz >= 2, sz != 0, 4'hF}, S - 1 = m >> 1,
// empty = ((wr ^ rd) & m) == 0, full = ((wr ^ rd) & m) == S,
// next(p) = (p + 1) & m (equal to ((p & m) + 1) & m),
// host-in offset = rd & (S - 1),
// host-out offset = (128 - So) | idx(wr) = wr | ~(So - 1).
// The owner's offsets and flags are picked by a one-hot AND-OR on ph.
//
// Timing: dm_op_i/dm_off_i/dm_hmask_i/dm_wdata_i come combinationally from
// the sequencer's D stage and reach the macro through op decode, row select
// and an AND-OR (8 cells deep, dm_op_i -> A_DIN, after CMOS5L mapping). They
// cannot be registered without a cycle of latency, which the wheel schedule
// does not have. The host inputs
// (flops in rosetta_host) take one or two levels and need P&R hold buffers
// (0.92 ns hold at the slow corner).
//
// The hard macro is u_sram.u_macro (rosetta_sram512x32).
module rosetta_mem_data (
    input  wire         clk, rst_n,
    input  wire  [4:0]  ph_i,
    input  wire  [2:0]  dm_op_i,
    input  wire  [6:0]  dm_off_i,
    input  wire  [1:0]  dm_hmask_i,
    input  wire  [31:0] dm_wdata_i,
    input  wire         h_en_i, h_we_i,
    input  wire  [8:0]  h_addr_i,
    input  wire  [31:0] h_wdata_i,
    input  wire  [3:0]  run_i,
    input  wire         hreg_we_i,
    input  wire  [6:0]  hreg_addr_i,
    input  wire  [31:0] hreg_wdata_i,
    output logic [31:0] hreg_rdata_o,
    output logic [31:0] rdata_o,
    output logic [3:0]  qe_o, qf_o, qne_o,
    output logic [15:0] obs_qcfg_o,
    output logic [27:0] obs_in_wr_o, obs_in_rd_o, obs_out_wr_o, obs_out_rd_o
);
  localparam logic [2:0] DM_READ = 3'd1, DM_WRITE = 3'd2, DM_POP = 3'd3,
                         DM_PUSH = 3'd4;
  localparam logic [6:0] A_IRQSTAT = 7'h04;

  // ======================================================== State
  logic [27:0] in_wr, in_rd, out_wr, out_rd;
  logic [7:0]  isz, osz;

  // ================================================ Queue arithmetic (3.2)
  logic [27:0] in_nxt, out_nxt, pop_off, push_off;
  logic [3:0]  in_full;

  function automatic logic [6:0] qmask(input logic [1:0] sz);   // 2S - 1
    qmask = {sz == 2'd3, sz[1], sz != 2'd0, 4'hF};
  endfunction

  for (genvar L = 0; L < 4; L++) begin : g_q
    logic [6:0] mi, mo, si, so, dxi, dxo;
    assign mi = qmask(isz[2*L +: 2]);
    assign mo = qmask(osz[2*L +: 2]);
    assign si = mi ^ {1'b0, mi[6:1]};                 // S = (m >> 1) + 1
    assign so = mo ^ {1'b0, mo[6:1]};
    assign dxi = (in_wr[7*L +: 7] ^ in_rd[7*L +: 7]) & mi;
    assign dxo = (out_wr[7*L +: 7] ^ out_rd[7*L +: 7]) & mo;
    assign qe_o[L]    = dxi == 7'd0;
    assign in_full[L] = dxi == si;
    assign qf_o[L]    = dxo == so;
    assign qne_o[L]   = dxo != 7'd0;
    assign in_nxt[7*L +: 7]   = (in_rd[7*L +: 7] + 7'd1) & mi;
    assign out_nxt[7*L +: 7]  = (out_wr[7*L +: 7] + 7'd1) & mo;
    assign pop_off[7*L +: 7]  = in_rd[7*L +: 7] & {1'b0, mi[6:1]};
    assign push_off[7*L +: 7] = out_wr[7*L +: 7] | ~{1'b0, mo[6:1]};
  end

  // ================================================== Macro drive (3.4)
  logic [3:0]  dl;                  // one-hot D owner
  logic [1:0]  o;                   // D owner, binary
  logic        hp, hacc, hw, lane, sel_qe, sel_qf;
  logic        l_rd, l_wr, l_pop, l_push, l_off, l_acc;
  logic [6:0]  sel_pop, sel_push, off;
  logic        en, we;
  logic [8:0]  addr;
  logic [31:0] wdata, bmask;

  assign dl = {ph_i[0], ph_i[4], ph_i[3], ph_i[2]};
  assign o  = {ph_i[0] | ph_i[4], ph_i[0] | ph_i[3]};
  assign hp = ph_i[1];

  always_comb begin
    sel_qe = 1'b0;
    sel_qf = 1'b0;
    sel_pop  = '0;
    sel_push = '0;
    for (int L = 0; L < 4; L++) begin
      sel_qe   |= dl[L] & qe_o[L];
      sel_qf   |= dl[L] & qf_o[L];
      sel_pop  |= {7{dl[L]}} & pop_off[7*L +: 7];
      sel_push |= {7{dl[L]}} & push_off[7*L +: 7];
    end
  end

  assign hacc   = hp & h_en_i;
  assign hw     = hacc & h_we_i;
  assign lane   = ~hp;            // lane requests are ignored in ph = 1
  assign l_rd   = lane & (dm_op_i == DM_READ);
  assign l_wr   = lane & (dm_op_i == DM_WRITE) & (|dm_hmask_i);
  assign l_pop  = lane & (dm_op_i == DM_POP) & ~sel_qe;
  assign l_push = lane & (dm_op_i == DM_PUSH) & ~sel_qf;
  assign l_off  = l_rd | l_wr;
  assign l_acc  = l_off | l_pop | l_push;

  assign en    = hacc | l_acc;
  assign we    = hw | l_wr | l_push;
  assign off   = ({7{hacc}} & h_addr_i[6:0]) | ({7{l_off}} & dm_off_i) |
                 ({7{l_pop}} & sel_pop) | ({7{l_push}} & sel_push);
  assign addr  = {({2{hacc}} & h_addr_i[8:7]) | ({2{l_acc}} & o), off};
  assign wdata = ({32{hw}} & h_wdata_i) | ({32{l_wr | l_push}} & dm_wdata_i);
  assign bmask = {{16{hw | l_push | (l_wr & dm_hmask_i[1])}},
                  {16{hw | l_push | (l_wr & dm_hmask_i[0])}}};

  rosetta_sram512x32 u_sram (
      .clk   (clk),
      .en    (en),
      .we    (we),
      .addr  (addr),
      .wdata (wdata),
      .bmask (bmask),
      .rdata (rdata_o)
  );

  // ================================== Pointer and configuration updates
  // Host register writes (host.md: QCFG/QIN/QOUT[L] = 0x10 + 8L + 5/6/7)
  // act only in ph = 1; lane pops and pushes never happen in ph = 1, so a
  // pointer never has two writers in one cycle.
  logic [3:0] rsel, w_cfg, w_in, w_out;
  logic [2:0] ra;

  assign ra = hreg_addr_i[2:0];
  for (genvar L = 0; L < 4; L++) begin : g_w
    logic hsel;
    assign rsel[L]  = hreg_addr_i[6:3] == 4'(2 + L);
    assign hsel     = hp & hreg_we_i & rsel[L];
    assign w_cfg[L] = hsel & (ra == 3'd5);
    assign w_in[L]  = hsel & (ra == 3'd6);
    assign w_out[L] = hsel & (ra == 3'd7);
  end

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      in_wr  <= '0;
      in_rd  <= '0;
      out_wr <= '0;
      out_rd <= '0;
      isz    <= '0;
      osz    <= '0;
    end else
      for (int L = 0; L < 4; L++) begin
        if (w_cfg[L]) begin
          isz[2*L +: 2] <= hreg_wdata_i[1:0];
          osz[2*L +: 2] <= hreg_wdata_i[3:2];
        end
        if (w_in[L])  in_wr[7*L +: 7]  <= hreg_wdata_i[6:0];
        if (w_out[L]) out_rd[7*L +: 7] <= hreg_wdata_i[6:0];
        if (w_in[L] && !run_i[L])        in_rd[7*L +: 7] <= hreg_wdata_i[14:8];
        else if (l_pop && dl[L])         in_rd[7*L +: 7] <= in_nxt[7*L +: 7];
        if (w_out[L] && !run_i[L])       out_wr[7*L +: 7] <= hreg_wdata_i[14:8];
        else if (l_push && dl[L])        out_wr[7*L +: 7] <= out_nxt[7*L +: 7];
      end

  // ============================================ Register reads (combinational)
  always_comb begin
    hreg_rdata_o = '0;
    if (hreg_addr_i == A_IRQSTAT) hreg_rdata_o[7:4] = qne_o;
    for (int L = 0; L < 4; L++)
      if (rsel[L])
        case (ra)
          3'd5: hreg_rdata_o = {28'h0, osz[2*L +: 2], isz[2*L +: 2]};
          3'd6: hreg_rdata_o = {14'h0, in_full[L], qe_o[L], 1'b0,
                                in_rd[7*L +: 7], 1'b0, in_wr[7*L +: 7]};
          3'd7: hreg_rdata_o = {14'h0, qf_o[L], ~qne_o[L], 1'b0,
                                out_wr[7*L +: 7], 1'b0, out_rd[7*L +: 7]};
          default: ;
        endcase
  end

  // ========================================================= Observation
  for (genvar L = 0; L < 4; L++) begin : g_obs
    assign obs_qcfg_o[4*L +: 4] = {osz[2*L +: 2], isz[2*L +: 2]};
  end
  assign obs_in_wr_o  = in_wr;
  assign obs_in_rd_o  = in_rd;
  assign obs_out_wr_o = out_wr;
  assign obs_out_rd_o = out_rd;

  // hreg_wdata_i[31:15] and [7] are not used by any queue register.
  logic unused_ok;
  assign unused_ok = &{1'b0, hreg_wdata_i[31:15], hreg_wdata_i[7]};
endmodule
