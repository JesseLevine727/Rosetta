`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_mem_data (data-memory wrapper and host
//  queues).
//
//  Written from docs/blocks/mem.md (sections 1, 3, 4), host.md section 4
//  (register addresses), sequencer.md section 1 (wheel, packing) and
//  docs/decisions.md ONLY (the RTL was not read). Properties are about the
//  block's ports; obs_* are ports; the macro pins are seen through rdata_o
//  in PROBE mode (see below).
//
//  Contents
//   [M] Executable model of 3.2-3.4: the 128 spec flops (isz, osz, in_wr,
//       in_rd, out_wr, out_rd per lane), driven from the input ports, and
//       the combinational outputs; compared with obs_*, qe_o, qf_o, qne_o
//       and hreg_rdata_o in EVERY cycle, reset cycles included (a_m_*).
//       The model computes the spec macro drive e_* of the 3.4 table.
//   [D] PROBE = 1 only (task prove, unbounded): the macro is the probe
//       model mem_probe.sv, rdata_o (= A_DOUT, 3.4) carries a self-checking
//       encoding of the macro pins of the same cycle. a_probe_* check that
//       rdata_o is exactly A_DOUT; a_d_* check the drive table of 3.4
//       bit-exactly in every cycle out of reset (A_DIN / A_BM through the
//       probe's anyconst bit pb); a_z_* the 0 entries (STRICT_ZERO); in
//       reset cycles a_d_rst allows the table row or no access.
//   [P] Required properties 3.6, stated directly on the ports:
//       a_p1_region     1  A_ADDR[8:7] = owner for every access with ph != 1
//                          (PROBE; every cycle, reset included)
//       a_p2_*          2  host only in ph = 1, lanes ignored in ph = 1
//       (mem_probe.sv)  3  A_DLY = 1, BIST pins 0, !(A_WEN & A_REN)
//       a_p4_*          4  POP empty / PUSH full: no access, no pointer
//                          change
//       a_p5_*          5  pointer ownership (who may change each pointer)
//       a_p6_*          6  queue accesses inside their queue
//       a_p7_*          7  FIFO behaviour with the tracked-word model of
//                          4.2: a pop / host read / lane read of the tracked
//                          address ca returns, bit tb, the last value the
//                          spec wrote there (host word pushed to host-in and
//                          popped by the lane; lane word pushed to host-out
//                          and read by the host); a push never overwrites an
//                          unread entry, a pop never reads an unpublished one
//       a_p8_*          8  flags against count = (wr' - rd') & m (reference
//                          formula with subtraction); count stays in [0, S]
//                          across every cycle without a host write of that
//                          queue's pointers or sizes
//       a_p9_half       9  a half-word store leaves the other half unchanged
//       a_hold             (bmc) A_DOUT holds after a cycle without a read
//   Covers (c_*): item 10 and every assumption.
//
//  Memory model per task (PROBE from `MEM_PROBE in the proof top):
//   PROBE = 1: mem_probe.sv; the checker itself runs the tracked-word model
//              of 4.2 (one tracked address ca, tracked bit pb, arbitrary
//              initial value, other addresses read anyseq) on the decoded
//              pins: m_d, m_q. Lemma l_trk ties it to the spec shadow.
//   PROBE = 0: src/rosetta_sram512x32.sv with ROSETTA_SRAM_BEHAV (full
//              array, bmc and cover); rdata_o is checked directly.
//
//  Environment (assumed; each has a cover):
//   env_rst        rst_n = 0 in the first cycle (flops start arbitrary);
//                  free afterwards (mid-trace resets allowed)
//   env_ph_onehot  ph_i one-hot. Rotation is NOT assumed: no property needs
//                  it, so the proof holds for any one-hot phase sequence.
//   env_hreg_ph    ONLY if HREG_PH_GATED = 0: hreg_we_i only when ph = 1.
//   Nothing else: dm_op_i (codes 5-7 included), h_en_i in any phase,
//   hreg_* with any address, run_i, all data are free.
//
//  Spec readings (marked choices; also in the report):
//   * "ph = k" is ph_i[k]; owner o: ph0 -> 3, ph2 -> 0, ph3 -> 1, ph4 -> 2.
//   * HREG_PH_GATED = 1: 3.4 says the register updates happen on "ph = 1
//     and hreg_we_i", so a hreg_we_i outside ph = 1 is IGNORED (checked).
//     Set HREG_PH_GATED = 0 to instead assume the port-table promise
//     ("only when ph = 1") and not check the gating.
//   * STRICT_ZERO = 1: the 0 entries of the 3.4 table (A_ADDR, A_DIN, A_BM
//     where unused) are required. 0: don't-care.
//   * Reset cycles: the macro drive in a cycle with rst_n = 0 may be the
//     table row (computed from the reset state) or no access (the spec
//     does not say whether accesses are gated by reset; memory contents are
//     undefined until written anyway). The tracked word forgets on reset.
//   * Per-lane packing (sequencer.md 1): lane L in bits [W*L +: W];
//     obs_qcfg_o[4L +: 4] = {osz[L], isz[L]}.
//   * hreg_rdata_o: IRQSTAT (0x04) = {24'h0, qne_o, 4'h0}; QCFG/QIN/QOUT[L]
//     at 0x15/0x16/0x17 + 8L as 3.4; every other address 0. Pointers read
//     back as stored (unmasked).
//   * Lane pops/pushes are not gated by run_i (3.4 has no such condition).
// ===========================================================================

module mem_data_props #(
    parameter PROBE         = 0,
    parameter STRICT_ZERO   = 1,
    parameter HREG_PH_GATED = 1
) (
    input wire         clk,
    input wire         rst_n,
    input wire [4:0]   ph_i,
    input wire [2:0]   dm_op_i,
    input wire [6:0]   dm_off_i,
    input wire [1:0]   dm_hmask_i,
    input wire [31:0]  dm_wdata_i,
    input wire         h_en_i,
    input wire         h_we_i,
    input wire [8:0]   h_addr_i,
    input wire [31:0]  h_wdata_i,
    input wire [3:0]   run_i,
    input wire         hreg_we_i,
    input wire [6:0]   hreg_addr_i,
    input wire [31:0]  hreg_wdata_i,
    input wire [31:0]  hreg_rdata_o,
    input wire [31:0]  rdata_o,
    input wire [3:0]   qe_o,
    input wire [3:0]   qf_o,
    input wire [3:0]   qne_o,
    input wire [15:0]  obs_qcfg_o,
    input wire [27:0]  obs_in_wr_o,
    input wire [27:0]  obs_in_rd_o,
    input wire [27:0]  obs_out_wr_o,
    input wire [27:0]  obs_out_rd_o
);
    localparam [2:0] OP_NONE = 3'd0, OP_READ = 3'd1, OP_WRITE = 3'd2,
                     OP_POP  = 3'd3, OP_PUSH = 3'd4;
    localparam [6:0] A_IRQSTAT = 7'h04;
    // access kinds (for naming the end-to-end checks)
    localparam [2:0] K_NONE = 3'd0, K_HWR = 3'd1, K_HRD = 3'd2, K_LRD = 3'd3,
                     K_LWR  = 3'd4, K_POP = 3'd5, K_PUSH = 3'd6;

    // =====================================================================
    // Queue arithmetic (3.2)
    // =====================================================================
    function automatic [6:0] f_size(input [1:0] c);          // S = 8 << c
        case (c)
            2'd0: f_size = 7'd8;
            2'd1: f_size = 7'd16;
            2'd2: f_size = 7'd32;
            default: f_size = 7'd64;
        endcase
    endfunction
    function automatic [6:0] f_m(input [1:0] c);             // m = 2S - 1
        case (c)
            2'd0: f_m = 7'd15;
            2'd1: f_m = 7'd31;
            2'd2: f_m = 7'd63;
            default: f_m = 7'd127;
        endcase
    endfunction
    function automatic [6:0] f_idx(input [6:0] p, input [1:0] c);
        f_idx = p & (f_size(c) - 7'd1);
    endfunction
    function automatic f_empty(input [6:0] wr, input [6:0] rd, input [1:0] c);
        f_empty = ((wr & f_m(c)) == (rd & f_m(c)));
    endfunction
    function automatic f_full(input [6:0] wr, input [6:0] rd, input [1:0] c);
        f_full = (((wr & f_m(c)) ^ (rd & f_m(c))) == f_size(c));
    endfunction
    function automatic [6:0] f_next(input [6:0] p, input [1:0] c);
        f_next = ((p & f_m(c)) + 7'd1) & f_m(c);
    endfunction
    // reference only (property 8): count with a subtraction
    function automatic [6:0] f_cnt(input [6:0] wr, input [6:0] rd, input [1:0] c);
        f_cnt = ((wr & f_m(c)) - (rd & f_m(c))) & f_m(c);
    endfunction
    function automatic [6:0] f_qcfg(input [1:0] l); f_qcfg = 7'h15 + {2'b00, l, 3'b000}; endfunction
    function automatic [6:0] f_qin (input [1:0] l); f_qin  = 7'h16 + {2'b00, l, 3'b000}; endfunction
    function automatic [6:0] f_qout(input [1:0] l); f_qout = 7'h17 + {2'b00, l, 3'b000}; endfunction

    // =====================================================================
    // Environment
    // =====================================================================
    wire env_onehot = (ph_i == 5'b00001) || (ph_i == 5'b00010) || (ph_i == 5'b00100) ||
                      (ph_i == 5'b01000) || (ph_i == 5'b10000);
    always @* begin
        if ($initstate) env_rst: assume (!rst_n);
        env_ph_onehot: assume (env_onehot);
        if (!HREG_PH_GATED) env_hreg_ph: assume (!hreg_we_i || ph_i[1]);
    end

    // rv1: rst_n high in the previous cycle and now
    logic rv1;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) rv1 <= 1'b0;
        else        rv1 <= 1'b1;

    // =====================================================================
    // [M] Model
    // =====================================================================
    wire       hp = ph_i[1];                               // host position
    wire [1:0] o  = ph_i[0] ? 2'd3 : ph_i[2] ? 2'd0 : ph_i[3] ? 2'd1 : 2'd2;

    logic [3:0][1:0] m_isz, m_osz;
    logic [3:0][6:0] m_in_wr, m_in_rd, m_out_wr, m_out_rd;

    logic [3:0] m_qe, m_fi, m_qf, m_eo;
    always_comb begin
        for (int l = 0; l < 4; l++) begin
            m_qe[l] = f_empty(m_in_wr[l],  m_in_rd[l],  m_isz[l]);
            m_fi[l] = f_full (m_in_wr[l],  m_in_rd[l],  m_isz[l]);
            m_qf[l] = f_full (m_out_wr[l], m_out_rd[l], m_osz[l]);
            m_eo[l] = f_empty(m_out_wr[l], m_out_rd[l], m_osz[l]);
        end
    end

    // ---- spec macro drive (3.4, first matching row)
    logic        e_men, e_wen, e_ren, e_pop, e_push;
    logic [8:0]  e_addr;
    logic [31:0] e_din, e_bm;
    logic [2:0]  e_kind;
    wire  [1:0]  m_isz_o = m_isz[o];
    wire  [1:0]  m_osz_o = m_osz[o];
    wire  [6:0]  e_in_off  = f_idx(m_in_rd[o], m_isz_o);
    wire  [6:0]  e_out_off = (7'd0 - f_size(m_osz_o)) | f_idx(m_out_wr[o], m_osz_o);
    always_comb begin
        e_men = 1'b0; e_wen = 1'b0; e_ren = 1'b0; e_pop = 1'b0; e_push = 1'b0;
        e_addr = 9'h0; e_din = 32'h0; e_bm = 32'h0; e_kind = K_NONE;
        if (hp) begin
            if (h_en_i) begin
                e_men = 1'b1; e_wen = h_we_i; e_ren = !h_we_i; e_addr = h_addr_i;
                if (h_we_i) begin e_din = h_wdata_i; e_bm = 32'hFFFF_FFFF; end
                e_kind = h_we_i ? K_HWR : K_HRD;
            end
        end else begin
            case (dm_op_i)
                OP_READ: begin
                    e_men = 1'b1; e_ren = 1'b1; e_addr = {o, dm_off_i}; e_kind = K_LRD;
                end
                OP_WRITE: if (dm_hmask_i != 2'b00) begin
                    e_men = 1'b1; e_wen = 1'b1; e_addr = {o, dm_off_i}; e_din = dm_wdata_i;
                    e_bm = {{16{dm_hmask_i[1]}}, {16{dm_hmask_i[0]}}}; e_kind = K_LWR;
                end
                OP_POP: if (!m_qe[o]) begin
                    e_men = 1'b1; e_ren = 1'b1; e_addr = {o, e_in_off}; e_pop = 1'b1; e_kind = K_POP;
                end
                OP_PUSH: if (!m_qf[o]) begin
                    e_men = 1'b1; e_wen = 1'b1; e_addr = {o, e_out_off}; e_din = dm_wdata_i;
                    e_bm = 32'hFFFF_FFFF; e_push = 1'b1; e_kind = K_PUSH;
                end
                default: ;
            endcase
        end
    end

    // ---- register writes (3.4, host.md 4)
    wire hwr = hreg_we_i && hp;
    logic [3:0] w_qcfg, w_qin, w_qout;
    always_comb begin
        for (int l = 0; l < 4; l++) begin
            w_qcfg[l] = hwr && (hreg_addr_i == f_qcfg(l));
            w_qin[l]  = hwr && (hreg_addr_i == f_qin(l));
            w_qout[l] = hwr && (hreg_addr_i == f_qout(l));
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_isz <= '0; m_osz <= '0;
            m_in_wr <= '0; m_in_rd <= '0; m_out_wr <= '0; m_out_rd <= '0;
        end else begin
            for (int l = 0; l < 4; l++) begin
                if (e_pop  && o == l) m_in_rd[l]  <= f_next(m_in_rd[l],  m_isz[l]);
                if (e_push && o == l) m_out_wr[l] <= f_next(m_out_wr[l], m_osz[l]);
                if (w_qcfg[l]) begin
                    m_isz[l] <= hreg_wdata_i[1:0];
                    m_osz[l] <= hreg_wdata_i[3:2];
                end
                if (w_qin[l]) begin
                    m_in_wr[l] <= hreg_wdata_i[6:0];
                    if (!run_i[l]) m_in_rd[l] <= hreg_wdata_i[14:8];
                end
                if (w_qout[l]) begin
                    m_out_rd[l] <= hreg_wdata_i[6:0];
                    if (!run_i[l]) m_out_wr[l] <= hreg_wdata_i[14:8];
                end
            end
        end
    end

    // ---- model outputs
    logic [15:0] m_obs_qcfg;
    logic [27:0] m_obs_in_wr, m_obs_in_rd, m_obs_out_wr, m_obs_out_rd;
    logic [31:0] m_hrd;
    always_comb begin
        m_hrd = 32'h0;
        if (hreg_addr_i == A_IRQSTAT) m_hrd = {24'h0, ~m_eo, 4'h0};
        for (int l = 0; l < 4; l++) begin
            m_obs_qcfg[4*l +: 4]   = {m_osz[l], m_isz[l]};
            m_obs_in_wr[7*l +: 7]  = m_in_wr[l];
            m_obs_in_rd[7*l +: 7]  = m_in_rd[l];
            m_obs_out_wr[7*l +: 7] = m_out_wr[l];
            m_obs_out_rd[7*l +: 7] = m_out_rd[l];
            if (hreg_addr_i == f_qcfg(l)) m_hrd = {28'h0, m_osz[l], m_isz[l]};
            if (hreg_addr_i == f_qin(l))
                m_hrd = {14'h0, m_fi[l], m_qe[l], 1'b0, m_in_rd[l], 1'b0, m_in_wr[l]};
            if (hreg_addr_i == f_qout(l))
                m_hrd = {14'h0, m_qf[l], m_eo[l], 1'b0, m_out_wr[l], 1'b0, m_out_rd[l]};
        end
    end

    always @* begin
        a_m_qcfg:   assert (obs_qcfg_o   == m_obs_qcfg);
        a_m_in_wr:  assert (obs_in_wr_o  == m_obs_in_wr);
        a_m_in_rd:  assert (obs_in_rd_o  == m_obs_in_rd);
        a_m_out_wr: assert (obs_out_wr_o == m_obs_out_wr);
        a_m_out_rd: assert (obs_out_rd_o == m_obs_out_rd);
        a_m_qe:     assert (qe_o  == m_qe);
        a_m_qf:     assert (qf_o  == m_qf);
        a_m_qne:    assert (qne_o == ~m_eo);
        a_m_hreg:   assert (hreg_rdata_o == m_hrd);
    end

    // =====================================================================
    // [D] Probe decode and drive table (PROBE = 1)
    // =====================================================================
    wire [8:0]  p_addr = rdata_o[8:0];
    wire        p_ren  = rdata_o[9];
    wire        p_wen  = rdata_o[10];
    wire        p_men  = rdata_o[11];
    wire        p_din  = rdata_o[12];                 // A_DIN[pb]
    wire        p_bm   = rdata_o[13];                 // A_BM[pb]
    wire [4:0]  p_pb   = rdata_o[18:14];
    logic [4:0] q_pb;
    always_ff @(posedge clk) q_pb <= p_pb;

    wire d_zero_ok = !STRICT_ZERO || ((e_men || p_addr == 9'h0) && (e_wen || (!p_din && !p_bm)));
    wire d_exact   = (p_men == e_men) && (p_wen == e_wen) && (p_ren == e_ren) &&
                     (!e_men || p_addr == e_addr) &&
                     (!e_wen || (p_din == e_din[p_pb] && p_bm == e_bm[p_pb])) && d_zero_ok;
    wire d_idle    = !p_men && !p_wen && !p_ren &&
                     (!STRICT_ZERO || (p_addr == 9'h0 && !p_din && !p_bm));

    // obs-derived sizes (port level) for properties 6 and 8
    wire [1:0]  ob_isz_o = obs_qcfg_o[4*o +: 2];
    wire [1:0]  ob_osz_o = obs_qcfg_o[4*o + 2 +: 2];

    generate if (PROBE) begin : g_probe
        always @* begin
            a_probe_cmp: assert (rdata_o[30:19] == ~rdata_o[11:0]);
            a_probe_par: assert (rdata_o[31] == ^rdata_o[18:0]);
            if (!$initstate) a_probe_pb: assert (p_pb == q_pb);

            if (rst_n) begin
                a_d_men: assert (p_men == e_men);
                a_d_wen: assert (p_wen == e_wen);
                a_d_ren: assert (p_ren == e_ren);
                if (e_men) a_d_addr: assert (p_addr == e_addr);
                if (e_wen) begin
                    a_d_din: assert (p_din == e_din[p_pb]);
                    a_d_bm:  assert (p_bm  == e_bm[p_pb]);
                end
                if (STRICT_ZERO) begin
                    if (!e_men) a_z_addr: assert (p_addr == 9'h0);
                    if (!e_wen) begin
                        a_z_din: assert (p_din == 1'b0);
                        a_z_bm:  assert (p_bm  == 1'b0);
                    end
                end
            end else begin
`ifndef MEM_NO_RSTCHK   // diagnostic only: shows the out-of-reset killer of a mutant
                a_d_rst: assert (d_exact || d_idle);
`endif
            end

            // ---- 3.6 item 1: region confinement, every cycle
            if (!hp && p_men) a_p1_region: assert (p_addr[8:7] == o);
            // ---- item 2
            if (rst_n && hp) a_p2_lane_ign: assert (p_men == h_en_i &&
                (!h_en_i || (p_addr == h_addr_i && p_wen == h_we_i && p_ren == !h_we_i)));
            if (rst_n && !hp) a_p2_host_ign: assert (p_men == e_men && p_wen == e_wen && p_ren == e_ren);
            // ---- item 4 (access part)
            if (rst_n && !hp && ((dm_op_i == OP_POP && m_qe[o]) || (dm_op_i == OP_PUSH && m_qf[o])))
                a_p4_noacc: assert (!p_men && !p_wen && !p_ren);
            // ---- item 6: queue accesses inside their queue
            if (rst_n && e_pop)
                a_p6_in:  assert (p_addr[8:7] == o && p_addr[6:0] < f_size(ob_isz_o));
            if (rst_n && e_push)
                a_p6_out: assert (p_addr[8:7] == o &&
                                  {1'b0, p_addr[6:0]} >= (8'd128 - {1'b0, f_size(ob_osz_o)}));
        end
    end endgenerate

    // =====================================================================
    // [P] Port-level properties with history (both modes)
    // =====================================================================
    logic [15:0] q_qcfg;
    logic [27:0] q_in_wr, q_in_rd, q_out_wr, q_out_rd;
    logic [3:0]  q_run, q_w_qcfg, q_w_qin, q_w_qout;
    logic [3:0]  q_popT, q_pushT, q_popE, q_pushF;
    logic [31:0] q_hwd;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_qcfg <= '0; q_in_wr <= '0; q_in_rd <= '0; q_out_wr <= '0; q_out_rd <= '0;
            q_run <= '0; q_w_qcfg <= '0; q_w_qin <= '0; q_w_qout <= '0;
            q_popT <= '0; q_pushT <= '0; q_popE <= '0; q_pushF <= '0; q_hwd <= '0;
        end else begin
            q_qcfg <= obs_qcfg_o; q_in_wr <= obs_in_wr_o; q_in_rd <= obs_in_rd_o;
            q_out_wr <= obs_out_wr_o; q_out_rd <= obs_out_rd_o;
            q_run <= run_i; q_hwd <= hreg_wdata_i;
            // any hreg_we_i in ph = 1 to that address (the gated reading);
            // with HREG_PH_GATED = 0 the assumption makes it the same thing
            q_w_qcfg <= w_qcfg; q_w_qin <= w_qin; q_w_qout <= w_qout;
            for (int l = 0; l < 4; l++) begin
                // lane l's D cycle, judged with the block's own flags
                q_popT[l]  <= !hp && o == l && dm_op_i == OP_POP  && !qe_o[l];
                q_popE[l]  <= !hp && o == l && dm_op_i == OP_POP  &&  qe_o[l];
                q_pushT[l] <= !hp && o == l && dm_op_i == OP_PUSH && !qf_o[l];
                q_pushF[l] <= !hp && o == l && dm_op_i == OP_PUSH &&  qf_o[l];
            end
        end
    end

    // Per-lane checks: computed as one bit per lane in the generate loop and
    // asserted once below (Yosys does not prefix procedural assertion labels
    // with the generate scope, so labels inside the loop would collide).
    logic [3:0] k_p8_qe, k_p8_qf, k_p8_qne, k_p8_qin_rd, k_p8_qout_rd,
                k_p8_rng_in, k_p8_rng_out,
                k_p5_in_rd, k_p5_out_wr, k_p5_in_wr, k_p5_out_rd, k_p5_qcfg,
                k_p5_pop_nx, k_p5_push_nx, k_p5_qin_rd, k_p5_qout_wr,
                k_p4_noptr, k_p7_noover, k_p7_nounder;
    wire state_same = obs_qcfg_o == q_qcfg && obs_in_wr_o == q_in_wr &&
                      obs_in_rd_o == q_in_rd && obs_out_wr_o == q_out_wr &&
                      obs_out_rd_o == q_out_rd;

    genvar gl;
    generate for (gl = 0; gl < 4; gl++) begin : g_lane
        wire [1:0] isz  = obs_qcfg_o[4*gl +: 2];
        wire [1:0] osz  = obs_qcfg_o[4*gl + 2 +: 2];
        wire [6:0] iw   = obs_in_wr_o[7*gl +: 7];
        wire [6:0] ir   = obs_in_rd_o[7*gl +: 7];
        wire [6:0] ow   = obs_out_wr_o[7*gl +: 7];
        wire [6:0] orr  = obs_out_rd_o[7*gl +: 7];
        wire [1:0] qisz = q_qcfg[4*gl +: 2];
        wire [1:0] qosz = q_qcfg[4*gl + 2 +: 2];
        wire [6:0] qiw  = q_in_wr[7*gl +: 7];
        wire [6:0] qir  = q_in_rd[7*gl +: 7];
        wire [6:0] qow  = q_out_wr[7*gl +: 7];
        wire [6:0] qor  = q_out_rd[7*gl +: 7];
        wire [6:0] si   = f_size(isz),  so  = f_size(osz);
        wire [6:0] qsi  = f_size(qisz), qso = f_size(qosz);
        wire [6:0] ci   = f_cnt(iw, ir, isz),    co  = f_cnt(ow, orr, osz);
        wire [6:0] qci  = f_cnt(qiw, qir, qisz), qco = f_cnt(qow, qor, qosz);

        // ---- item 8: flags against the count
        assign k_p8_qe[gl]  = qe_o[gl]  == (ci == 7'd0);
        assign k_p8_qf[gl]  = qf_o[gl]  == (co == so);
        assign k_p8_qne[gl] = qne_o[gl] == (co != 7'd0);
        assign k_p8_qin_rd[gl]  = hreg_addr_i != f_qin(gl)  || hreg_rdata_o[17:16] == {ci == si, ci == 7'd0};
        assign k_p8_qout_rd[gl] = hreg_addr_i != f_qout(gl) || hreg_rdata_o[17:16] == {co == so, co == 7'd0};
        // count stays in [0, S] without host writes to this queue
        assign k_p8_rng_in[gl]  = !(rv1 && qci <= qsi && !q_w_qin[gl]  && !q_w_qcfg[gl]) || ci <= si;
        assign k_p8_rng_out[gl] = !(rv1 && qco <= qso && !q_w_qout[gl] && !q_w_qcfg[gl]) || co <= so;
        // ---- item 5: pointer ownership
        assign k_p5_in_rd[gl]  = !(rv1 && ir != qir)  || q_popT[gl]  || (q_w_qin[gl]  && !q_run[gl]);
        assign k_p5_out_wr[gl] = !(rv1 && ow != qow)  || q_pushT[gl] || (q_w_qout[gl] && !q_run[gl]);
        assign k_p5_in_wr[gl]  = !(rv1 && iw != qiw)  || q_w_qin[gl];
        assign k_p5_out_rd[gl] = !(rv1 && orr != qor) || q_w_qout[gl];
        assign k_p5_qcfg[gl]   = !(rv1 && {osz, isz} != {qosz, qisz}) || q_w_qcfg[gl];
        // a taken pop / push advances the pointer by next()
        assign k_p5_pop_nx[gl]  = !(rv1 && q_popT[gl])  || ir == f_next(qir, qisz);
        assign k_p5_push_nx[gl] = !(rv1 && q_pushT[gl]) || ow == f_next(qow, qosz);
        // a host write of a lane-owned pointer: taken iff halted
        assign k_p5_qin_rd[gl]  = !(rv1 && q_w_qin[gl])  || ir == (q_run[gl] ? qir : q_hwd[14:8]);
        assign k_p5_qout_wr[gl] = !(rv1 && q_w_qout[gl]) || ow == (q_run[gl] ? qow : q_hwd[14:8]);
        // ---- item 4: POP empty / PUSH full change no pointer (none at all)
        assign k_p4_noptr[gl] = !(rv1 && (q_popE[gl] || q_pushF[gl])) || state_same;
        // ---- item 7: never overwrite an unread entry / read an unpublished
        //      one (count of the cycle before the operation)
        assign k_p7_noover[gl]  = !(rv1 && ow != qow && !q_w_qout[gl] && qco <= qso) || qco < qso;
        assign k_p7_nounder[gl] = !(rv1 && ir != qir && !q_w_qin[gl]) || qci != 7'd0;
    end endgenerate

    always @* begin
        a_p8_qe:      assert (&k_p8_qe);
        a_p8_qf:      assert (&k_p8_qf);
        a_p8_qne:     assert (&k_p8_qne);
        a_p8_qin_rd:  assert (&k_p8_qin_rd);
        a_p8_qout_rd: assert (&k_p8_qout_rd);
        a_p8_rng_in:  assert (&k_p8_rng_in);
        a_p8_rng_out: assert (&k_p8_rng_out);
        a_p5_in_rd:   assert (&k_p5_in_rd);
        a_p5_out_wr:  assert (&k_p5_out_wr);
        a_p5_in_wr:   assert (&k_p5_in_wr);
        a_p5_out_rd:  assert (&k_p5_out_rd);
        a_p5_qcfg:    assert (&k_p5_qcfg);
        a_p5_pop_nx:  assert (&k_p5_pop_nx);
        a_p5_push_nx: assert (&k_p5_push_nx);
        a_p5_qin_rd:  assert (&k_p5_qin_rd);
        a_p5_qout_wr: assert (&k_p5_qout_wr);
        a_p4_noptr:   assert (&k_p4_noptr);
        a_p7_noover:  assert (&k_p7_noover);
        a_p7_nounder: assert (&k_p7_nounder);
    end

    // =====================================================================
    // Tracked word (4.2): spec shadow and observed read data (both modes)
    // =====================================================================
    (* anyconst *) logic [8:0] ca;
    (* anyconst *) logic [4:0] tb_free;
    wire [4:0] tb = PROBE ? p_pb : tb_free;

    // cd: bit tb of the last value the spec wrote to ca since reset; ck: bit
    // tb was written; lwh: the last write to ca left bit tb alone (a half
    // store of the other half); lwk: kind of the last write to ca.
    logic        cd, ck, lwh, q_rdca, q_rdany;
    logic [2:0]  lwk, q_kind;
    logic [31:0] q_rdata;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cd <= 1'b0; ck <= 1'b0; lwh <= 1'b0; lwk <= K_NONE;
            q_rdca <= 1'b0; q_rdany <= 1'b0; q_kind <= K_NONE; q_rdata <= 32'h0;
        end else begin
            if (e_wen && e_addr == ca) begin
                if (e_bm[tb]) begin cd <= e_din[tb]; ck <= 1'b1; end
                lwh <= !e_bm[tb];
                lwk <= e_kind;
            end
            q_rdca  <= e_ren && (e_addr == ca);
            q_rdany <= e_ren;
            q_kind  <= e_kind;
            q_rdata <= rdata_o;
        end
    end

    // Macro model of 4.2 on the decoded pins (PROBE only): m_d = bit tb of
    // word ca, m_q = A_DOUT[tb]. No reset; arbitrary initial values.
    logic m_d, m_q;
    (* anyseq *) logic m_any;
    always_ff @(posedge clk) begin
        if (p_men && p_wen && (p_addr == ca) && p_bm) m_d <= p_din;
        if (p_men && p_ren && !p_wen)                 m_q <= (p_addr == ca) ? m_d : m_any;
    end

    wire obs_bit = PROBE ? m_q : rdata_o[tb];

    always @* begin
        if (q_rdca && ck) begin
            // ---- item 7: host-in word popped by the lane / host-out word
            //      read by the host / (and a plain MEM load)
            if (q_kind == K_POP) a_p7_pop:   assert (obs_bit == cd);
            if (q_kind == K_HRD) a_p7_hread: assert (obs_bit == cd);
            if (q_kind == K_LRD) a_p7_lread: assert (obs_bit == cd);
            // ---- item 9: a half store left the unselected half unchanged
            if (lwh)             a_p9_half:  assert (obs_bit == cd);
        end
        // ---- A_DOUT holds without a read (behavioural memory only)
        if (!PROBE && rv1 && !q_rdany) a_hold: assert (rdata_o == q_rdata);
        // ---- lemma for the unbounded proof (checker-internal)
        if (PROBE && ck) l_trk: assert (m_d == cd);
    end

    // =====================================================================
    // Covers (3.6 item 10, features, assumptions)
    // =====================================================================
    logic q_hacc, q_lwr_half, q_rst_n;
    always_ff @(posedge clk) begin
        q_hacc     <= rst_n && hp && h_en_i;
        q_lwr_half <= rst_n && e_kind == K_LWR && dm_hmask_i != 2'b11;
        q_rst_n    <= rst_n;
    end

    always @* begin
        // assumptions
        c_rst_rel:   cover (rv1);
        c_rst_again: cover (!$initstate && q_rst_n && !rst_n);   // mid-trace reset
        c_ph0: cover (rv1 && ph_i[0]);
        c_ph1: cover (rv1 && ph_i[1]);
        c_ph2: cover (rv1 && ph_i[2]);
        c_ph3: cover (rv1 && ph_i[3]);
        c_ph4: cover (rv1 && ph_i[4]);
        if (!HREG_PH_GATED) c_env_hreg: cover (rv1 && hreg_we_i && hp);
        // each isz / osz, used by a pop / push
        c_pop_isz0:  cover (rst_n && e_pop  && m_isz_o == 2'd0);
        c_pop_isz1:  cover (rst_n && e_pop  && m_isz_o == 2'd1);
        c_pop_isz2:  cover (rst_n && e_pop  && m_isz_o == 2'd2);
        c_pop_isz3:  cover (rst_n && e_pop  && m_isz_o == 2'd3);
        c_push_osz0: cover (rst_n && e_push && m_osz_o == 2'd0);
        c_push_osz1: cover (rst_n && e_push && m_osz_o == 2'd1);
        c_push_osz2: cover (rst_n && e_push && m_osz_o == 2'd2);
        c_push_osz3: cover (rst_n && e_push && m_osz_o == 2'd3);
        // wrap of both queues (masked pointer at m, next is 0)
        c_wrap_in:   cover (rst_n && e_pop  && (m_in_rd[o]  & f_m(m_isz_o)) == f_m(m_isz_o));
        c_wrap_out:  cover (rst_n && e_push && (m_out_wr[o] & f_m(m_osz_o)) == f_m(m_osz_o));
        c_wrap_in64: cover (rst_n && e_pop  && m_isz_o == 2'd3 && m_in_rd[o] == 7'd127);
        // full and empty; POP while empty, PUSH while full (no access)
        c_in_full:   cover (rv1 && m_fi[1]);
        c_out_full:  cover (rv1 && qf_o[2]);
        c_in_empty:  cover (rv1 && qe_o[3] && m_in_wr[3] != 7'd0);
        c_out_empty: cover (rv1 && !qne_o[0] && m_out_wr[0] != 7'd0);
        c_pop_empty: cover (rst_n && !hp && dm_op_i == OP_POP  && m_qe[o]);
        c_push_full: cover (rst_n && !hp && dm_op_i == OP_PUSH && m_qf[o]);
        // host write of a lane-owned pointer: halted (taken), running (ignored)
        c_qin_halt:  cover (rv1 && q_w_qin[1]  && !q_run[1] && q_in_rd[13:7] != obs_in_rd_o[13:7]);
        c_qin_run:   cover (rv1 && q_w_qin[1]  &&  q_run[1] && q_hwd[14:8] != q_in_rd[13:7]);
        c_qout_halt: cover (rv1 && q_w_qout[2] && !q_run[2] && q_out_wr[20:14] != obs_out_wr_o[20:14]);
        c_qout_run:  cover (rv1 && q_w_qout[2] &&  q_run[2] && q_hwd[14:8] != q_out_wr[20:14]);
        // host data access and a lane request in the same wheel turn
        c_same_turn: cover (rv1 && q_hacc && ph_i[2] && e_men);
        // FIFO: host word popped by the lane / lane word read by the host
        c_fifo_in:   cover (rv1 && q_rdca && ck && q_kind == K_POP && lwk == K_HWR && obs_bit == cd);
        c_fifo_out:  cover (rv1 && q_rdca && ck && q_kind == K_HRD && lwk == K_PUSH && obs_bit == cd);
        c_lread:     cover (rv1 && q_rdca && ck && q_kind == K_LRD && lwk == K_LWR && obs_bit == cd);
        // half store then read: the other half survived
        c_half:      cover (rv1 && q_rdca && ck && lwh && lwk == K_LWR && obs_bit == cd);
        // other request kinds
        c_wr_mask0:  cover (rst_n && !hp && dm_op_i == OP_WRITE && dm_hmask_i == 2'b00);
        c_op_5_7:    cover (rst_n && !hp && dm_op_i >= 3'd5);
        c_lane_ph1:  cover (rst_n && hp && dm_op_i != OP_NONE && !h_en_i);
        c_hen_noph:  cover (rst_n && !hp && h_en_i && dm_op_i == OP_NONE);
        if (HREG_PH_GATED) c_hreg_noph: cover (rst_n && !hp && hreg_we_i && hreg_addr_i == f_qin(2'd0));
        c_hold:      cover (rv1 && !q_rdany && rdata_o == q_rdata && q_kind == K_PUSH);
        // register reads
        c_rd_irq:    cover (rst_n && hreg_addr_i == A_IRQSTAT && hreg_rdata_o != 32'h0);
        c_rd_qin:    cover (rst_n && hreg_addr_i == f_qin(2'd3)  && hreg_rdata_o[17]);
        c_rd_qout:   cover (rst_n && hreg_addr_i == f_qout(2'd1) && hreg_rdata_o[17]);
        c_rd_qcfg:   cover (rst_n && hreg_addr_i == f_qcfg(2'd2) && hreg_rdata_o[3:0] == 4'hB);
    end
endmodule
