`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_mem_prog (program-memory wrapper).
//
//  Written from docs/blocks/mem.md (sections 1, 2, 4) and docs/decisions.md
//  ONLY (the RTL was not read). Properties are about the block's ports; the
//  macro pins are seen through rdata_o (see below).
//
//  Two modes (parameter PROBE, set by the proof top from `MEM_PROBE):
//
//   PROBE = 1 (task prove, unbounded). The macro is the probe model
//     formal/mem/mem_probe.sv: rdata_o (= A_DOUT, mem.md 2.2) carries a
//     self-checking encoding of the macro pins of the SAME cycle. The checker
//     decodes it and checks:
//       a_probe_*   rdata_o is exactly A_DOUT (no register, gate or change)
//       a_d_*       the drive table of mem.md 2.2, bit-exact, every cycle
//                   (A_DIN / A_BM through the probe's anyconst bit pb)
//       a_z_*       the 0 entries of that table (STRICT_ZERO, see readings)
//       a_p1/2/3_*  required properties 2.3 items 1-3, stated directly
//       a_p4_*      item 4 with the tracked-word model of mem.md 4.2 run by
//                   the checker on the decoded pins (tracked address ca,
//                   tracked bit pb)
//     The probe asserts A_DLY = 1, BIST pins 0, !(A_WEN & A_REN) itself.
//
//   PROBE = 0 (tasks bmc, cover). The real behavioural memory
//     (src/rosetta_sram512x32.sv with ROSETTA_SRAM_BEHAV). The pins are not
//     visible; the checker checks the end-to-end consequences on rdata_o:
//       a_p4_*      item 4: after the last write w to ca (computed from the
//                   ports by the table of 2.2), a fetch or host read of ca in
//                   cycle t gives rdata_o = w in cycle t+1 (arbitrary
//                   tracked address ca and bit tb: every bit of every word)
//       a_hold      A_DOUT holds in every cycle after one without a read
//                   (after a host write or an idle ph = 4 in particular)
//
//  Environment (assumed; each has a cover):
//   env_rst        chk_rst_n = 0 in the first cycle. chk_rst_n resets the
//                  checker only (the block has no reset and no flops); it is
//                  free afterwards (a later pulse only forgets the shadow).
//   env_ph_onehot  ph_i one-hot (sequencer.md: ph_o is the one-hot phase).
//                  Rotation is NOT assumed: the wrapper is combinational, so
//                  every property holds for any one-hot phase sequence.
//   h_en_i is NOT assumed to be 0 outside ph = 4 ("h_en_i outside ph = 4 is
//   ignored", 2.2): the drive must ignore it, and that is checked.
//
//  Spec readings (also in the report):
//   * "ph = k" is ph_i[k]; "ph in 0..3" is !ph_i[4] given one-hot.
//   * STRICT_ZERO = 1: the 0 entries of the 2.2 table (A_ADDR, A_DIN, A_BM in
//     rows where they are unused) are required. Set STRICT_ZERO = 0 to treat
//     them as don't-care (functionally harmless; they only save toggling).
// ===========================================================================

module mem_prog_props #(
    parameter PROBE       = 0,
    parameter STRICT_ZERO = 1
) (
    input wire         clk,
    input wire         chk_rst_n,
    input wire [4:0]   ph_i,
    input wire [8:0]   fetch_addr_i,
    input wire         h_en_i,
    input wire         h_we_i,
    input wire [8:0]   h_addr_i,
    input wire [31:0]  h_wdata_i,
    input wire [31:0]  rdata_o
);
    // =====================================================================
    // Environment
    // =====================================================================
    wire env_onehot = (ph_i == 5'b00001) || (ph_i == 5'b00010) || (ph_i == 5'b00100) ||
                      (ph_i == 5'b01000) || (ph_i == 5'b10000);
    always @* begin
        if ($initstate) env_rst: assume (!chk_rst_n);
        env_ph_onehot: assume (env_onehot);
    end

    // past-valid: chk_rst_n high now and in the previous cycle
    logic pv;
    always_ff @(posedge clk or negedge chk_rst_n)
        if (!chk_rst_n) pv <= 1'b0;
        else            pv <= 1'b1;

    // =====================================================================
    // Spec drive (mem.md 2.2), from the ports
    // =====================================================================
    wire        ph4    = ph_i[4];
    wire        e_lane = !ph4;                     // ph in 0..3: lane fetch
    wire        e_hacc = ph4 && h_en_i;            // host access
    wire        e_men  = e_lane || e_hacc;
    wire        e_wen  = e_hacc && h_we_i;
    wire        e_ren  = e_lane || (e_hacc && !h_we_i);
    wire [8:0]  e_addr = e_lane ? fetch_addr_i : (e_hacc ? h_addr_i : 9'h0);
    wire [31:0] e_din  = e_wen ? h_wdata_i : 32'h0;
    wire [31:0] e_bm   = e_wen ? 32'hFFFF_FFFF : 32'h0;

    // =====================================================================
    // Probe decode (PROBE = 1)
    // =====================================================================
    wire [8:0]  p_addr = rdata_o[8:0];
    wire        p_ren  = rdata_o[9];
    wire        p_wen  = rdata_o[10];
    wire        p_men  = rdata_o[11];
    wire        p_din  = rdata_o[12];               // A_DIN[pb]
    wire        p_bm   = rdata_o[13];               // A_BM[pb]
    wire [4:0]  p_pb   = rdata_o[18:14];

    logic [4:0] q_pb;
    always_ff @(posedge clk) q_pb <= p_pb;

    // Tracked word (mem.md 4.2): address ca, bit tb of it
    (* anyconst *) logic [8:0] ca;
    (* anyconst *) logic [4:0] tb_free;
    wire [4:0] tb = PROBE ? p_pb : tb_free;

    generate if (PROBE) begin : g_probe
        always @* begin
            a_probe_cmp: assert (rdata_o[30:19] == ~rdata_o[11:0]);
            a_probe_par: assert (rdata_o[31] == ^rdata_o[18:0]);
            if (pv) a_probe_pb: assert (p_pb == q_pb);

            // ---- drive table, bit-exact
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

            // ---- required properties 2.3, directly
            // 1: A_WEN only for a host write in ph = 4
            if (p_wen) a_p1_wen: assert (ph4 && h_en_i && h_we_i);
            // 2: never write-through (A_DLY / BIST: in mem_probe.sv)
            a_p2_wr_rd: assert (!(p_wen && p_ren));
            // 3: ph 0..3 read fetch_addr_i (h_en_i ignored there)
            if (!ph4) a_p3_fetch: assert (p_men && p_ren && !p_wen && p_addr == fetch_addr_i);
            // h_en_i outside ph = 4 ignored; host write needs ph = 4
            if (ph4 && !h_en_i) a_p_idle: assert (!p_men && !p_wen && !p_ren);
        end
    end endgenerate

    // =====================================================================
    // Tracked-word shadow (spec side) and observed read data
    // =====================================================================
    // cd: bit tb of the last value the spec writes to ca; ck: it was written
    // since the checker reset. Write data enters through the table of 2.2.
    logic cd, ck, q_rdca, q_rdlane, q_rdany, q_wr, q_idle4;
    logic [31:0] q_rdata;
    always_ff @(posedge clk or negedge chk_rst_n) begin
        if (!chk_rst_n) begin
            cd <= 1'b0; ck <= 1'b0;
            q_rdca <= 1'b0; q_rdlane <= 1'b0; q_rdany <= 1'b0;
            q_wr <= 1'b0; q_idle4 <= 1'b0; q_rdata <= 32'h0;
        end else begin
            if (e_wen && e_addr == ca && e_bm[tb]) begin
                cd <= e_din[tb];
                ck <= 1'b1;
            end
            q_rdca   <= e_ren && (e_addr == ca);
            q_rdlane <= e_lane;
            q_rdany  <= e_ren;
            q_wr     <= e_wen;
            q_idle4  <= ph4 && !h_en_i;
            q_rdata  <= rdata_o;
        end
    end

    // Macro model of mem.md 4.2 on the decoded pins (PROBE only): m_d is bit
    // tb of word ca in the array, m_q is A_DOUT[tb]. No reset (the macro has
    // none): arbitrary initial values.
    logic m_d, m_q;
    (* anyseq *) logic m_any;
    always_ff @(posedge clk) begin
        if (p_men && p_wen && (p_addr == ca) && p_bm) m_d <= p_din;
        if (p_men && p_ren && !p_wen)                 m_q <= (p_addr == ca) ? m_d : m_any;
    end

    wire obs_bit = PROBE ? m_q : rdata_o[tb];

    always @* begin
        // ---- required property 2.3 item 4 (both modes)
        if (q_rdca && ck) begin
            if (q_rdlane) a_p4_fetch: assert (obs_bit == cd);
            else          a_p4_hread: assert (obs_bit == cd);
        end
        // ---- A_DOUT holds without a read (behavioural memory only)
        if (!PROBE && pv && !q_rdany) a_hold: assert (rdata_o == q_rdata);
        // ---- lemma for the unbounded proof (checker-internal)
        if (PROBE && ck) l_trk: assert (m_d == cd);
    end

    // =====================================================================
    // Covers (2.3 item 5, features, assumptions)
    // =====================================================================
    always @* begin
        // assumptions
        c_rst_rel: cover (pv);
        c_ph0: cover (pv && ph_i[0]);
        c_ph1: cover (pv && ph_i[1]);
        c_ph2: cover (pv && ph_i[2]);
        c_ph3: cover (pv && ph_i[3]);
        c_ph4: cover (pv && ph_i[4]);
        // item 5: host write, then a lane fetch of the same word returns it
        c_wr_fetch: cover (q_rdca && q_rdlane && ck && obs_bit == cd);
        // item 5: host read (of a written word, data returned)
        c_hread:    cover (q_rdca && !q_rdlane && ck && obs_bit == cd);
        // item 5: ph = 4 with no host access
        c_idle4:    cover (pv && ph4 && !h_en_i);
        // the cycle after a host write / idle ph = 4 keeps the old word
        c_hold_wr:    cover (pv && q_wr && ph_i[0] && rdata_o == q_rdata);
        c_hold_idle4: cover (pv && q_idle4 && ph_i[0] && rdata_o == q_rdata);
        // h_en_i outside ph = 4 (ignored)
        c_hen_noph: cover (pv && !ph4 && h_en_i && h_we_i);
    end
endmodule
