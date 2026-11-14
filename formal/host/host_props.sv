`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_host (SPI target host port).
//
//  Written from docs/blocks/host.md and docs/decisions.md ONLY (the RTL was
//  not read). Every property is about the block's ports; obs_* are ports.
//
//  Contents
//   [M] Executable model of host.md section 3 (the nine flops of 3.1, rules
//       3.2-3.4), driven from the input ports and compared with the outputs
//       in EVERY cycle, reset cycles included (a_m_*). Exact for ANY input
//       sequence (free SPI pins), given a one-hot rotating ph_i.
//   [P] Required properties 1-9 of section 7, stated again directly on the
//       ports without the model (a_p1_* ... a_p9_*, a_rst_*, a_rel_*).
//   [E] Required property 10: a protocol-legal SPI master (env_* rules, see
//       below) drives sclk_i/csn_i/mosi_i directly (ruling: the pins arrive
//       already synchronised from the crossbar). Every frame carries free
//       (symbolic) command and data bits; the master side records what it
//       sent at the rising edges it produced (e_cmd, e_dat) and keeps a
//       one-deep scoreboard of the request the frame must cause (q_*), the
//       value the register bus / memory presented at the specified instant
//       (vr) and port-level shadows of MADDR / IRQMASK (s_maddr, s_imask).
//       This is the symbolic-transaction technique with EVERY frame tracked:
//       under legal timing at most one frame and one request are in flight,
//       so the scoreboard is one entry deep. Assertions a_e_*:
//         a_e_strobe   the exact strobe vector in every cycle: one strobe per
//                      complete write (or MDATA read), in the first cycle of
//                      the right wheel phase after the request, nothing else
//         a_e_wr_*     address and data of every write = the frame's bits
//         a_e_miso     MISO bit-exact in every cycle of every frame: bit
//                      48-k of the served value from the falling edge after
//                      rising edge k-1 to the falling edge after rising edge k
//                      (k = 17..48), 0 at every other time
//         a_e_hreg_addr, a_e_rd_addr, a_e_irq, a_e_one_eff, a_e_wr_abort
//       l_* are checker-internal lemmas tying the scoreboard to the model;
//       they make the end-to-end properties k-inductive (prove is unbounded,
//       no 800-cycle unrolling).
//   Covers (c_*): u_env (real timing) covers every SPI timing rule at its
//   boundary and a command byte + service at the fastest legal SCLK;
//   COVER_ONLY instances (time-compressed master) cover whole frames of
//   every kind, back-to-back MDATA writes, aborts at edge 20, extra clocks,
//   IRQ raise/clear; the free-pin instance covers every request kind, each
//   IRQ source, the capture-vs-shift priority case, MADDR wrap, mid-frame
//   reset.
//
//  Environment (STANDALONE = 1: assumed; 0: asserted / used as a gate)
//   env_rst        rst_n = 0 in the first cycle (flops start arbitrary)
//   env_ph_onehot  ph_i one-hot
//   env_ph_rot     ph_i(t) = rotate(ph_i(t-1)) while out of reset (the wheel,
//                  sequencer.md property 1); the phase at reset release is
//                  arbitrary, so the master has arbitrary phase relative to it
//   ENV_SPI = 1 only (legal master, host.md 2.1 translated to the
//   synchronised pins; T = one clk cycle; the synchronisers of the two pins
//   may resolve one cycle apart, so each pin-level bound loses one cycle):
//   env_half       SCLK high and low time >= HALF (7) cycles
//   env_cs_rise    CS_n rises only with SCLK low for >= CS_GAP (7) cycles
//                  since the last falling edge
//   env_cs_fall    CS_n falls only with SCLK low, after CS_n high >= CS_GAP
//   env_cs_su      a rising edge in a frame comes >= CS_GAP cycles after CS_n
//                  fell
//   env_mosi_su    MOSI unchanged in the MOSI_SU (3) cycles before a rising
//                  edge and in the rising-edge cycle
//   env_mosi_hd    MOSI unchanged in the MOSI_HD (3) cycles after a rising
//                  edge
//   SCLK may toggle while CS_n is high (shared bus); frames may be cut at any
//   point (CS_n rises early: abort) or carry extra clocks; there is no upper
//   bound on any time. With ENV_SPI = 0 the three SPI pins are completely
//   free (any toggle rate, glitches, CS_n mid-bit) and only [M] and [P] are
//   checked.
//
//  Spec readings that matter (also in the report):
//   * h_pm_addr_o / h_pm_wdata_o / h_dm_addr_o / h_dm_wdata_o / hreg_wdata_o
//     are checked in the cycles their enable/strobe is 1; "All outputs not
//     listed are 0" (3.3) is read as covering the en/we strobes only, so the
//     address/data buses are don't-care otherwise. hreg_addr_o = cmd[6:0] is
//     checked in every cycle (port table: "always").
//   * "ph = k" is ph_i[k] (one-hot).
//   * RDATA(MADDR) = {22'b0, maddr}, RDATA(IRQMASK) = {24'b0, imask}.
//   * irq uses the imask of the same cycle (the old value in the cycle an
//     IRQMASK write is served).
//   * MISO returns to 0 at the falling edge after rising edge 48 (the 3.2
//     rule and the worked timing), not at rising edge 48 (3.2 prose).
//   * A MADDR / IRQMASK write still raises hreg_we_o (it is a REG write in
//     the 3.3 table); writes to ID raise hreg_we_o and change nothing here.
//   * hst = ISSUED with a REG command has no row in 3.3; the model holds hst.
//     Unreachable with a rotating ph_i (cmd cannot change within 6 cycles of
//     a request), so no design choice is checked there.
// ===========================================================================

module host_props #(
    // 1: stand-alone proof (environment assumed, covers active).
    // 0: monitor inside a larger proof (ph rules asserted, SPI legality used
    //    only as a gate of the end-to-end checks, covers off).
    parameter STANDALONE = 1,
    // 1: legal SPI master ([E] active). 0: free SPI pins ([M], [P] only).
    parameter ENV_SPI    = 1,
    // Legal timing at the synchronised pins (cycles of clk). The lemmas need
    // HALF >= 7 (a request is served and its read data captured within 6
    // cycles, before the next SCLK edge).
    parameter HALF       = 7,
    parameter CS_GAP     = 7,
    parameter MOSI_SU    = 3,
    parameter MOSI_HD    = 3,
    // 1: cover-only instance (no assertion is active; environment assumed;
    //    frame-level covers). Used with a compressed time scale.
    parameter COVER_ONLY = 0
) (
    input wire         clk,
    input wire         rst_n,
    input wire [4:0]   ph_i,
    input wire         sclk_i,
    input wire         csn_i,
    input wire         mosi_i,
    input wire         miso_o,
    input wire         irq_o,
    input wire         h_pm_en_o,
    input wire         h_pm_we_o,
    input wire [8:0]   h_pm_addr_o,
    input wire [31:0]  h_pm_wdata_o,
    input wire [31:0]  pm_rdata_i,
    input wire         h_dm_en_o,
    input wire         h_dm_we_o,
    input wire [8:0]   h_dm_addr_o,
    input wire [31:0]  h_dm_wdata_o,
    input wire [31:0]  dm_rdata_i,
    input wire         hreg_we_o,
    input wire [6:0]   hreg_addr_o,
    input wire [31:0]  hreg_wdata_o,
    input wire [31:0]  hreg_rdata_i,
    input wire [3:0]   attn_i,
    input wire [3:0]   qne_i,
    input wire [31:0]  obs_sr_o,
    input wire [7:0]   obs_cmd_o,
    input wire [5:0]   obs_bitcnt_o,
    input wire [9:0]   obs_maddr_o,
    input wire [1:0]   obs_hst_o,
    input wire [7:0]   obs_imask_o
);
    localparam [6:0]  A_ID      = 7'h00;
    localparam [6:0]  A_MADDR   = 7'h01;
    localparam [6:0]  A_MDATA   = 7'h02;
    localparam [6:0]  A_IRQMASK = 7'h05;
    localparam [31:0] ID_VAL    = 32'h52530401;
    localparam [1:0]  IDLE = 2'd0, PEND = 2'd1, ISSUED = 2'd2;

    // =====================================================================
    // Common helpers
    // =====================================================================
    // rv1: rst_n was 1 in the previous cycle and is 1 now. rcnt: cycles
    // since reset release (saturating), for multi-cycle windows.
    logic       rv1;
    logic [2:0] rcnt;
    logic       c_sp;          // sclk_i one cycle earlier (= spec sclk_prev)
    logic [4:0] ph_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rv1  <= 1'b0;
            rcnt <= 3'd0;
            c_sp <= 1'b0;
            ph_q <= 5'd0;
        end else begin
            rv1  <= 1'b1;
            rcnt <= (rcnt == 3'd7) ? rcnt : rcnt + 3'd1;
            c_sp <= sclk_i;
            ph_q <= ph_i;
        end
    end
    wire rise = sclk_i && !c_sp;
    wire fall = !sclk_i && c_sp;

    // ---------------------------------------------------------------------
    // Environment: wheel
    // ---------------------------------------------------------------------
    wire env_onehot = (ph_i == 5'b00001) || (ph_i == 5'b00010) || (ph_i == 5'b00100) ||
                      (ph_i == 5'b01000) || (ph_i == 5'b10000);
    wire env_rot    = !rv1 || (ph_i == {ph_q[3:0], ph_q[4]});
    generate
        if (STANDALONE) begin : g_env_ph_assume
            always @* begin
                if ($initstate) env_rst: assume (!rst_n);
                env_ph_onehot: assume (env_onehot);
                env_ph_rot:    assume (env_rot);
            end
        end else begin : g_env_ph_assert
            always @* begin
                if ($initstate) env_rst: assert (!rst_n);
                env_ph_onehot: assert (env_onehot);
                env_ph_rot:    assert (env_rot);
            end
        end
    endgenerate

    // =====================================================================
    // [M] Executable model of section 3
    // =====================================================================
    logic [31:0] m_sr;
    logic [5:0]  m_bc;
    logic [7:0]  m_cmd;
    logic        m_miso;
    logic [9:0]  m_maddr;
    logic [1:0]  m_hst;
    logic [7:0]  m_imask;
    logic        m_irq;

    wire [6:0] m_k     = {1'b0, m_bc} + 7'd1;          // edge number
    wire       m_rd    = m_cmd[7];
    wire [6:0] m_a     = m_cmd[6:0];
    wire       m_md    = (m_a == A_MDATA);
    wire       m_prog  = m_md && !m_maddr[9];
    wire       m_data  = m_md &&  m_maddr[9];
    wire       m_reg   = !m_md;
    wire       m_serve = (m_hst == PEND) && (m_prog ? ph_i[4] : ph_i[1]);
    wire       m_cap   = (m_hst == ISSUED) && ((m_prog && ph_i[0]) || (m_data && ph_i[2]));
    wire       m_rdreg = m_serve && m_reg && m_rd;      // REG read service
    wire       m_wrreg = m_serve && m_reg && !m_rd;     // REG write service
    wire [31:0] m_rdata = (m_a == A_ID)      ? ID_VAL :
                          (m_a == A_MADDR)   ? {22'h0, m_maddr} :
                          (m_a == A_IRQMASK) ? {24'h0, m_imask} : hreg_rdata_i;
    wire       m_edge  = !csn_i && rise;
    wire       m_shin  = m_edge && ((m_k <= 7'd8) ||
                                    (m_k >= 7'd9 && m_k <= 7'd40 && !m_rd));
    wire       m_sh0   = m_edge && (m_k >= 7'd17 && m_k <= 7'd47 && m_rd);
    wire       m_req   = m_edge && ((m_k == 7'd8 && m_sr[6]) || (m_k == 7'd40 && !m_rd));
    wire       m_mison = (m_rd && m_bc >= 6'd16 && m_bc <= 6'd47) ? m_sr[31] : 1'b0;
    wire       m_irqn  = |(attn_i & m_imask[3:0]) || |(qne_i & m_imask[7:4]);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_sr    <= 32'h0;
            m_bc    <= 6'd0;
            m_cmd   <= 8'h0;
            m_miso  <= 1'b0;
            m_maddr <= 10'h0;
            m_hst   <= IDLE;
            m_imask <= 8'h0;
            m_irq   <= 1'b0;
        end else begin
            // bitcnt
            if (csn_i)       m_bc <= 6'd0;
            else if (rise)   m_bc <= (m_k > 7'd48) ? 6'd48 : m_k[5:0];
            // cmd
            if (m_edge && m_k == 7'd8) m_cmd <= {m_sr[6:0], mosi_i};
            // sr: capture beats shift (3.2 last bullet)
            if (m_rdreg)     m_sr <= m_rdata;
            else if (m_cap)  m_sr <= m_prog ? pm_rdata_i : dm_rdata_i;
            else if (m_shin) m_sr <= {m_sr[30:0], mosi_i};
            else if (m_sh0)  m_sr <= {m_sr[30:0], 1'b0};
            // miso
            if (csn_i)       m_miso <= 1'b0;
            else if (fall)   m_miso <= m_mison;
            // maddr
            if (m_wrreg && m_a == A_MADDR)  m_maddr <= m_sr[9:0];
            else if (m_serve && m_md)       m_maddr <= {m_maddr[9], m_maddr[8:0] + 9'd1};
            // imask
            if (m_wrreg && m_a == A_IRQMASK) m_imask <= m_sr[7:0];
            // hst
            if (m_hst == IDLE && m_req) m_hst <= PEND;
            else if (m_serve)           m_hst <= (m_rd && m_md) ? ISSUED : IDLE;
            else if (m_cap)             m_hst <= IDLE;
            // irq
            m_irq <= m_irqn;
        end
    end

    wire m_pm_en = m_serve && m_prog;
    wire m_dm_en = m_serve && m_data;

`ifndef HOST_NO_MODEL_CHECK
    always @* if (!COVER_ONLY) begin
        a_m_sr:     assert (obs_sr_o     == m_sr);
        a_m_bc:     assert (obs_bitcnt_o == m_bc);
        a_m_cmd:    assert (obs_cmd_o    == m_cmd);
        a_m_maddr:  assert (obs_maddr_o  == m_maddr);
        a_m_hst:    assert (obs_hst_o    == m_hst);
        a_m_imask:  assert (obs_imask_o  == m_imask);
        a_m_miso:   assert (miso_o       == m_miso);
        a_m_irq:    assert (irq_o        == m_irq);
        a_m_pm_en:  assert (h_pm_en_o    == m_pm_en);
        a_m_pm_we:  assert (h_pm_we_o    == (m_pm_en && !m_rd));
        a_m_dm_en:  assert (h_dm_en_o    == m_dm_en);
        a_m_dm_we:  assert (h_dm_we_o    == (m_dm_en && !m_rd));
        a_m_hreg_we:   assert (hreg_we_o   == m_wrreg);
        a_m_hreg_addr: assert (hreg_addr_o == m_a);
        if (m_pm_en) begin
            a_m_pm_addr:  assert (h_pm_addr_o  == m_maddr[8:0]);
            a_m_pm_wdata: assert (h_pm_wdata_o == m_sr);
        end
        if (m_dm_en) begin
            a_m_dm_addr:  assert (h_dm_addr_o  == m_maddr[8:0]);
            a_m_dm_wdata: assert (h_dm_wdata_o == m_sr);
        end
        if (m_wrreg) begin
            a_m_hreg_wdata: assert (hreg_wdata_o == m_sr);
        end
    end
`endif

    // =====================================================================
    // [P] Required properties on the ports, without the model
    // =====================================================================
    // Registered copies of port values (previous cycle).
    logic        q_miso, q_csn, q_rise, q_fall, q_we_maddr, q_we_imask, q_memacc, q_cmd7;
    logic [31:0] q_wdata, q_sr;
    logic [9:0]  q_maddr;
    logic [7:0]  q_imask;
    logic [5:0]  q_bc;
    logic [1:0]  q_hst;
    logic [3:0]  q_attn, q_qne;
    logic        q_reqok;      // a request was allowed by 3.2 in the previous cycle
    logic        q_mosi, q_rdreg, q_capm;
    logic [7:0]  q_cmd;
    logic [31:0] q_rdval, q_capv;
    logic        req_seen;     // a request entered PEND in this CS_n-low period
    logic [2:0]  pend_run;     // consecutive cycles with hst = PEND (sat 7)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_miso <= 1'b0; q_csn <= 1'b1; q_rise <= 1'b0; q_fall <= 1'b0;
            q_we_maddr <= 1'b0; q_we_imask <= 1'b0; q_memacc <= 1'b0; q_cmd7 <= 1'b0;
            q_wdata <= 32'h0; q_sr <= 32'h0; q_maddr <= 10'h0; q_imask <= 8'h0;
            q_bc <= 6'd0; q_hst <= IDLE; q_attn <= 4'h0; q_qne <= 4'h0;
            q_reqok <= 1'b0; req_seen <= 1'b0; pend_run <= 3'd0;
            q_mosi <= 1'b0; q_rdreg <= 1'b0; q_capm <= 1'b0; q_cmd <= 8'h0;
            q_rdval <= 32'h0; q_capv <= 32'h0;
        end else begin
            q_miso     <= miso_o;
            q_csn      <= csn_i;
            q_rise     <= rise;
            q_fall     <= fall;
            q_we_maddr <= hreg_we_o && hreg_addr_o == A_MADDR;
            q_we_imask <= hreg_we_o && hreg_addr_o == A_IRQMASK;
            q_memacc   <= h_pm_en_o || h_dm_en_o;
            q_cmd7     <= obs_cmd_o[7];
            q_wdata    <= hreg_wdata_o;
            q_sr       <= obs_sr_o;
            q_maddr    <= obs_maddr_o;
            q_imask    <= obs_imask_o;
            q_bc       <= obs_bitcnt_o;
            q_hst      <= obs_hst_o;
            q_attn     <= attn_i;
            q_qne      <= qne_i;
            q_reqok    <= !csn_i && rise &&
                          ((obs_bitcnt_o == 6'd7  && obs_sr_o[6]) ||
                           (obs_bitcnt_o == 6'd39 && !obs_cmd_o[7]));
            req_seen   <= csn_i ? 1'b0 : (req_seen || p_entry);
            q_mosi     <= mosi_i;
            q_cmd      <= obs_cmd_o;
            // REG read service / memory read capture, from ports (3.3)
            q_rdreg    <= obs_hst_o == PEND && obs_cmd_o[7] && obs_cmd_o[6:0] != A_MDATA && ph_i[1];
            q_rdval    <= (obs_cmd_o[6:0] == A_ID)      ? ID_VAL :
                          (obs_cmd_o[6:0] == A_MADDR)   ? {22'h0, obs_maddr_o} :
                          (obs_cmd_o[6:0] == A_IRQMASK) ? {24'h0, obs_imask_o} : hreg_rdata_i;
            q_capm     <= obs_hst_o == ISSUED && (obs_maddr_o[9] ? ph_i[2] : ph_i[0]);
            q_capv     <= obs_maddr_o[9] ? dm_rdata_i : pm_rdata_i;
            pend_run   <= (obs_hst_o != PEND) ? 3'd0 :
                          (pend_run == 3'd7) ? pend_run : pend_run + 3'd1;
        end
    end
    wire p_entry = rv1 && q_hst == IDLE && obs_hst_o == PEND;   // request accepted last cycle
    wire [6:0]  q_k    = {1'b0, q_bc} + 7'd1;
    wire        q_edge = !q_csn && q_rise;
    wire [31:0] q_sr_n = (q_edge && (q_k <= 7'd8 || (q_k <= 7'd40 && !q_cmd7))) ? {q_sr[30:0], q_mosi} :
                         (q_edge && q_k >= 7'd17 && q_k <= 7'd47 && q_cmd7)    ? {q_sr[30:0], 1'b0} :
                                                                                 q_sr;
    wire any_en  = h_pm_en_o || h_dm_en_o || hreg_we_o;

    always @* if (!COVER_ONLY) begin
        // ---- reset: every output and observed flop at its reset value,
        //      in every reset cycle (flops start arbitrary) ----
        if (!rst_n) begin
            a_rst_miso:  assert (!miso_o);
            a_rst_irq:   assert (!irq_o);
            a_rst_en:    assert (!h_pm_en_o && !h_pm_we_o && !h_dm_en_o && !h_dm_we_o && !hreg_we_o);
            a_rst_addr:  assert (hreg_addr_o == 7'h0);
            a_rst_obs:   assert (obs_sr_o == 32'h0 && obs_cmd_o == 8'h0 && obs_bitcnt_o == 6'd0 &&
                                 obs_maddr_o == 10'h0 && obs_hst_o == IDLE && obs_imask_o == 8'h0);
        end
        // ---- first cycle after release: still the reset state ----
        if (rst_n && !rv1) begin
            a_rel_miso:  assert (!miso_o);
            a_rel_irq:   assert (!irq_o);
            a_rel_en:    assert (!any_en);
            a_rel_obs:   assert (obs_sr_o == 32'h0 && obs_cmd_o == 8'h0 && obs_bitcnt_o == 6'd0 &&
                                 obs_maddr_o == 10'h0 && obs_hst_o == IDLE && obs_imask_o == 8'h0);
        end
        if (rst_n) begin
            // ---- 1. slot discipline ----
            a_p1_pm_slot:  assert (!h_pm_en_o || ph_i[4]);
            a_p1_dm_slot:  assert (!h_dm_en_o || ph_i[1]);
            a_p1_reg_slot: assert (!hreg_we_o || ph_i[1]);
            a_p1_onehot:   assert ((h_pm_en_o + h_dm_en_o + hreg_we_o) <= 2'd1);
            a_p1_pend:     assert (!any_en || obs_hst_o == PEND);
            a_p1_pm_we:    assert (!h_pm_we_o || h_pm_en_o);
            a_p1_dm_we:    assert (!h_dm_we_o || h_dm_en_o);
            a_p1_pm_space: assert (!h_pm_en_o || !obs_maddr_o[9]);
            a_p1_dm_space: assert (!h_dm_en_o ||  obs_maddr_o[9]);
            a_p1_reg_kind: assert (!hreg_we_o || hreg_addr_o != A_MDATA);
            a_p1_md_kind:  assert (!(h_pm_en_o || h_dm_en_o) || hreg_addr_o == A_MDATA);
            a_p1_hst_enc:  assert (obs_hst_o != 2'd3);
            // ---- 3. the strobe's address / data come from maddr / sr ----
            if (h_pm_en_o) begin
                a_p3_pm_addr:  assert (h_pm_addr_o == obs_maddr_o[8:0]);
            end
            if (h_pm_we_o) begin
                a_p3_pm_wdata: assert (h_pm_wdata_o == obs_sr_o);
            end
            if (h_dm_en_o) begin
                a_p3_dm_addr:  assert (h_dm_addr_o == obs_maddr_o[8:0]);
            end
            if (h_dm_we_o) begin
                a_p3_dm_wdata: assert (h_dm_wdata_o == obs_sr_o);
            end
            if (hreg_we_o) begin
                a_p3_reg_wdata: assert (hreg_wdata_o == obs_sr_o);
            end
            a_p3_reg_addr: assert (hreg_addr_o == obs_cmd_o[6:0]);
            // ---- 2. one effect per frame ----
            if (p_entry) begin
                a_p2_req_edge: assert (q_reqok);     // only at edge 8 (read) / 40 (write)
                a_p2_one_req:  assert (!req_seen);   // at most one per CS_n-low period
            end
            if (req_seen) begin               // after the frame's request, no edge 8 / 40 can come
                a_p2_seen:     assert (obs_bitcnt_o >= 6'd8 && (obs_cmd_o[7] || obs_bitcnt_o >= 6'd40));
            end
            if (rv1 && q_hst != IDLE) begin
                a_p2_no_req:   assert (obs_hst_o != PEND || q_hst == PEND);  // no re-arm
            end
            // frame receive counters (3.2): edges counted, saturating at 48,
            // cleared while CS_n is high; command byte taken at edge 8
            if (rv1) begin
                a_p2_bitcnt:   assert (obs_bitcnt_o == (q_csn ? 6'd0 :
                                                        q_rise ? ((q_bc >= 6'd48) ? 6'd48 : q_bc + 6'd1) :
                                                                 q_bc));
                a_p2_cmd:      assert (obs_cmd_o == ((!q_csn && q_rise && q_bc == 6'd7) ?
                                                     {q_sr[6:0], q_mosi} : q_cmd));
            end
            a_p2_bc_max:       assert (obs_bitcnt_o <= 6'd48);
            // ---- 3/4. sr: shift rules of 3.2 (MSB first; capture wins) ----
            if (rv1 && !q_rdreg && !q_capm) begin
                a_p3_sr:       assert (obs_sr_o == q_sr_n);
            end
            // ---- 4. read data reaches sr at the specified cycle ----
            if (rv1 && q_rdreg) begin
                a_p4_cap_reg:  assert (obs_sr_o == q_rdval && obs_hst_o == IDLE);
            end
            if (rv1 && q_capm) begin
                a_p4_cap_mem:  assert (obs_sr_o == q_capv && obs_hst_o == IDLE);
            end
            // ---- 5. MISO idle ----
            if (rv1 && q_csn) begin
                a_p5_cs_high:  assert (!miso_o);
            end
            if (rv1 && !q_csn && !q_fall) begin
                a_p5_hold:     assert (miso_o == q_miso);
            end
            if (rv1 && !q_csn && q_fall) begin
                a_p5_fall:     assert (miso_o == ((q_cmd7 && q_bc >= 6'd16 && q_bc <= 6'd47) ? q_sr[31] : 1'b0));
            end
            if (obs_bitcnt_o < 6'd16) begin
                a_p5_edges16:  assert (!miso_o);     // edges 1-16 of any frame
            end
            if (obs_bitcnt_o >= 6'd8 && !obs_cmd_o[7]) begin
                a_p5_write:    assert (!miso_o);     // a write frame after its command byte
            end
            // ---- 6. auto-increment / MADDR ----
            if (rv1) begin
                a_p6_maddr:    assert (obs_maddr_o == (q_we_maddr ? q_wdata[9:0] :
                                                       q_memacc   ? {q_maddr[9], q_maddr[8:0] + 9'd1} :
                                                                    q_maddr));
                a_p6_imask:    assert (obs_imask_o == (q_we_imask ? q_wdata[7:0] : q_imask));
            end
            // ---- 7. service latency ----
            if (obs_hst_o == PEND) begin
                a_p7_pend5:    assert (pend_run <= 3'd4);   // at most 5 PEND cycles in a row
            end
            if (rv1 && q_hst == ISSUED) begin
                a_p7_iss1:     assert (obs_hst_o == IDLE);
            end
            if (rv1 && obs_hst_o == ISSUED) begin
                a_p7_iss_from: assert (q_hst == PEND && q_memacc);
            end
            // ---- 8. IRQ ----
            if (rv1) begin
                a_p8_irq:      assert (irq_o == (|(q_attn & q_imask[3:0]) || |(q_qne & q_imask[7:4])));
            end
        end
    end

    // =====================================================================
    // [E] Reference SPI master and end-to-end properties (ENV_SPI = 1)
    // =====================================================================
    generate if (ENV_SPI) begin : g_spi
        // ---------------- master timing state ----------------
        logic       c_csp, c_mp;
        logic [3:0] sst, cst, mst;     // cycles the previous level has held
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                c_csp <= 1'b1; c_mp <= 1'b0;
                sst <= 4'd0; cst <= 4'd0; mst <= 4'd0;
            end else begin
                c_csp <= csn_i;
                c_mp  <= mosi_i;
                sst   <= (sclk_i != c_sp)  ? 4'd1 : (sst == 4'd15) ? sst : sst + 4'd1;
                cst   <= (csn_i  != c_csp) ? 4'd1 : (cst == 4'd15) ? cst : cst + 4'd1;
                mst   <= (mosi_i != c_mp)  ? 4'd1 : (mst == 4'd15) ? mst : mst + 4'd1;
            end
        end
        wire sc_chg = sclk_i != c_sp;
        wire cs_chg = csn_i  != c_csp;
        wire mo_chg = mosi_i != c_mp;
        wire env_half    = !sc_chg || sst >= HALF;
        wire env_cs_rise = !(cs_chg &&  csn_i) || (!sclk_i && !c_sp && sst >= CS_GAP);
        wire env_cs_fall = !(cs_chg && !csn_i) || (!sclk_i && !c_sp && cst >= CS_GAP);
        wire env_cs_su   = !(rise && !csn_i) || (!cs_chg && cst >= CS_GAP);
        wire env_mosi_su = !rise || (!mo_chg && mst >= MOSI_SU);
        wire env_mosi_hd = !mo_chg || !(sclk_i && (!c_sp || sst <= MOSI_HD));
        wire legal_now   = env_half && env_cs_rise && env_cs_fall && env_cs_su &&
                           env_mosi_su && env_mosi_hd;

        logic spi_ok;                  // legal since the last reset
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) spi_ok <= 1'b1;
            else        spi_ok <= spi_ok && legal_now;
        end
        wire chk = rst_n && spi_ok && legal_now;

        if (STANDALONE) begin : g_env_spi
            always @* begin
                if (rst_n) begin
                    env_half_a:    assume (env_half);
                    env_cs_rise_a: assume (env_cs_rise);
                    env_cs_fall_a: assume (env_cs_fall);
                    env_cs_su_a:   assume (env_cs_su);
                    env_mosi_su_a: assume (env_mosi_su);
                    env_mosi_hd_a: assume (env_mosi_hd);
                end
            end
        end

        // ---------------- what the master sent ----------------
        logic [5:0]  ek;               // rising edges in this frame (sat 63)
        logic [7:0]  e_cmd;            // bits of edges 1-8 (command byte)
        logic [31:0] e_dat;            // bits of edges 9-40 (write data D)
        wire e_edge = !csn_i && rise;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                ek <= 6'd0; e_cmd <= 8'h0; e_dat <= 32'h0;
            end else begin
                if (csn_i)       ek <= 6'd0;
                else if (rise)   ek <= (ek == 6'd63) ? ek : ek + 6'd1;
                if (e_edge && ek < 6'd8)                 e_cmd <= {e_cmd[6:0], mosi_i};
                if (e_edge && ek >= 6'd8 && ek < 6'd40)  e_dat <= {e_dat[30:0], mosi_i};
            end
        end
        // Request the frame must make (host.md 2): read at edge 8, write at
        // edge 40 (exactly 40 edges, the 40th completes D).
        wire e_req_rd = e_edge && ek == 6'd7  &&  e_cmd[6];
        wire e_req_wr = e_edge && ek == 6'd39 && !e_cmd[7];
        wire e_req    = e_req_rd || e_req_wr;

        // ---------------- scoreboard ----------------
        logic        q_v, q_iss;       // request pending / memory read issued
        logic [2:0]  q_age;            // cycles since the request
        logic [9:0]  s_maddr;          // MADDR as the host's frames define it
        logic [7:0]  s_imask;          // IRQMASK as the host's frames define it
        logic [31:0] vr;               // value served for the last read
        logic        x_irq;
        logic        f_eff;            // a strobe already happened in this frame
        wire [6:0]  x_a    = e_cmd[6:0];
        wire        x_rd   = e_cmd[7];
        wire        x_md   = (x_a == A_MDATA);
        wire        x_prog = x_md && !s_maddr[9];
        wire        x_data = x_md &&  s_maddr[9];
        wire        x_reg  = !x_md;
        wire        x_serve = q_v && (x_prog ? ph_i[4] : ph_i[1]);
        wire        x_cap   = q_iss && (x_prog ? ph_i[0] : ph_i[2]);
        wire [31:0] x_rdata = (x_a == A_ID)      ? ID_VAL :
                              (x_a == A_MADDR)   ? {22'h0, s_maddr} :
                              (x_a == A_IRQMASK) ? {24'h0, s_imask} : hreg_rdata_i;
        wire        q_v_n   = e_req ? 1'b1 : x_serve ? 1'b0 : q_v;
        wire        q_iss_n = (x_serve && x_rd && x_md) ? 1'b1 : x_cap ? 1'b0 : q_iss;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                q_v <= 1'b0; q_iss <= 1'b0; q_age <= 3'd0;
                s_maddr <= 10'h0; s_imask <= 8'h0; vr <= 32'h0; x_irq <= 1'b0; f_eff <= 1'b0;
            end else begin
                q_v   <= q_v_n;
                q_iss <= q_iss_n;
                q_age <= e_req ? 3'd1 : (q_v_n || q_iss_n) ? ((q_age == 3'd7) ? q_age : q_age + 3'd1) : 3'd0;
                if (x_serve && !x_rd && x_a == A_MADDR)   s_maddr <= e_dat[9:0];
                else if (x_serve && x_md)                 s_maddr <= {s_maddr[9], s_maddr[8:0] + 9'd1};
                if (x_serve && !x_rd && x_a == A_IRQMASK) s_imask <= e_dat[7:0];
                if (x_serve && x_rd && x_reg)             vr <= x_rdata;      // RDATA(A) at the service cycle
                else if (x_cap)                           vr <= x_prog ? pm_rdata_i : dm_rdata_i;
                x_irq <= |(attn_i & s_imask[3:0]) || |(qne_i & s_imask[7:4]);
                f_eff <= csn_i ? 1'b0 : (f_eff || any_en);
            end
        end

        // expected MISO: the edge whose bit is on the pin now
        wire [6:0] e_k   = c_sp ? {1'b0, ek} : {1'b0, ek} + 7'd1;
        wire       e_rdf = x_rd && ek >= 6'd8;
        wire [6:0] e_idx = 7'd48 - e_k;
        wire       e_miso = (e_rdf && e_k >= 7'd17 && e_k <= 7'd48) ? vr[e_idx[4:0]] : 1'b0;
        wire any_we = h_pm_we_o || h_dm_we_o || hreg_we_o;

        always @* if (!COVER_ONLY) begin
            if (chk) begin
                // (a) exactly one strobe per complete write (and per MDATA
                //     read), at the first cycle of the right phase, nothing else
                a_e_strobe:    assert ({h_pm_en_o, h_pm_we_o, h_dm_en_o, h_dm_we_o, hreg_we_o} ==
                                       {x_serve && x_prog, x_serve && x_prog && !x_rd,
                                        x_serve && x_data, x_serve && x_data && !x_rd,
                                        x_serve && x_reg && !x_rd});
                // write address and data = the frame's bits (property 3)
                if (x_serve && !x_rd && x_reg) begin
                    a_e_wr_reg:  assert (hreg_addr_o == x_a && hreg_wdata_o == e_dat);
                end
                if (x_serve && !x_rd && x_prog) begin
                    a_e_wr_pm:   assert (h_pm_addr_o == s_maddr[8:0] && h_pm_wdata_o == e_dat);
                end
                if (x_serve && !x_rd && x_data) begin
                    a_e_wr_dm:   assert (h_dm_addr_o == s_maddr[8:0] && h_dm_wdata_o == e_dat);
                end
                if (x_serve && x_rd && x_prog) begin
                    a_e_rd_addr_pm: assert (h_pm_addr_o == s_maddr[8:0]);
                end
                if (x_serve && x_rd && x_data) begin
                    a_e_rd_addr_dm: assert (h_dm_addr_o == s_maddr[8:0]);
                end
                // the command's address is on the register bus from edge 8 on
                if (ek >= 6'd8) begin
                    a_e_hreg_addr: assert (hreg_addr_o == x_a);
                end
                // (b) MISO bit-exact in every cycle (properties 4, 5)
                a_e_miso:      assert (miso_o == e_miso);
                // (c) one effect per frame; writes only from >= 40 edges (9)
                if (any_en) begin
                    a_e_one_eff:  assert (!f_eff);
                end
                if (any_we) begin
                    a_e_wr_abort: assert (!csn_i && ek >= 6'd40 && !x_rd);
                end
                // (d) interrupt (property 8 against the frames' IRQMASK)
                a_e_irq:       assert (irq_o == x_irq);
            end
        end

        // ---------------- lemmas (checker-internal invariants) ----------------
        function automatic [2:0] phdist(input [4:0] p, input integer tgt);
            integer cur;
            begin
                cur = p[0] ? 0 : p[1] ? 1 : p[2] ? 2 : p[3] ? 3 : 4;
                phdist = (tgt >= cur) ? tgt - cur : tgt + 5 - cur;
            end
        endfunction
        wire [2:0] x_dist = phdist(ph_i, x_prog ? 4 : 1);
        wire [7:0]  msk8  = (ek >= 6'd8) ? 8'hFF : ((8'h1 << ek[2:0]) - 8'h1);
        wire [5:0]  nwd   = (ek >= 6'd40) ? 6'd32 : (ek - 6'd8);
        wire [31:0] msk32 = (nwd == 6'd32) ? 32'hFFFFFFFF : ((32'h1 << nwd[4:0]) - 32'h1);
        wire [5:0]  rsh   = (ek <= 6'd16) ? 6'd0 : (ek >= 6'd47) ? 6'd31 : ek - 6'd16;

        always @* if (!COVER_ONLY) begin
            if (chk) begin
                l_bc:      assert (m_bc == ((ek > 6'd48) ? 6'd48 : ek));
                l_cs:      assert (ek == 6'd0 || !c_csp);
                if (ek >= 6'd1 && ek <= 6'd7) begin
                    l_cmd_lo:  assert (((m_sr[7:0] ^ e_cmd) & msk8) == 8'h0);
                end
                if (ek >= 6'd8) begin
                    l_cmd:     assert (m_cmd == e_cmd);
                end
                if (ek >= 6'd9 && !x_rd) begin
                    l_wdat:    assert (((m_sr ^ e_dat) & msk32) == 32'h0);
                end
                if (ek >= 6'd8 && x_rd && !q_v && !q_iss) begin
                    l_rdat:    assert (m_sr == (vr << rsh));
                end
                l_hst:     assert (m_hst == {q_iss, q_v});
                l_maddr:   assert (m_maddr == s_maddr);
                l_imask:   assert (m_imask == s_imask);
                l_irq:     assert (m_irq == x_irq);
                if (q_v || q_iss) begin
                    l_q_sclk:  assert (c_sp && !c_csp && sst == {1'b0, q_age} &&
                                       ek == (x_rd ? 6'd8 : 6'd40));
                end
                if (q_v) begin
                    l_q_age:   assert (q_age >= 3'd1 && ({1'b0, q_age} + {1'b0, x_dist}) <= 4'd5);
                end
                if (q_iss) begin
                    l_q_iss:   assert (x_rd && x_md && (x_prog ? ph_i[0] : ph_i[2]) &&
                                       q_age >= 3'd2 && q_age <= 3'd6);
                end
                l_q_excl:  assert (!(q_v && q_iss));
                if (f_eff) begin
                    l_feff:    assert (!q_v && ek >= (x_rd ? 6'd8 : 6'd40));
                end
            end
        end

        // ---------------- covers ----------------
        if (STANDALONE) begin : g_cov_spi
            // fastest legal SCLK: every edge of the frame at the minimum spacing
            logic fast;
            logic last_md_wr;          // last request was an MDATA write
            logic rd_kind_prog, rd_kind_reg, rd_kind_data;
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    fast <= 1'b0; last_md_wr <= 1'b0;
                    rd_kind_prog <= 1'b0; rd_kind_reg <= 1'b0; rd_kind_data <= 1'b0;
                end else begin
                    if (cs_chg && !csn_i) fast <= 1'b1;
                    else if ((sc_chg && !csn_i && ek != 6'd0 && sst != HALF) ||
                             (rise && !csn_i && ek == 6'd0 && cst != CS_GAP))
                        fast <= 1'b0;
                    if (x_serve) begin
                        last_md_wr <= x_md && !x_rd;
                    end else if (e_req_rd || (cs_chg && csn_i && ek < 6'd40)) begin
                        last_md_wr <= 1'b0;   // any other frame in between breaks the run
                    end
                    if (x_serve && x_rd) begin
                        rd_kind_prog <= x_prog;
                        rd_kind_reg  <= x_reg;
                        rd_kind_data <= x_data;
                    end
                end
            end
            logic irq_q;
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) irq_q <= 1'b0;
                else        irq_q <= irq_o;
            end
            if (!COVER_ONLY) begin : g_cov_real
                // Real timing (HALF = 7 ...): assumption witnesses at their
                // boundaries, and a command byte + service at the fastest
                // legal SCLK (all within ~120 cycles; whole frames are
                // >= 570 cycles and are covered by u_cov, below).
                always @* begin
                    if (rst_n && rv1) begin
                        c_env_half:    cover (sc_chg && sst == HALF && !csn_i);
                        c_env_cs_fall: cover (cs_chg && !csn_i && cst == CS_GAP);
                        c_env_cs_rise: cover (cs_chg &&  csn_i && sst == CS_GAP && ek != 6'd0);
                        c_env_cs_su:   cover (rise && !csn_i && cst == CS_GAP);
                        c_env_mosi_su: cover (rise && !csn_i && mst == MOSI_SU);
                        c_env_mosi_hd: cover (mo_chg && sclk_i && c_sp && sst == MOSI_HD + 1 && !csn_i);
                        c_env_sclk_cs_high: cover (rise && csn_i);
                        c_fast_cmd:    cover (e_req_rd && fast);                 // edge 8 at f_clk/14
                        c_fast_serve:  cover (x_serve && x_rd && fast && !c_csp); // its service
                        c_abort_cmd:   cover (cs_chg && csn_i && ek == 6'd3);
                    end
                end
`ifdef HOST_DEEP_WITNESS
                // Supplementary deep witness (not part of the sby tasks): run
                // as bmc with `abc bmc3`; the FAIL trace IS the witness of a
                // whole REG write frame at the fastest legal SCLK (~566 cycles).
                always @* begin
                    if (rst_n && rv1) begin
                        w_fast_write: assert (!(x_serve && !x_rd && x_reg && fast && e_dat != 32'h0));
                    end
                end
`endif
            end else begin : g_cov_frames
                // Cover-only instance (u_c): the same master with the time
                // scale compressed (HALF = 1, CS gaps 2, setup/hold 0), so whole
                // frames fit a cover trace. Assertions are off in this
                // instance; the covers name the design's outputs together
                // with the scoreboard's expectation.
                always @* begin
                    if (rst_n && rv1) begin
                        c_wr_reg:      cover (hreg_we_o && x_serve && !x_rd && x_reg &&
                                              hreg_wdata_o == e_dat && e_dat != 32'h0);
                        c_wr_prog:     cover (h_pm_we_o && x_serve && !x_rd && x_prog &&
                                              h_pm_wdata_o == e_dat && e_dat != 32'h0);
                        c_wr_data:     cover (h_dm_we_o && x_serve && !x_rd && x_data &&
                                              h_dm_wdata_o == e_dat && e_dat != 32'h0);
                        c_rd_reg_done: cover (e_edge && ek == 6'd47 && x_rd && rd_kind_reg  && miso_o);
                        c_rd_prog_done:cover (e_edge && ek == 6'd47 && x_rd && rd_kind_prog && miso_o);
                        c_rd_data_done:cover (e_edge && ek == 6'd47 && x_rd && rd_kind_data && miso_o);
                        c_abort_wr20:  cover (cs_chg && csn_i && ek == 6'd20 && !x_rd);
                        c_abort_rd20:  cover (cs_chg && csn_i && ek == 6'd20 &&  x_rd && miso_o);
                        c_extra_wr:    cover (e_edge && ek == 6'd40 && !x_rd);
                        c_extra_rd:    cover (e_edge && ek == 6'd48 &&  x_rd);
                        c_b2b_md_wr:   cover ((h_pm_we_o || h_dm_we_o) && x_serve && x_md && !x_rd &&
                                              last_md_wr && e_dat != 32'h0);
                        c_irq_raise:   cover (!irq_q && irq_o);
                        c_irq_clear:   cover (irq_q && !irq_o && s_imask != 8'h0);
                    end
                end
            end
        end
    end endgenerate

    // =====================================================================
    // Covers that do not need the legal master (both instances; the free
    // instance reaches them with fast, glitchy SPI and short traces)
    // =====================================================================
    generate if (STANDALONE && !ENV_SPI && !COVER_ONLY) begin : g_cov_free
        logic irq_q;
        logic nr_busy;
        always_ff @(posedge clk) nr_busy <= nr_busy || (rst_n && obs_bitcnt_o >= 6'd3);
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                irq_q <= 1'b0;
            end else begin
                irq_q <= irq_o;
            end
        end
        genvar gi;
        always @* begin
            if (rst_n && rv1) begin
                c_f_wr_reg:    cover (hreg_we_o && hreg_wdata_o != 32'h0);
                c_f_wr_prog:   cover (h_pm_we_o);
                c_f_wr_dm:     cover (h_dm_en_o);                     // a DATA-kind access
                c_f_rd_reg:    cover (q_hst == PEND && obs_hst_o == IDLE && q_cmd7 && !q_memacc);
                c_f_rd_prog:   cover (q_hst == ISSUED && q_maddr[9] == 1'b0);
                c_f_rd_data:   cover (q_hst == ISSUED && q_maddr[9] == 1'b1);
                c_f_wr_maddr:  cover (q_we_maddr && obs_maddr_o[9]);
                c_f_wr_imask:  cover (q_we_imask && obs_imask_o == 8'hFF);
                c_f_abort20:   cover (csn_i && !q_csn && obs_bitcnt_o == 6'd20);
                c_f_wrap:      cover (q_memacc && q_maddr[8:0] == 9'h1FF);
                c_f_cap_shift: cover (q_hst == ISSUED && obs_hst_o == IDLE && !q_csn && q_rise);
                c_f_irq_clear: cover (irq_q && !irq_o);
            end
            // a reset in the middle of a frame (nr_busy: flop without reset)
            if ($initstate) begin
                c_f_nr_init:   assume (!nr_busy);
            end
            c_f_rst_mid:   cover (!rst_n && nr_busy);
        end
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_irq_cov
            always @* begin
                if (rst_n && rv1) begin
                    cover (!irq_q && irq_o && q_attn == (4'h1 << gi) && q_qne == 4'h0);  // c_f_irq_attn
                    cover (!irq_q && irq_o && q_qne == (4'h1 << gi) && q_attn == 4'h0);  // c_f_irq_qne
                end
            end
        end
    end endgenerate
endmodule
