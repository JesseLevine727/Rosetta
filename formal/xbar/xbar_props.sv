`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_xbar (synchronisers and pin crossbar).
//
//  Written from docs/blocks/xbar.md as amended by docs/decisions.md ruling 14
//  ONLY (the RTL was not read). Every property is about the block's ports.
//
//  Ruling 14 replaces xbar.md X1 by its fallback: each of the 16 input pins
//  has TWO synchroniser flops (s1, s2) ahead of the crossbar multiplexer, and
//  each lane input is registered once more after it (lin):
//
//      pin -> s1 -> s2 -> mux -> lin -> lane_in_o
//
//  so a pin value sampled at clock edge t appears on lane_in_o after edge
//  t+2 (lane_in_o(t) = pins(t-3) in cycle numbering). Loopback sources (the
//  combinational output values) enter at the mux, so lane_in_o(t) =
//  outputs(t-1). The host path is pin -> s1 -> s2 (host_*_o(t) = pins(t-2)).
//  Everything else is as in xbar.md.
//
//  Method: an executable model of the spec (m_s1, m_s2, m_lin, m_insel,
//  m_outsel, m_loop, all reset asynchronously to the spec reset values) is
//  driven from the input ports, and EVERY output port is compared with it in
//  EVERY cycle, including reset cycles (a_*_exact). Because every output is
//  pinned to a function of the model state and of inputs other than the
//  pins, the exact checks also prove required property 1's "no
//  combinational path from ui_in_i/uio_in_i to any output", property 8's
//  "with loop = 0 lane inputs do not depend on lane_val_i/lane_oe_i", and
//  property 9 (lane independence): the model's lane_in_o[4L +: 4] reads only
//  insel of lane L and the sources it selects.
//
//  The required properties are also stated directly on the ports, with
//  explicit history registers and without the model pipeline (a_p1_* ...
//  a_p8_*), so a failure names the spec item it breaks.
//
//  Environment: rst_n low in the first cycle, free afterwards (resets in the
//  middle of a trace are checked too). Flops start at arbitrary values. There
//  is no other assumption: hreg_we_i may be high in any cycle (the "only when
//  ph = 1" rule is the host's, it does not matter here), the crossbar may be
//  changed while lanes run (the loader rule is not enforced by hardware), all
//  of run_i, lane_val_i, lane_oe_i, host_miso_i, host_irq_i are free.
//
//  Spec readings that matter (see the delivery report):
//   * obs_insel_o[4i +: 4] = insel of lane input i (= {XIN1, XIN0});
//     obs_outsel_o[5p +: 5] = outsel[p] = {en, lane[1:0], out[1:0]}
//     (= {XOUT2[9:0], XOUT1[29:0], XOUT0[29:0]}); obs_loop_o = loop.
//     Set OBS_CHECK = 0 to leave the obs_* ports unchecked.
//   * With loop = 1, sources 0 and 1 are uo_out_o[0], uo_out_o[1], i.e. the
//     host MISO and IRQ (xbar.md 4.1 read literally).
//   * The reset values of the 13 s2 flops of pins 3-15 are not observable at
//     the ports (insel = 0 in the first cycle after reset). The model uses
//     0x0002 (s2 continues hs2 = 3'b010); only bits 0-2 are observable.
//   * uo_out_o[1:0] pass host_miso_i/host_irq_i through in every cycle,
//     reset included. hreg_rdata_o is combinational on hreg_addr_i and the
//     current registers, in every cycle, not gated by hreg_we_i.
// ===========================================================================

module xbar_props #(
    // 1: stand-alone proof (environment rules are assumed, covers active).
    // 0: monitor inside a larger proof (environment rules are asserted,
    //    covers off).
    parameter STANDALONE = 1,
    // 1: check obs_insel_o / obs_outsel_o / obs_loop_o (layout above).
    parameter OBS_CHECK  = 1
) (
    input wire        clk,
    input wire        rst_n,
    input wire [7:0]  ui_in_i,
    input wire [7:0]  uio_in_i,
    input wire        host_sclk_o,
    input wire        host_csn_o,
    input wire        host_mosi_o,
    input wire        host_miso_i,
    input wire        host_irq_i,
    input wire [15:0] lane_in_o,
    input wire [15:0] lane_val_i,
    input wire [15:0] lane_oe_i,
    input wire [3:0]  run_i,
    input wire [7:0]  uo_out_o,
    input wire [7:0]  uio_out_o,
    input wire [7:0]  uio_oe_o,
    input wire        hreg_we_i,
    input wire [6:0]  hreg_addr_i,
    input wire [31:0] hreg_wdata_i,
    input wire [31:0] hreg_rdata_o,
    input wire [63:0] obs_insel_o,
    input wire [69:0] obs_outsel_o,
    input wire        obs_loop_o
);
    localparam [15:0] S_RST = 16'h0002;      // s1 (and hs2) reset: CS_n high

    // register addresses and used-bit masks
    localparam [6:0] A_XIN0  = 7'h40, A_XIN1  = 7'h41, A_XOUT0 = 7'h42,
                     A_XOUT1 = 7'h43, A_XOUT2 = 7'h44, A_XCTL  = 7'h45;
    function automatic [31:0] mask_of(input [6:0] a);
        case (a)
            7'h40, 7'h41: mask_of = 32'hFFFF_FFFF;
            7'h42, 7'h43: mask_of = 32'h3FFF_FFFF;
            7'h44:        mask_of = 32'h0000_03FF;
            7'h45:        mask_of = 32'h0000_0001;
            default:      mask_of = 32'h0;
        endcase
    endfunction
    function automatic is_reg(input [6:0] a);
        is_reg = (a >= 7'h40) && (a <= 7'h45);
    endfunction

    // pin index q: ui[q] for q < 8, uio[q-8] for q >= 8
    wire [15:0] pins = {uio_in_i, ui_in_i};

    // ---------------------------------------------------------------------
    // Environment
    // ---------------------------------------------------------------------
    generate
        if (STANDALONE) begin : g_env_assume
            always @* begin
                if ($initstate) begin
                    env_rst: assume (!rst_n);
                end
            end
        end else begin : g_env_assert
            always @* begin
                if ($initstate) begin
                    env_rst: assert (!rst_n);
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Executable model of xbar.md section 4 (+ ruling 14)
    // ---------------------------------------------------------------------
    logic [15:0] m_s1, m_s2, m_lin;
    logic [63:0] m_insel;
    logic [69:0] m_outsel;
    logic        m_loop;

    // output pins (4.2), from the model's outsel and the current inputs
    // pin p: en = m_outsel[5p+4], lane = m_outsel[5p+3:5p+2],
    // j = 4*lane + out = m_outsel[5p+3:5p]
    logic [13:0] m_en, m_drive, m_pv;
    integer p;
    always_comb begin
        for (p = 0; p < 14; p = p + 1) begin
            m_en[p]    = m_outsel[5*p+4];
            m_drive[p] = m_en[p] && run_i[m_outsel[5*p+2 +: 2]] && lane_oe_i[m_outsel[5*p +: 4]];
            m_pv[p]    = m_drive[p] && lane_val_i[m_outsel[5*p +: 4]];
        end
    end
    wire [7:0] m_uo     = {m_pv[5:0], host_irq_i, host_miso_i};
    wire [7:0] m_uio_oe = m_drive[13:6];
    wire [7:0] m_uio_o  = m_pv[13:6];

    // crossbar sources (4.1): live = second synchroniser stage; loopback =
    // output values, a released uio reads 1
    wire [15:0] m_lsrc = {~m_uio_oe | m_uio_o, m_uo};
    wire [15:0] m_src  = m_loop ? m_lsrc : m_s2;
    logic [15:0] m_nlin;
    integer i;
    always_comb begin
        for (i = 0; i < 16; i = i + 1)
            m_nlin[i] = m_src[m_insel[4*i +: 4]];
    end

    // register readback (4.3)
    logic [31:0] m_rdata;
    always_comb begin
        case (hreg_addr_i)
            A_XIN0:  m_rdata = m_insel[31:0];
            A_XIN1:  m_rdata = m_insel[63:32];
            A_XOUT0: m_rdata = {2'b00, m_outsel[29:0]};
            A_XOUT1: m_rdata = {2'b00, m_outsel[59:30]};
            A_XOUT2: m_rdata = {22'h0, m_outsel[69:60]};
            A_XCTL:  m_rdata = {31'h0, m_loop};
            default: m_rdata = 32'h0;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_s1     <= S_RST;
            m_s2     <= S_RST;
            m_lin    <= 16'h0;
            m_insel  <= 64'h0;
            m_outsel <= 70'h0;
            m_loop   <= 1'b0;
        end else begin
            m_s1  <= pins;
            m_s2  <= m_s1;
            m_lin <= m_nlin;
            if (hreg_we_i) begin
                case (hreg_addr_i)
                    A_XIN0:  m_insel[31:0]   <= hreg_wdata_i;
                    A_XIN1:  m_insel[63:32]  <= hreg_wdata_i;
                    A_XOUT0: m_outsel[29:0]  <= hreg_wdata_i[29:0];
                    A_XOUT1: m_outsel[59:30] <= hreg_wdata_i[29:0];
                    A_XOUT2: m_outsel[69:60] <= hreg_wdata_i[9:0];
                    A_XCTL:  m_loop          <= hreg_wdata_i[0];
                    default: ;
                endcase
            end
        end
    end

    // ---------------------------------------------------------------------
    // History of the ports (no reset; only read behind an rst_n chain that
    // reaches back to the assumed reset in the first cycle)
    // ---------------------------------------------------------------------
    logic        h_r1, h_r2, h_r3;           // rst_n 1, 2, 3 cycles ago
    logic [15:0] h_p1, h_p2, h_p3;           // pins  1, 2, 3 cycles ago
    logic [63:0] h_insel1;                   // model insel 1 cycle ago
    logic        h_loop1;
    logic [7:0]  h_uo1, h_uio_oe1, h_uio_o1; // output ports 1 cycle ago
    logic        h_we1;
    logic [6:0]  h_addr1;
    logic [31:0] h_wdata1, h_rdata1;
    logic [63:0] h_obs_in1;
    logic [69:0] h_obs_out1;
    logic        h_obs_loop1;
    logic [13:0] h_en1, h_drive1;
    logic [69:0] h_outsel1;
    logic [3:0]  h_run1;
    always @(posedge clk) begin
        h_r1 <= rst_n;  h_r2 <= h_r1;  h_r3 <= h_r2;
        h_p1 <= pins;   h_p2 <= h_p1;  h_p3 <= h_p2;
        h_insel1    <= m_insel;
        h_loop1     <= m_loop;
        h_uo1       <= uo_out_o;
        h_uio_oe1   <= uio_oe_o;
        h_uio_o1    <= uio_out_o;
        h_we1       <= hreg_we_i;
        h_addr1     <= hreg_addr_i;
        h_wdata1    <= hreg_wdata_i;
        h_rdata1    <= hreg_rdata_o;
        h_obs_in1   <= obs_insel_o;
        h_obs_out1  <= obs_outsel_o;
        h_obs_loop1 <= obs_loop_o;
        h_en1       <= m_en;
        h_drive1    <= m_drive;
        h_outsel1   <= m_outsel;
        h_run1      <= run_i;
    end
    wire ok1 = rst_n && h_r1;                  // out of reset this and last cycle
    wire ok2 = ok1 && h_r2;
    wire ok3 = ok2 && h_r3;
    wire rel = rst_n && !h_r1 && !$initstate;  // first cycle after reset
    wire rel2 = rst_n && h_r1 && !h_r2 && !$initstate;   // second cycle after

    // =====================================================================
    // Exact behaviour: every output port equals the model in every cycle
    // =====================================================================
    always @* begin
        a_host_sclk_exact: assert (host_sclk_o == m_s2[0]);
        a_host_csn_exact:  assert (host_csn_o  == m_s2[1]);
        a_host_mosi_exact: assert (host_mosi_o == m_s2[2]);
        a_miso_exact:      assert (uo_out_o[0] == host_miso_i);
        a_irq_exact:       assert (uo_out_o[1] == host_irq_i);
        a_rdata_exact:     assert (hreg_rdata_o == m_rdata);
        if (OBS_CHECK != 0) begin
            a_obs_insel_exact:  assert (obs_insel_o  == m_insel);
            a_obs_outsel_exact: assert (obs_outsel_o == m_outsel);
            a_obs_loop_exact:   assert (obs_loop_o   == m_loop);
        end
    end

    // every lane input, every select value (lane L input j = bit 4L+j)
    genvar gi, gp, gs;
    generate
        for (gi = 0; gi < 16; gi = gi + 1) begin : g_lin
            always @* begin
                assert (lane_in_o[gi] == m_lin[gi]);   // @a_lane_in_exact
            end
        end
    endgenerate

    // every protocol output pin p, every select value
    generate
        for (gp = 0; gp < 6; gp = gp + 1) begin : g_uo
            always @* begin
                assert (uo_out_o[gp+2] == m_pv[gp]);   // @a_uo_exact
            end
        end
        for (gp = 6; gp < 14; gp = gp + 1) begin : g_uio
            always @* begin
                assert (uio_out_o[gp-6] == m_pv[gp]);   // @a_uio_out_exact
                assert (uio_oe_o[gp-6]  == m_drive[gp]);   // @a_uio_oe_exact
            end
        end
    endgenerate

    // =====================================================================
    // Required properties stated directly on the ports
    // =====================================================================

    // ---- 1. Input path latency: host pins after two flops, lane inputs
    //      after three (two synchronisers + lane-input register).
    always @* begin
        if (ok2) begin
            assert (host_sclk_o == h_p2[0]);   // @a_p1_host_sclk
            assert (host_csn_o  == h_p2[1]);   // @a_p1_host_csn
            assert (host_mosi_o == h_p2[2]);   // @a_p1_host_mosi
        end
    end
    generate
        for (gi = 0; gi < 16; gi = gi + 1) begin : g_p1
            always @* begin
                if (ok3 && !h_loop1) begin
                    assert (lane_in_o[gi] == h_p3[h_insel1[4*gi +: 4]]);   // @a_p1_lane_in
                end
            end
        end
    endgenerate

    // ---- 8. Loopback: lane_in_o(t) = SRC(insel)(t-1) from the output PORTS
    //      one cycle earlier (uo_out_o[s] for s < 8; uio_oe ? uio_out : 1).
    wire [15:0] h_lsrc1 = {~h_uio_oe1 | h_uio_o1, h_uo1};
    generate
        for (gi = 0; gi < 16; gi = gi + 1) begin : g_p8
            always @* begin
                if (ok1 && h_loop1) begin
                    assert (lane_in_o[gi] == h_lsrc1[h_insel1[4*gi +: 4]]);   // @a_p8_loop
                end
            end
        end
    endgenerate

    // ---- 2-5. Output pins: equations, halted lanes and unmapped pins
    //      release, uio_oe rule both directions. (Model outsel = the
    //      register contents, which are checked against hreg_rdata_o.)
    generate
        for (gp = 0; gp < 14; gp = gp + 1) begin : g_out
            wire en = m_en[gp];
            wire [1:0] ln = m_outsel[5*gp+2 +: 2];
            wire [3:0] j  = m_outsel[5*gp +: 4];
            wire released = (gp < 6) ? !uo_out_o[(gp < 6) ? gp+2 : 2]
                                     : (!uio_oe_o[(gp >= 6) ? gp-6 : 0] &&
                                        !uio_out_o[(gp >= 6) ? gp-6 : 0]);
            wire value    = (gp < 6) ? uo_out_o[(gp < 6) ? gp+2 : 2]
                                     : uio_out_o[(gp >= 6) ? gp-6 : 0];
            always @* begin
                // 4: unmapped pins drive nothing
                if (!en) begin
                    assert (released);   // @a_p4_unmapped
                end
                // 3: halted lanes drive nothing
                if (en && !run_i[ln]) begin
                    assert (released);   // @a_p3_halted
                end
                // a mapped, running lane with its output disabled: released
                if (!lane_oe_i[j]) begin
                    assert (released);   // @a_p2_oe_off
                end
                // 2: driven value
                if (en && run_i[ln] && lane_oe_i[j]) begin
                    assert (value == lane_val_i[j]);   // @a_p2_value
                end
            end
            if (gp >= 6) begin : g_oe
                always @* begin
                    // 5: uio_oe rule, both directions
                    assert (uio_oe_o[gp-6] == (en && run_i[ln] && lane_oe_i[j]));   // @a_p5_uio_oe
                    // a uio pin never outputs 1 while not driven
                    assert (!uio_out_o[gp-6] || uio_oe_o[gp-6]);   // @a_p2_uio_out_only_when_oe
                end
            end
        end
    endgenerate

    // ---- 6. Configuration: changes only at edges with hreg_we_i and a
    //      matching address; reads return what was written, unused bits 0.
    always @* begin
        assert ((hreg_rdata_o & ~mask_of(hreg_addr_i)) == 32'h0);   // @a_p6_unused_zero
        if (!is_reg(hreg_addr_i)) begin
            assert (hreg_rdata_o == 32'h0);   // @a_p6_unowned_zero
        end
        // read back what was written in the previous cycle
        if (ok1 && h_we1 && is_reg(h_addr1) && hreg_addr_i == h_addr1) begin
            assert (hreg_rdata_o == (h_wdata1 & mask_of(h_addr1)));   // @a_p6_readback
        end
        // no write to the register read now: same value as one cycle ago
        if (ok1 && hreg_addr_i == h_addr1 && !(h_we1 && h_addr1 == hreg_addr_i)) begin
            assert (hreg_rdata_o == h_rdata1);   // @a_p6_read_stable
        end
        if (OBS_CHECK != 0 && ok1) begin
            if (!(h_we1 && h_addr1 == A_XIN0)) begin
                assert (obs_insel_o[31:0] == h_obs_in1[31:0]);   // @a_p6_xin0_hold
            end
            if (!(h_we1 && h_addr1 == A_XIN1)) begin
                assert (obs_insel_o[63:32] == h_obs_in1[63:32]);   // @a_p6_xin1_hold
            end
            if (!(h_we1 && h_addr1 == A_XOUT0)) begin
                assert (obs_outsel_o[29:0] == h_obs_out1[29:0]);   // @a_p6_xout0_hold
            end
            if (!(h_we1 && h_addr1 == A_XOUT1)) begin
                assert (obs_outsel_o[59:30] == h_obs_out1[59:30]);   // @a_p6_xout1_hold
            end
            if (!(h_we1 && h_addr1 == A_XOUT2)) begin
                assert (obs_outsel_o[69:60] == h_obs_out1[69:60]);   // @a_p6_xout2_hold
            end
            if (!(h_we1 && h_addr1 == A_XCTL)) begin
                assert (obs_loop_o == h_obs_loop1);   // @a_p6_xctl_hold
            end
        end
    end

    // ---- 7. Reset: in reset and in the first cycle after it, every protocol
    //      pin is released, all enables 0, CS_n high, lane inputs 0, loop 0,
    //      all registers read 0. In the second cycle after reset the host
    //      still sees the s1 reset value and the lane inputs still read 0
    //      (insel = 0, s2[0] reset value 0).
    always @* begin
        if (!rst_n || rel) begin
            assert (uo_out_o[7:2] == 6'h0);   // @a_p7_uo_rel
            assert (uio_oe_o == 8'h0);   // @a_p7_uio_oe0
            assert (uio_out_o == 8'h0);   // @a_p7_uio_out0
            assert ({host_mosi_o, host_csn_o, host_sclk_o} == 3'b010);   // @a_p7_host
            assert (lane_in_o == 16'h0);   // @a_p7_lane_in0
            assert (hreg_rdata_o == 32'h0);   // @a_p7_rdata0
            if (OBS_CHECK != 0) begin
                assert (obs_insel_o == 64'h0 && obs_outsel_o == 70'h0 && !obs_loop_o);   // @a_p7_obs0
            end
        end
        if (rel2) begin
            assert ({host_mosi_o, host_csn_o, host_sclk_o} == 3'b010);   // @a_p7_host2
            assert (lane_in_o == 16'h0);   // @a_p7_lane_in02
        end
    end

    // =====================================================================
    // Covers (stand-alone only)
    // =====================================================================
    generate
        if (STANDALONE) begin : g_cover
            // environment: reset released; reset again in mid-operation
            always @* begin
                cover (rel);   // @c_rst_release
                cover (rel2 && hreg_we_i);   // @c_rst_release2
                cover (!rst_n && h_r1 && h_r2 && (h_insel1 != 64'h0) &&   // @c_rst_mid
                                      h_drive1 != 14'h0 && !$initstate);
                // a write in reset is ignored
                cover (!rst_n && hreg_we_i && hreg_addr_i == A_XOUT1 &&   // @c_we_in_reset
                                      hreg_wdata_i == 32'hFFFF_FFFF);
                // host pins see a 1 from their pin, exactly two cycles later
                cover (ok2 && host_sclk_o && !h_p1[0] && h_p2[0]);   // @c_host_sclk
                cover (ok2 && !host_csn_o && h_p1[1] && !h_p2[1]);   // @c_host_csn0
                cover (ok2 && host_mosi_o && !h_p1[2] && h_p2[2]);   // @c_host_mosi
                // fixed host outputs
                cover (uo_out_o[0] && !uo_out_o[1]);   // @c_miso
                cover (!uo_out_o[0] && uo_out_o[1]);   // @c_irq
                // reset select: every lane input listens to ui[0]
                cover (ok3 && m_insel == 64'h0 && !h_loop1 && lane_in_o == 16'hFFFF);   // @c_rst_sel_ui0
                // two lanes listening to the host bus (ui[0..2])
                cover (ok3 && !h_loop1 && h_insel1[3:0] == 4'd0 &&   // @c_listen_host
                                      h_insel1[23:20] == 4'd2 && lane_in_o[0] && lane_in_o[5]);
                // loopback combined with lanes from different selects
                cover (ok1 && h_loop1 && lane_in_o[3] && !lane_in_o[12] &&   // @c_loop_mix
                                      h_insel1[15:12] == 4'd9 && h_insel1[51:48] == 4'd4);
            end

            // register write and readback of every register, all used bits 1
            // after a write of all ones (unused bits ignored), and a write to
            // an unowned address leaving every register unchanged
            for (gs = 0; gs < 6; gs = gs + 1) begin : g_c_reg
                always @* begin
                    cover (ok1 && h_we1 && h_addr1 == 7'h40 + gs &&   // @c_reg_rb
                                     h_wdata1 == 32'hFFFF_FFFF &&
                                     hreg_addr_i == 7'h40 + gs &&
                                     hreg_rdata_o == mask_of(7'h40 + gs));
                    cover (ok1 && !(h_we1 && h_addr1 == 7'h40 + gs) &&   // @c_reg_hold
                                       hreg_addr_i == 7'h40 + gs && hreg_rdata_o != 32'h0 &&
                                       hreg_we_i);
                end
            end
            always @* begin
                cover (ok1 && h_we1 && h_addr1 == 7'h46 && h_wdata1 == 32'hFFFF_FFFF &&   // @c_unowned_we
                                      m_insel != 64'h0);
                cover (ok1 && h_we1 && h_addr1 == 7'h00 && h_wdata1 == 32'hFFFF_FFFF &&   // @c_alias_we
                                      hreg_addr_i == A_XIN0 && hreg_rdata_o == 32'h1234_5678);
            end

            // every input select value s on every lane input i, live path:
            // the pin value of three cycles ago (a 1, after a 0) arrives
            for (gi = 0; gi < 16; gi = gi + 1) begin : g_c_in
                for (gs = 0; gs < 16; gs = gs + 1) begin : g_s
                    always @* begin
                        cover (ok3 && !h_loop1 && h_insel1[4*gi +: 4] == gs &&   // @c_in_sel
                                         h_p3[gs] && !h_p2[gs] && lane_in_o[gi]);
                    end
                end
            end

            // loopback of every source s (on lane input s): through uo
            // (s < 8, host MISO/IRQ for s = 0, 1), through a driven uio
            // (value 0 and 1) and through a released uio (reads 1)
            for (gs = 0; gs < 16; gs = gs + 1) begin : g_c_loop
                if (gs < 8) begin : g_uo
                    always @* begin
                        cover (ok1 && h_loop1 && h_insel1[4*gs +: 4] == gs &&   // @c_loop_uo1
                                           h_uo1[gs] && lane_in_o[gs]);
                    end
                end else begin : g_uio
                    always @* begin
                        cover (ok1 && h_loop1 && h_insel1[4*gs +: 4] == gs &&   // @c_loop_rel
                                           !h_uio_oe1[gs-8] && lane_in_o[gs]);
                        cover (ok1 && h_loop1 && h_insel1[4*gs +: 4] == gs &&   // @c_loop_drv0
                                            h_uio_oe1[gs-8] && !h_uio_o1[gs-8] && !lane_in_o[gs]);
                        cover (ok1 && h_loop1 && h_insel1[4*gs +: 4] == gs &&   // @c_loop_drv1
                                            h_uio_oe1[gs-8] && h_uio_o1[gs-8] && lane_in_o[gs]);
                    end
                end
            end

            // every output select value (lane, out) on every pin p with a
            // running lane: the pin drives a 1; uio pins also drive a 0; a
            // lane halting while mapped releases the pin
            for (gp = 0; gp < 14; gp = gp + 1) begin : g_c_out
                for (gs = 0; gs < 16; gs = gs + 1) begin : g_s
                    always @* begin
                        cover (rst_n && m_outsel[5*gp +: 5] == (5'h10 | gs) &&   // @c_out_sel
                                          run_i[gs / 4] && lane_oe_i[gs] && lane_val_i[gs] &&
                                          ((gp < 6) ? uo_out_o[(gp < 6) ? gp+2 : 2]
                                                    : uio_out_o[(gp >= 6) ? gp-6 : 0]));
                    end
                end
                if (gp >= 6) begin : g_drv0
                    always @* begin
                        cover (rst_n && uio_oe_o[gp-6] && !uio_out_o[gp-6]);   // @c_uio_drive0
                    end
                end
                always @* begin
                    cover (ok1 && h_drive1[gp] && m_outsel[5*gp +: 5] == h_outsel1[5*gp +: 5] &&   // @c_halt_mapped
                                          h_run1[m_outsel[5*gp+2 +: 2]] && !run_i[m_outsel[5*gp+2 +: 2]] &&
                                          lane_oe_i[m_outsel[5*gp +: 4]] && lane_val_i[m_outsel[5*gp +: 4]]);
                end
            end
        end
    endgenerate

endmodule
