`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_stuff (bit stuffing), spec v2.
//
//  Written from docs/blocks/stuff.md ONLY (the RTL was not read). Every
//  property is about the block's ports.
//
//  Two checker modules:
//
//    stuff_props     one instance, free inputs. An executable model of the
//                    "Behaviour" section (run, last, viol) is driven from the
//                    input ports, and every output is compared with it every
//                    cycle (required properties 3, 4, 5 and the exact cycle
//                    behaviour). Properties 3, 4 and 5 are also stated
//                    directly on the ports, without the model.
//
//    stuff_rt_props  a transmitter instance A and a receiver instance B joined
//                    by a lossless, order-preserving wire with independent
//                    timing on each side (built in stuff_fv.sv). Required
//                    properties 1 (max run on the wire) and 2 (round trip,
//                    progress, no violation on a legal stream). Round trip is
//                    proved with a symbolic watched data bit, so the proof is
//                    unbounded.
//
//  Environment: rst_n low in the first cycle and free afterwards (resets in
//  the middle of a trace are checked too); cfg_* change only in a cycle with
//  rst_n low or clear_i high (spec v2, "Behaviour").
//
//  Spec reading that matters:
//   * stuff_bit_o is checked only while stuff_next_o = 1.
//   * Outputs are combinational functions of (run, last, viol) and the
//     CURRENT cfg_* inputs (spec v2). So in a clear cycle in which cfg
//     changes, stuff_next_o / violation_o follow the new cfg and the old
//     state. Set CFG_CYCLE_CHECK = 0 to skip the output checks in exactly
//     those cycles (see the report: an implementation with a registered
//     stuff_next_o differs only there).
//   * The reset value of last (0) is not observable at the ports and is not
//     checked.
// ===========================================================================

module stuff_props #(
    // 1: stand-alone proof (environment rules are assumed, covers active).
    // 0: monitor inside a larger proof (environment rules are asserted,
    //    covers off).
    parameter STANDALONE      = 1,
    // 1: check outputs also in a cycle where cfg_* changed (spec v2 text).
    // 0: do not check outputs in such cycles.
    parameter CFG_CYCLE_CHECK = 1
) (
    input wire       clk,
    input wire       rst_n,
    input wire       cfg_en_i,
    input wire [2:0] cfg_n_i,
    input wire       cfg_mode_i,
    input wire       clear_i,
    input wire       bit_valid_i,
    input wire       bit_i,
    input wire       stuff_next_o,
    input wire       stuff_bit_o,
    input wire       violation_o
);
    localparam ONES  = 1'b0;
    localparam EQUAL = 1'b1;

    // cfg of the previous cycle; f_chg = cfg differs from the previous cycle
    logic [4:0] p_cfg;
    always @(posedge clk) p_cfg <= {cfg_en_i, cfg_n_i, cfg_mode_i};
    wire f_chg = !$initstate && ({cfg_en_i, cfg_n_i, cfg_mode_i} != p_cfg);
    wire chk   = (CFG_CYCLE_CHECK != 0) || !f_chg || !rst_n;

    // ---------------------------------------------------------------------
    // Environment
    // ---------------------------------------------------------------------
    wire env_cfg_ok = !f_chg || !rst_n || clear_i;
    generate
        if (STANDALONE) begin : g_env_assume
            always @* begin
                if ($initstate) begin
                    env_rst: assume (!rst_n);
                end
                env_cfg: assume (env_cfg_ok);
            end
        end else begin : g_env_assert
            always @* begin
                if ($initstate) begin
                    env_rst: assert (!rst_n);
                end
                env_cfg: assert (env_cfg_ok);
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Executable model of "Behaviour" (state: run 0-7, last, viol)
    // ---------------------------------------------------------------------
    logic [2:0] m_run;
    logic       m_last;
    logic       m_viol;

    wire       m_active = cfg_en_i && (cfg_n_i != 3'd0) &&
                          !(cfg_mode_i == EQUAL && cfg_n_i == 3'd1);
    wire       m_sn     = m_active && (m_run == cfg_n_i);            // stuff_next_o
    wire       m_sb     = (cfg_mode_i == EQUAL) ? ~m_last : 1'b0;  // stuff_bit_o
    wire       m_vo     = m_viol && m_active;                      // violation_o
    wire       m_bad    = m_sn && (bit_i != m_sb);
    wire [2:0] m_inc    = (m_run == 3'd7) ? 3'd7 : m_run + 3'd1;
    wire       m_take   = bit_valid_i && !clear_i;                 // clear wins

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_run  <= 3'd0;
            m_last <= 1'b0;
            m_viol <= 1'b0;
        end else if (clear_i) begin
            m_run  <= 3'd0;
            m_viol <= 1'b0;
        end else if (bit_valid_i) begin
            m_viol <= m_bad;
            if (m_bad) begin
                m_run <= 3'd1;
                if (cfg_mode_i == EQUAL) m_last <= bit_i;
            end else if (cfg_mode_i == ONES) begin
                m_run <= bit_i ? m_inc : 3'd0;
            end else begin
                m_run  <= (m_run != 3'd0 && bit_i == m_last) ? m_inc : 3'd1;
                m_last <= bit_i;
            end
        end else begin
            m_viol <= 1'b0;
        end
    end

    // ---------------------------------------------------------------------
    // Exact behaviour: outputs equal the model in every cycle
    // ---------------------------------------------------------------------
    always @* begin
        if (chk) begin
            a_sn_exact:   assert (stuff_next_o == m_sn);
            a_viol_exact: assert (violation_o  == m_vo);
            if (m_sn) begin
                a_sb_exact: assert (stuff_bit_o == m_sb);
            end
        end
    end

    // ---------------------------------------------------------------------
    // Port-level statements (no model)
    // ---------------------------------------------------------------------
    wire p_active   = cfg_en_i && (cfg_n_i != 3'd0) &&
                      !(cfg_mode_i == EQUAL && cfg_n_i == 3'd1);
    wire p_bad      = rst_n && bit_valid_i && !clear_i && stuff_next_o &&
                      (bit_i != stuff_bit_o);
    logic p_bad_q;   // p_bad one cycle earlier
    always @(posedge clk) p_bad_q <= p_bad;

    // Property 4: reset values (asynchronous), disabled behaviour incl.
    // EQUAL with N = 1
    always @* begin
        if (!rst_n) begin
            a_rst_sn:   assert (!stuff_next_o);
            a_rst_viol: assert (!violation_o);
        end
        if (chk && !p_active) begin
            a_dis_sn:   assert (!stuff_next_o);
            a_dis_viol: assert (!violation_o);
        end
        if (chk && stuff_next_o && cfg_mode_i == ONES) begin
            a_ones_sb0: assert (stuff_bit_o == 1'b0);
        end
    end

    always @(posedge clk) begin
        if (!$initstate && rst_n) begin
            // Property 4: first cycle after reset release
            if (!$past(rst_n)) begin
                a_rel_sn:   assert (!stuff_next_o);
                a_rel_viol: assert (!violation_o);
            end
            // Property 4: clear_i wins over bit_valid_i and forgets history
            if ($past(rst_n) && $past(clear_i)) begin
                a_clr_sn:   assert (!stuff_next_o);
                a_clr_viol: assert (!violation_o);
            end
            // Property 3: violation_o is 1 exactly in the cycle after a wire
            // bit arrived at a stuff position with the wrong value
            if (chk) begin
                a_viol_port: assert (violation_o == ($past(p_bad) && p_active));
            end
            if (!f_chg) begin
                // no bit, no clear: state holds, no violation
                if ($past(rst_n) && !$past(clear_i) && !$past(bit_valid_i)) begin
                    a_idle_sn:   assert (stuff_next_o == $past(stuff_next_o));
                    a_idle_viol: assert (!violation_o);
                    if (stuff_next_o) begin
                        a_idle_sb: assert (stuff_bit_o == $past(stuff_bit_o));
                    end
                end
                // Property 2 (progress), block level: a correct stuff bit is
                // never followed by another stuff request
                if ($past(rst_n && m_take && stuff_next_o && bit_i == stuff_bit_o)) begin
                    a_stuff_then_data: assert (!stuff_next_o);
                end
                // Property 5: after a violating bit the run is 1, so the next
                // stuff request comes at once only if N = 1 (ONES)
                if ($past(p_bad)) begin
                    a_viol_restart: assert (stuff_next_o ==
                        (p_active && cfg_mode_i == ONES && cfg_n_i == 3'd1));
                end
                // Property 5: violation_o two cycles in a row only for two
                // consecutive violating bits, which needs ONES with N = 1
                if (violation_o && $past(violation_o)) begin
                    a_viol_single: assert ($past(p_bad) && $past(p_bad_q));
                    a_viol_b2b_n1: assert (cfg_mode_i == ONES && cfg_n_i == 3'd1);
                end
            end
        end
    end

    // ---------------------------------------------------------------------
    // Covers (stand-alone only)
    // ---------------------------------------------------------------------
    logic f_clr_mid;   // a clear happened while a run was partly built
    logic f_viol;      // a violation in this frame
    logic f_cfgchg;    // cfg changed in a clear cycle at some point
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_clr_mid <= 1'b0;
            f_viol    <= 1'b0;
            f_cfgchg  <= 1'b0;
        end else begin
            if (clear_i && m_active && m_run != 3'd0 && !m_sn) f_clr_mid <= 1'b1;
            if (clear_i)          f_viol <= 1'b0;
            else if (violation_o) f_viol <= 1'b1;
            if (clear_i && f_chg) f_cfgchg <= 1'b1;
        end
    end

    generate
        if (STANDALONE) begin : g_cover
            always @(posedge clk) begin
                if (!$initstate) begin
                    c_rst_release:  cover (rst_n && !$past(rst_n));
                    // a correct stuff bit consumed, each mode
                    c_stuff_ok_ones: cover (rst_n && $past(rst_n) && cfg_mode_i == ONES &&
                                            $past(m_take && stuff_next_o && bit_i == stuff_bit_o));
                    c_stuff_ok_eq0:  cover (rst_n && $past(rst_n) && cfg_mode_i == EQUAL &&
                                            $past(m_take && stuff_next_o && bit_i == stuff_bit_o && !bit_i));
                    c_stuff_ok_eq1:  cover (rst_n && $past(rst_n) && cfg_mode_i == EQUAL &&
                                            $past(m_take && stuff_next_o && bit_i == stuff_bit_o && bit_i));
                    // clear while a stuff bit is pending; clear together with a bit
                    c_clr_pending: cover (rst_n && clear_i && stuff_next_o);
                    c_clr_and_bit: cover (rst_n && clear_i && bit_valid_i && m_active &&
                                          m_run != 3'd0 && !m_sn);
                    // idle cycles while a stuff bit is pending
                    c_idle_pending: cover (rst_n && stuff_next_o && $past(stuff_next_o) &&
                                           $past(!bit_valid_i && !clear_i && rst_n));
                    // cfg changes: in reset, in a clear cycle, and the clear-cycle
                    // outputs following the new cfg
                    c_cfg_chg_rst:   cover (!rst_n && f_chg);
                    c_cfg_chg_clr:   cover (rst_n && clear_i && f_chg);
                    c_cfg_chg_sn:    cover (rst_n && clear_i && f_chg && stuff_next_o);
                end
                // violation detected, each mode; back to back (ONES, N = 1)
                c_viol_ones:   cover (violation_o && cfg_mode_i == ONES);
                c_viol_eq:     cover (violation_o && cfg_mode_i == EQUAL);
                c_viol_b2b:    cover (violation_o && p_bad);   // next cycle high too
                // after a violation the run restarts: a full N = 7 run after it
                c_viol_restart_ones: cover (f_viol && stuff_next_o && cfg_mode_i == ONES  && cfg_n_i == 3'd7);
                c_viol_restart_eq:   cover (f_viol && stuff_next_o && cfg_mode_i == EQUAL && cfg_n_i == 3'd7);
                // clear mid-run, then a full new run up to a stuff request
                c_clr_mid_then_stuff: cover (f_clr_mid && stuff_next_o && cfg_n_i == 3'd3);
                // new cfg after a clear, then a stuff request in the new frame
                c_cfg_chg_then_stuff: cover (f_cfgchg && stuff_next_o);
                // N = 1 and N = 7 (EQUAL: N = 2, the smallest active N)
                c_ones_n1: cover (stuff_next_o && cfg_mode_i == ONES  && cfg_n_i == 3'd1);
                c_ones_n7: cover (stuff_next_o && cfg_mode_i == ONES  && cfg_n_i == 3'd7);
                c_eq_n2:   cover (stuff_next_o && cfg_mode_i == EQUAL && cfg_n_i == 3'd2);
                c_eq_n7:   cover (stuff_next_o && cfg_mode_i == EQUAL && cfg_n_i == 3'd7);
                c_usb:     cover (stuff_next_o && cfg_mode_i == ONES  && cfg_n_i == 3'd6);
                c_hdlc:    cover (stuff_next_o && cfg_mode_i == ONES  && cfg_n_i == 3'd5);
                c_can:     cover (stuff_next_o && cfg_mode_i == EQUAL && cfg_n_i == 3'd5);
                // disabled operation: long runs, no stuff request
                c_dis_en0_ones: cover (rst_n && !cfg_en_i && cfg_n_i == 3'd3 &&
                                       cfg_mode_i == ONES  && m_run == 3'd7);
                c_dis_en0_eq:   cover (rst_n && !cfg_en_i && cfg_n_i == 3'd3 &&
                                       cfg_mode_i == EQUAL && m_run == 3'd7);
                c_dis_n0:       cover (rst_n && cfg_en_i && cfg_n_i == 3'd0 && m_run == 3'd7);
                c_dis_eq_n1:    cover (rst_n && cfg_en_i && cfg_n_i == 3'd1 &&
                                       cfg_mode_i == EQUAL && m_run == 3'd7);
            end
        end
    endgenerate

endmodule


// ===========================================================================
//  Round trip, progress and wire run length (required properties 1 and 2).
//
//  Observes transmitter instance A (a_*) and receiver instance B (b_*), each
//  with its own cfg (a frame's cfg reaches B when B clears into that frame).
//  A's bit_valid_i/bit_i are the wire as transmitted; B's are the wire as
//  received. The channel must be lossless and order-preserving with at most
//  DEPTH bits in flight; frames are delimited by clear: A clears only when
//  the wire is empty, and B clears once, after A, before it takes any bit of
//  the new frame. These harness rules are checked (env_*).
//
//  Data bit sent     = A takes a bit while A.stuff_next_o = 0.
//  Data bit received = B takes a bit while B.stuff_next_o = 0.
//  Round trip: the k-th data bit received equals the k-th data bit sent, no
//  data bit is received that was not sent, none is lost. Checked by
//  watching one symbolically chosen data bit (w_*) and counting the data
//  bits ahead of it.
// ===========================================================================
module stuff_rt_props #(
    parameter DEPTH = 2
) (
    input wire       clk,
    input wire       rst_n,
    // instance A: transmitter side
    input wire       a_cfg_en_i,
    input wire [2:0] a_cfg_n_i,
    input wire       a_cfg_mode_i,
    input wire       a_clear_i,
    input wire       a_bit_valid_i,
    input wire       a_bit_i,
    input wire       a_stuff_next_o,
    input wire       a_stuff_bit_o,
    input wire       a_violation_o,
    // instance B: receiver side
    input wire       b_cfg_en_i,
    input wire [2:0] b_cfg_n_i,
    input wire       b_cfg_mode_i,
    input wire       b_clear_i,
    input wire       b_bit_valid_i,
    input wire       b_bit_i,
    input wire       b_stuff_next_o,
    input wire       b_stuff_bit_o,
    input wire       b_violation_o
);
    localparam CW = $clog2(DEPTH + 1) + 1;   // one spare bit to see overflow
    localparam EQUAL = 1'b1;

    wire a_push   = a_bit_valid_i && !a_clear_i;
    wire b_pop    = b_bit_valid_i && !b_clear_i;
    wire tx_data  = a_push && !a_stuff_next_o;
    wire rx_data  = b_pop  && !b_stuff_next_o;
    wire a_active = a_cfg_en_i && a_cfg_n_i != 3'd0 &&
                    !(a_cfg_mode_i == EQUAL && a_cfg_n_i == 3'd1);
    wire [4:0] a_cfg = {a_cfg_en_i, a_cfg_n_i, a_cfg_mode_i};
    wire [4:0] b_cfg = {b_cfg_en_i, b_cfg_n_i, b_cfg_mode_i};

    // ---------------------------------------------------------------------
    // Mirror of the wire (bit value and "is a stuff bit" tag, head at [0])
    // ---------------------------------------------------------------------
    logic [CW-1:0]    cnt;
    logic [DEPTH-1:0] q_bit, q_tag;
    logic [DEPTH-1:0] n_bit, n_tag;
    logic             pend;        // A has cleared, B not yet
    wire  [CW-1:0]    wr_idx = cnt - {{(CW-1){1'b0}}, b_pop};
    integer i;

    always_comb begin
        n_bit = q_bit;
        n_tag = q_tag;
        if (b_pop) begin
            n_bit = q_bit >> 1;
            n_tag = q_tag >> 1;
        end
        for (i = 0; i < DEPTH; i = i + 1) begin
            if (a_push && wr_idx == i) begin
                n_bit[i] = a_bit_i;
                n_tag[i] = a_stuff_next_o;
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt   <= '0;
            q_bit <= '0;
            q_tag <= '0;
            pend  <= 1'b0;
        end else begin
            cnt   <= cnt + {{(CW-1){1'b0}}, a_push} - {{(CW-1){1'b0}}, b_pop};
            q_bit <= n_bit;
            q_tag <= n_tag;
            if (a_clear_i)      pend <= 1'b1;
            else if (b_clear_i) pend <= 1'b0;
        end
    end

    // channel / harness obligations
    logic [4:0] p_a_cfg, p_b_cfg;
    always @(posedge clk) begin
        p_a_cfg <= a_cfg;
        p_b_cfg <= b_cfg;
    end
    always @* begin
        if (rst_n) begin
            env_rt_tx_obeys: assert (!(a_push && a_stuff_next_o) || a_bit_i == a_stuff_bit_o);
            env_rt_no_under: assert (!b_pop || cnt != '0);
            env_rt_no_over:  assert (!(a_push && !b_pop && cnt == DEPTH));
            env_rt_chan_bit: assert (!b_pop || b_bit_i == q_bit[0]);
            env_rt_a_clr:    assert (!a_clear_i || (cnt == '0 && !pend && !b_bit_valid_i));
            env_rt_b_clr:    assert (!b_clear_i || pend);
            env_rt_b_wait:   assert (!b_bit_valid_i || !pend);
            // each side's cfg changes only in reset or with its own clear;
            // B uses the cfg of the frame it is in
            if (!$initstate) begin
                env_rt_cfg_a:  assert (a_clear_i || a_cfg == p_a_cfg);
                env_rt_cfg_b:  assert (b_clear_i || b_cfg == p_b_cfg);
            end
            env_rt_cfg_ab: assert (pend || a_clear_i || a_cfg == b_cfg);
        end
    end

    // ---------------------------------------------------------------------
    // Required property 2: round trip
    // ---------------------------------------------------------------------
    logic [CW-1:0] in_flight;      // data bits sent, not yet received
    logic          w_armed;        // watched data bit sent, not yet received
    logic          w_val;
    logic [CW-1:0] w_ahead;        // data bits that will be received before it
    (* anyseq *) wire f_watch_go;  // free choice of which data bit to watch

    wire w_now = tx_data && f_watch_go && !w_armed;
    wire w_hit = w_armed && rx_data && (w_ahead == '0);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_flight <= '0;
            w_armed   <= 1'b0;
            w_val     <= 1'b0;
            w_ahead   <= '0;
        end else begin
            in_flight <= in_flight + {{(CW-1){1'b0}}, tx_data}
                                   - {{(CW-1){1'b0}}, rx_data};
            if (w_armed && rx_data) begin
                if (w_ahead == '0) w_armed <= 1'b0;
                else               w_ahead <= w_ahead - 1'b1;
            end
            if (w_now) begin
                w_armed <= 1'b1;
                w_val   <= a_bit_i;
                w_ahead <= in_flight - {{(CW-1){1'b0}}, rx_data};
            end
        end
    end

    always @* begin
        if (rst_n) begin
            // the receiver's data bit at the watched position is the watched bit
            if (w_hit) begin
                rt_watch_value: assert (b_bit_i == w_val);
            end
            // no data bit appears that was never sent
            rt_no_phantom: assert (!(rx_data && in_flight == '0));
            // no data bit is lost: undelivered data fits in the wire
            rt_no_loss:    assert (in_flight <= cnt);
            // the watched bit is still ahead in the wire until it is received
            if (w_armed) begin
                rt_watch_live: assert (w_ahead < in_flight);
            end
            // receiver drops exactly the bits the transmitter inserted
            if (b_pop) begin
                rt_classify: assert (b_stuff_next_o == q_tag[0]);
            end
            // violation_o never fires on a legal stream
            rt_no_viol_a: assert (!a_violation_o);
            rt_no_viol_b: assert (!b_violation_o);
        end
    end

    // ---------------------------------------------------------------------
    // Required property 1: run length on the wire (as transmitted), and
    // required property 2: progress (a stuff bit is followed by a data bit)
    // ---------------------------------------------------------------------
    logic [3:0] wr;           // current run on the wire, saturating at 15
    logic       wr_last, wr_hist;
    logic       f_prev_stuff; // the previous wire bit of this frame was a stuff bit
    wire  [3:0] wr_inc = (wr == 4'd15) ? 4'd15 : wr + 4'd1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr <= 4'd0; wr_last <= 1'b0; wr_hist <= 1'b0; f_prev_stuff <= 1'b0;
        end else if (a_clear_i) begin
            wr <= 4'd0; wr_hist <= 1'b0; f_prev_stuff <= 1'b0;
        end else if (a_push) begin
            if (a_cfg_mode_i == EQUAL)
                wr <= (wr_hist && a_bit_i == wr_last) ? wr_inc : 4'd1;
            else
                wr <= a_bit_i ? wr_inc : 4'd0;
            wr_last      <= a_bit_i;
            wr_hist      <= 1'b1;
            f_prev_stuff <= a_stuff_next_o;
        end
    end

    always @* begin
        if (rst_n && !a_clear_i && a_active) begin
            rt_max_run: assert (wr <= {1'b0, a_cfg_n_i});
        end
        if (rst_n && a_push && f_prev_stuff) begin
            rt_progress: assert (!a_stuff_next_o);
        end
    end

    // ---------------------------------------------------------------------
    // Covers
    // ---------------------------------------------------------------------
    logic f_drop;       // B has dropped a stuff bit since reset
    logic f_drop_fr;    // ... since its last clear
    logic f_frame2;     // B has started a second frame
    logic f_newcfg;     // B has cleared into a frame with a different cfg
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_drop <= 1'b0; f_drop_fr <= 1'b0; f_frame2 <= 1'b0; f_newcfg <= 1'b0;
        end else begin
            if (b_pop && b_stuff_next_o) f_drop <= 1'b1;
            if (b_clear_i)                     f_drop_fr <= 1'b0;
            else if (b_pop && b_stuff_next_o)  f_drop_fr <= 1'b1;
            if (b_clear_i)                     f_frame2 <= 1'b1;
            if (b_clear_i && b_cfg != p_b_cfg) f_newcfg <= 1'b1;
        end
    end

    always @(posedge clk) begin
        // stuff bit inserted by A and dropped by B, each mode
        c_rt_ins_ones:  cover (rst_n && a_push && a_stuff_next_o && a_cfg_mode_i != EQUAL);
        c_rt_ins_eq:    cover (rst_n && a_push && a_stuff_next_o && a_cfg_mode_i == EQUAL);
        c_rt_drop_ones: cover (rst_n && b_pop && b_stuff_next_o && b_cfg_mode_i != EQUAL);
        c_rt_drop_eq:   cover (rst_n && b_pop && b_stuff_next_o && b_cfg_mode_i == EQUAL);
        // the round-trip check fires on a data bit after a dropped stuff bit
        c_rt_watch_ones:    cover (rst_n && w_hit && f_drop_fr && b_cfg_mode_i != EQUAL);
        c_rt_watch_eq:      cover (rst_n && w_hit && f_drop_fr && b_cfg_mode_i == EQUAL);
        c_rt_watch_ones_n7: cover (rst_n && w_hit && f_drop_fr && b_cfg_mode_i != EQUAL && b_cfg_n_i == 3'd7);
        c_rt_watch_eq_n7:   cover (rst_n && w_hit && f_drop_fr && b_cfg_mode_i == EQUAL && b_cfg_n_i == 3'd7);
        c_rt_watch_ones_n1: cover (rst_n && w_hit && f_drop_fr && b_cfg_mode_i != EQUAL && b_cfg_n_i == 3'd1);
        c_rt_watch_eq_n2:   cover (rst_n && w_hit && f_drop_fr && b_cfg_mode_i == EQUAL && b_cfg_n_i == 3'd2);
        c_rt_watch_frame2:  cover (rst_n && w_hit && f_drop_fr && f_frame2);
        c_rt_watch_newcfg:  cover (rst_n && w_hit && f_drop_fr && f_newcfg && f_drop);
        // the watched bit has another data bit ahead of it
        c_rt_watch_behind:  cover (rst_n && w_armed && w_ahead != '0);
        // max run reached exactly (bound is tight)
        c_rt_run_ones_n7: cover (rst_n && a_active && a_cfg_mode_i != EQUAL && a_cfg_n_i == 3'd7 && wr == 4'd7);
        c_rt_run_eq_n7:   cover (rst_n && a_active && a_cfg_mode_i == EQUAL && a_cfg_n_i == 3'd7 && wr == 4'd7);
        // disabled (incl. EQUAL with N = 1): long runs pass through unchanged
        c_rt_dis_run:     cover (rst_n && !a_active && wr == 4'd9 && !f_drop);
        c_rt_dis_eq_n1:   cover (rst_n && a_cfg_en_i && a_cfg_mode_i == EQUAL && a_cfg_n_i == 3'd1 && wr == 4'd4);
        // wire full, and A and B out of step (timing really decoupled)
        c_rt_full:        cover (rst_n && cnt == DEPTH);
        c_rt_skew:        cover (rst_n && a_active && !pend && a_stuff_next_o != b_stuff_next_o);
    end

endmodule
