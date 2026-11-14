`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_stuff (bit stuffing).
//
//  Written from docs/blocks/stuff.md ONLY (the RTL was not read). Every
//  property is about the block's ports.
//
//  Two checker modules:
//
//    stuff_props     one instance, free inputs. An executable model of the
//                    "Behaviour" section (run, last, history) is driven from
//                    the input ports, and every output is compared with it
//                    every cycle. That covers required properties 3 and 4
//                    and the exact cycle behaviour; property 3 and the reset,
//                    clear and disabled rules are also stated directly on the
//                    ports, without the model.
//
//    stuff_rt_props  a transmitter instance A and a receiver instance B joined
//                    by a lossless, order-preserving wire with independent
//                    timing on each side (built in stuff_fv.sv). Covers
//                    required properties 1 (max run on the wire) and 2
//                    (round trip, no violation on a legal stream). Round trip
//                    is proved with a symbolic watched data bit, so the proof
//                    is unbounded.
//
//  Environment (conventions.md): rst_n low in the first cycle; cfg_* stable
//  (we assume constant over the whole trace). rst_n is free afterwards, so
//  resets in the middle of a trace are also checked.
//
//  Spec reading that matters (see the spec-issue list in the report):
//   * stuff_bit_o is checked only while stuff_next_o = 1 (ports table:
//     "meaningful when stuff_next_o"); the Behaviour section's formula is
//     not enforced at other times.
//   * Outputs are compared with the model including cycles where rst_n is
//     low (asynchronous reset: outputs are 0 during reset).
//   * Run counter saturates at 7 exactly as written; this makes N = 7 behave
//     differently from N < 7 after a violation (two cycles of violation_o in
//     a row are possible only for N = 7). Properties follow the spec text.
// ===========================================================================

module stuff_props #(
    // 1: stand-alone proof (environment rules are assumed, covers active).
    // 0: monitor inside a larger proof (environment rules are asserted,
    //    covers off).
    parameter STANDALONE = 1
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
            always @(posedge clk) begin
                if (!$initstate) begin
                    env_cfg: assume (cfg_en_i   == $past(cfg_en_i)  &&
                                     cfg_n_i    == $past(cfg_n_i)   &&
                                     cfg_mode_i == $past(cfg_mode_i));
                end
            end
        end else begin : g_env_assert
            always @* begin
                if ($initstate) begin
                    env_rst: assert (!rst_n);
                end
            end
            always @(posedge clk) begin
                if (!$initstate) begin
                    env_cfg: assert (cfg_en_i   == $past(cfg_en_i)  &&
                                     cfg_n_i    == $past(cfg_n_i)   &&
                                     cfg_mode_i == $past(cfg_mode_i));
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Executable model of "Behaviour" (state: run 0-7, last, history)
    // ---------------------------------------------------------------------
    localparam ONES  = 1'b0;
    localparam EQUAL = 1'b1;

    logic [2:0] m_run;
    logic       m_last;
    logic       m_hist;
    logic       m_viol;

    wire       m_on    = cfg_en_i && (cfg_n_i != 3'd0);
    wire       m_sn    = m_on && (m_run == cfg_n_i);           // stuff_next_o
    wire       m_sb    = (cfg_mode_i == EQUAL) ? ~m_last : 1'b0; // stuff_bit_o
    wire [2:0] m_inc   = (m_run == 3'd7) ? 3'd7 : m_run + 3'd1; // saturating
    wire       m_take  = bit_valid_i && !clear_i;               // clear wins

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_run  <= 3'd0;
            m_last <= 1'b0;
            m_hist <= 1'b0;
            m_viol <= 1'b0;
        end else if (clear_i) begin
            m_run  <= 3'd0;
            m_hist <= 1'b0;
            m_viol <= 1'b0;
        end else if (bit_valid_i) begin
            m_viol <= m_sn && (bit_i != m_sb);
            if (cfg_mode_i == ONES) begin
                m_run <= bit_i ? m_inc : 3'd0;
            end else begin
                m_run  <= (m_hist && bit_i == m_last) ? m_inc : 3'd1;
                m_last <= bit_i;
                m_hist <= 1'b1;
            end
        end else begin
            m_viol <= 1'b0;
        end
    end

    // ---------------------------------------------------------------------
    // Exact behaviour: outputs equal the model in every cycle
    // ---------------------------------------------------------------------
    always @* begin
        a_sn_exact:   assert (stuff_next_o == m_sn);
        a_viol_exact: assert (violation_o  == m_viol);
        if (m_sn) begin
            a_sb_exact: assert (stuff_bit_o == m_sb);
        end
    end

    // ---------------------------------------------------------------------
    // Required property 4: reset values, clear priority, disabled behaviour
    // (stated directly on the ports; redundant with the model on purpose)
    // ---------------------------------------------------------------------
    always @* begin
        if (!rst_n) begin
            a_rst_sn:   assert (!stuff_next_o);
            a_rst_viol: assert (!violation_o);
        end
        if (!(cfg_en_i && cfg_n_i != 3'd0)) begin
            a_dis_sn:   assert (!stuff_next_o);
            a_dis_viol: assert (!violation_o);
        end
        if (stuff_next_o && cfg_mode_i == ONES) begin
            a_ones_sb0: assert (stuff_bit_o == 1'b0);
        end
    end

    always @(posedge clk) begin
        if (!$initstate && rst_n) begin
            // first cycle after reset release
            if (!$past(rst_n)) begin
                a_rel_sn:   assert (!stuff_next_o);
                a_rel_viol: assert (!violation_o);
            end
            // clear_i wins over bit_valid_i and forgets history
            if ($past(rst_n) && $past(clear_i)) begin
                a_clr_sn:   assert (!stuff_next_o);
                a_clr_viol: assert (!violation_o);
            end
            // Required property 3, stated on the ports only: violation_o is
            // 1 exactly in the cycle after a wire bit arrived at a stuff
            // position with the wrong value.
            a_viol_port: assert (violation_o ==
                ($past(rst_n) && $past(bit_valid_i) && !$past(clear_i) &&
                 $past(stuff_next_o) && ($past(bit_i) != $past(stuff_bit_o))));
            // no bit, no clear: state holds, no violation
            if ($past(rst_n) && !$past(clear_i) && !$past(bit_valid_i)) begin
                a_idle_sn:   assert (stuff_next_o == $past(stuff_next_o));
                a_idle_viol: assert (!violation_o);
                if (stuff_next_o) begin
                    a_idle_sb: assert (stuff_bit_o == $past(stuff_bit_o));
                end
            end
            // a correct stuff bit is never demanded twice in a row, except in
            // EQUAL mode with N = 1 (spec issue: the stuff bit itself is a run
            // of length 1 = N, so stuffing never ends)
            if ($past(rst_n) && $past(m_take) && $past(stuff_next_o) &&
                ($past(bit_i) == $past(stuff_bit_o)) &&
                !(cfg_mode_i == EQUAL && cfg_n_i == 3'd1)) begin
                a_stuff_then_data: assert (!stuff_next_o);
            end
            // back-to-back violations only possible for N = 7 (saturation)
            if (violation_o && $past(violation_o)) begin
                a_viol_b2b_n7: assert (cfg_n_i == 3'd7);
            end
        end
    end

    // model sanity: in EQUAL mode "history" is exactly run != 0
    always @* begin
        if (cfg_mode_i == EQUAL) begin
            a_model_hist: assert (m_hist == (m_run != 3'd0));
        end
    end

    // ---------------------------------------------------------------------
    // Covers (stand-alone only)
    // ---------------------------------------------------------------------
    logic f_clr_mid;   // a clear happened while a run was partly built
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            f_clr_mid <= 1'b0;
        else if (clear_i && m_on && m_run != 3'd0 && !m_sn)
            f_clr_mid <= 1'b1;
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
                    // violation detected, each mode, and back to back (N = 7)
                    c_viol_ones:   cover (violation_o && cfg_mode_i == ONES);
                    c_viol_eq:     cover (violation_o && cfg_mode_i == EQUAL);
                    c_viol_b2b:    cover (violation_o && $past(violation_o));
                    // clear while a stuff bit is pending; clear together with a bit
                    c_clr_pending: cover (rst_n && clear_i && stuff_next_o);
                    c_clr_and_bit: cover (rst_n && clear_i && bit_valid_i && m_on &&
                                          m_run != 3'd0 && !m_sn);
                    // clear mid-run, then a full new run up to a stuff request
                    c_clr_mid_then_stuff: cover (f_clr_mid && stuff_next_o && cfg_n_i == 3'd3);
                    // idle cycles while a stuff bit is pending
                    c_idle_pending: cover (rst_n && stuff_next_o && $past(stuff_next_o) &&
                                           $past(!bit_valid_i && !clear_i && rst_n));
                    // saturation after a violation (N < 7)
                    c_saturate:    cover (m_on && m_run == 3'd7 && cfg_n_i == 3'd2);
                end
                // N = 1 and N = 7 in each mode
                c_ones_n1: cover (stuff_next_o && cfg_mode_i == ONES  && cfg_n_i == 3'd1);
                c_ones_n7: cover (stuff_next_o && cfg_mode_i == ONES  && cfg_n_i == 3'd7);
                c_eq_n1:   cover (stuff_next_o && cfg_mode_i == EQUAL && cfg_n_i == 3'd1);
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
            end
        end
    endgenerate

endmodule


// ===========================================================================
//  Round trip and wire run length (required properties 1 and 2).
//
//  Observes transmitter instance A (a_*) and receiver instance B (b_*).
//  A's bit_valid_i/bit_i are the wire as transmitted; B's are the wire as
//  received. The channel between them must be lossless and order-preserving
//  with at most DEPTH bits in flight; frames are delimited by clear: A clears
//  only when the wire is empty, and B clears once, after A, before it takes
//  any bit of the new frame. These channel rules are checked (env_*).
//
//  Data bit sent    = A takes a bit while A.stuff_next_o = 0.
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
    input wire       cfg_en_i,
    input wire [2:0] cfg_n_i,
    input wire       cfg_mode_i,
    // instance A: transmitter side
    input wire       a_clear_i,
    input wire       a_bit_valid_i,
    input wire       a_bit_i,
    input wire       a_stuff_next_o,
    input wire       a_stuff_bit_o,
    input wire       a_violation_o,
    // instance B: receiver side
    input wire       b_clear_i,
    input wire       b_bit_valid_i,
    input wire       b_bit_i,
    input wire       b_stuff_next_o,
    input wire       b_stuff_bit_o,
    input wire       b_violation_o
);
    localparam CW = $clog2(DEPTH + 1) + 1;   // one spare bit to see overflow
    localparam EQUAL = 1'b1;

    wire a_push  = a_bit_valid_i && !a_clear_i;
    wire b_pop   = b_bit_valid_i && !b_clear_i;
    wire tx_data = a_push && !a_stuff_next_o;
    wire rx_data = b_pop  && !b_stuff_next_o;
    wire on      = cfg_en_i && cfg_n_i != 3'd0;

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
    always @* begin
        if (rst_n) begin
            env_rt_tx_obeys: assert (!(a_push && a_stuff_next_o) || a_bit_i == a_stuff_bit_o);
            env_rt_no_under: assert (!b_pop || cnt != '0);
            env_rt_no_over:  assert (!(a_push && !b_pop && cnt == DEPTH));
            env_rt_chan_bit: assert (!b_pop || b_bit_i == q_bit[0]);
            env_rt_a_clr:    assert (!a_clear_i || (cnt == '0 && !pend && !b_bit_valid_i));
            env_rt_b_clr:    assert (!b_clear_i || pend);
            env_rt_b_wait:   assert (!b_bit_valid_i || !pend);
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
    // Required property 1: run length on the wire (as transmitted)
    // ---------------------------------------------------------------------
    logic [3:0] wr;          // current run on the wire, saturating at 15
    logic       wr_last, wr_hist;
    logic       f_sent;      // A has put a bit on the wire in this frame
    logic       f_prev_stuff;// the previous wire bit of this frame was a stuff bit
    wire  [3:0] wr_inc = (wr == 4'd15) ? 4'd15 : wr + 4'd1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr <= 4'd0; wr_last <= 1'b0; wr_hist <= 1'b0;
            f_sent <= 1'b0; f_prev_stuff <= 1'b0;
        end else if (a_clear_i) begin
            wr <= 4'd0; wr_hist <= 1'b0;
            f_sent <= 1'b0; f_prev_stuff <= 1'b0;
        end else if (a_push) begin
            if (cfg_mode_i == EQUAL)
                wr <= (wr_hist && a_bit_i == wr_last) ? wr_inc : 4'd1;
            else
                wr <= a_bit_i ? wr_inc : 4'd0;
            wr_last      <= a_bit_i;
            wr_hist      <= 1'b1;
            f_sent       <= 1'b1;
            f_prev_stuff <= a_stuff_next_o;
        end
    end

    always @* begin
        if (rst_n && on) begin
            rt_max_run: assert (wr <= {1'b0, cfg_n_i});
        end
    end

    // progress: a stuff bit is always followed by a data bit, except EQUAL
    // with N = 1, where (spec issue) stuffing never ends after the first bit
    always @* begin
        if (rst_n && a_push && a_stuff_next_o && f_prev_stuff) begin
            rt_progress: assert (cfg_mode_i == EQUAL && cfg_n_i == 3'd1);
        end
        if (rst_n && cfg_en_i && cfg_mode_i == EQUAL && cfg_n_i == 3'd1 && f_sent) begin
            rt_eq_n1_livelock: assert (a_stuff_next_o);   // documents the spec issue
        end
    end

    // ---------------------------------------------------------------------
    // Covers
    // ---------------------------------------------------------------------
    logic f_drop;      // B has dropped a stuff bit since reset
    logic f_frame2;    // a second frame (after a clear) has started
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_drop <= 1'b0; f_frame2 <= 1'b0;
        end else begin
            if (b_pop && b_stuff_next_o) f_drop <= 1'b1;
            if (b_clear_i)               f_frame2 <= 1'b1;
        end
    end

    always @(posedge clk) begin
        // stuff bit inserted by A and dropped by B, each mode
        c_rt_ins_ones:  cover (rst_n && a_push && a_stuff_next_o && cfg_mode_i != EQUAL);
        c_rt_ins_eq:    cover (rst_n && a_push && a_stuff_next_o && cfg_mode_i == EQUAL);
        c_rt_drop_ones: cover (rst_n && b_pop && b_stuff_next_o && cfg_mode_i != EQUAL);
        c_rt_drop_eq:   cover (rst_n && b_pop && b_stuff_next_o && cfg_mode_i == EQUAL);
        // the round-trip check fires on a data bit after a dropped stuff bit
        c_rt_watch_ones:    cover (rst_n && w_hit && f_drop && cfg_mode_i != EQUAL);
        c_rt_watch_eq:      cover (rst_n && w_hit && f_drop && cfg_mode_i == EQUAL);
        c_rt_watch_ones_n7: cover (rst_n && w_hit && f_drop && cfg_mode_i != EQUAL && cfg_n_i == 3'd7);
        c_rt_watch_eq_n7:   cover (rst_n && w_hit && f_drop && cfg_mode_i == EQUAL && cfg_n_i == 3'd7);
        c_rt_watch_ones_n1: cover (rst_n && w_hit && f_drop && cfg_mode_i != EQUAL && cfg_n_i == 3'd1);
        c_rt_watch_eq_n2:   cover (rst_n && w_hit && f_drop && cfg_mode_i == EQUAL && cfg_n_i == 3'd2);
        c_rt_watch_frame2:  cover (rst_n && w_hit && f_drop && f_frame2);
        // the watched bit has another data bit ahead of it (in_flight > 1)
        c_rt_watch_behind:  cover (rst_n && w_armed && w_ahead != '0);
        // max run reached exactly (bound is tight)
        c_rt_run_ones_n7: cover (rst_n && on && cfg_mode_i != EQUAL && cfg_n_i == 3'd7 && wr == 4'd7);
        c_rt_run_eq_n7:   cover (rst_n && on && cfg_mode_i == EQUAL && cfg_n_i == 3'd7 && wr == 4'd7);
        // disabled: long runs pass through unchanged
        c_rt_dis_run:     cover (rst_n && !on && wr == 4'd9 && f_drop == 1'b0);
        // wire full, and A and B out of step (timing really decoupled)
        c_rt_full:        cover (rst_n && cnt == DEPTH);
        c_rt_skew:        cover (rst_n && on && a_stuff_next_o != b_stuff_next_o);
    end

endmodule
