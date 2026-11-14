`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_chan (lane channels and timestamp).
//
//  Written from docs/blocks/channels.md ONLY (the RTL was not read). Every
//  property is about the block's ports.
//
//  Contents
//   * Executable model of section 2 (data[0..3], valid[0..3], ts), driven
//     from the input ports. ch_valid_o, ch_data_o, ts_o and hreg_rdata_o are
//     compared with it in EVERY cycle, including reset cycles (required
//     properties 5, 6 and the exact cycle behaviour).
//   * Required properties 1-5 stated again directly on the ports, without
//     the model (a_ts_*, a_put_*, a_get_*, a_frame_*, a_clr_*, a_rd_*,
//     a_rst_*, a_rel_*). They read the state from the registered outputs:
//     ch_valid_o / ch_data_o ARE valid / data (spec section 1, "registered").
//   * Environment rules (env_*): what the sequencer and the host guarantee
//     at this block's ports. Assumed (STANDALONE = 1) or asserted
//     (STANDALONE = 0, monitor inside a chip-level proof). ENV_SEQ = 0 drops
//     the sequencer rules (keeping only reset and env_clr_op), so the
//     "second guard" behaviour of section 2 (put to a full channel, get from
//     an empty one, put + get in one cycle) is proved for free inputs.
//   * Covers.
//
//  Timestamp. Required property 1 is stated inductively:
//     a_ts_rst  ts_o = 0 while rst_n = 0
//     a_ts_rel  ts_o = 0 in the first cycle with rst_n = 1 (cycle 0)
//     a_ts_inc  ts_o(t+1) = ts_o(t) + 1 mod 2^32 when rst_n = 1 in t and t+1
//  By induction on t these three fix ts_o in every cycle of every trace:
//  ts_o(t) = (t - t_rel) mod 2^32, t_rel = the last reset release, and 0 in
//  reset. That is exactly section 2 ("ts_o = t mod 2^32 in cycle t, counted
//  from internal reset release"), so nothing about absolute time is left
//  unstated. None of them mentions absolute time, so an unbounded engine
//  never has to unroll 2^32 cycles: the step property is checked from every
//  ts value, 0xFFFFFFFF included. (m_ts below is the same counter.)
//
//  Assertions inside generate loops carry their name as a trailing comment
//  (Yosys does not scope labels per loop iteration).
//
//  Spec readings that matter (see the report):
//   * "Judged on the valid bit at the start of the cycle" = ch_valid_o of
//     that cycle. Put and get in one cycle name the same channel (one
//     ch_idx_i): empty -> the put succeeds, the get is ignored; valid -> the
//     get succeeds, the put is ignored.
//   * Host clear acts on hreg_we_i && hreg_addr_i == 0x06 in any cycle
//     (ruling 4: the block cannot see ph): valid[c] <= 0 for each
//     hreg_wdata_i[c] = 1; hreg_wdata_i[31:4] ignored; data never changed.
//   * A CHSTAT write together with a put or get is excluded by assumption
//     (env_clr_op, ruling 1), in every stand-alone instance, so required
//     property 2 holds without exception and the "clear beats put" priority
//     of section 2 is never exercised (see the report).
//   * Host writes to other addresses (TIME, CHDATA, unowned) change nothing
//     (ruling 3).
//   * hreg_rdata_o is a function of all 7 hreg_addr_i bits and the state of
//     the same cycle, independent of hreg_we_i (combinational; a read in a
//     write cycle shows the old state) (ruling 3).
// ===========================================================================

module chan_props #(
    // 1: stand-alone proof (environment rules are assumed, covers active).
    // 0: monitor inside a larger proof (environment rules are asserted,
    //    covers off).
    parameter STANDALONE   = 1,
    // 1: the sequencer/host guarantees on ch_put_i / ch_get_i / hreg_we_i
    //    are part of the environment (assumed or asserted per STANDALONE).
    // 0: they are neither assumed nor asserted (free inputs: proves the
    //    block's own second guard). Reset in cycle 0 is kept.
    parameter ENV_SEQ      = 1,
    // 1: proof-only free initial state. The first cycle need not be in
    //    reset; the model starts equal to the block's outputs. Used only to
    //    make the timestamp wrap reachable. Same assertions (from a superset
    //    of the initial states); only the wrap covers are active.
    parameter FREE_INIT    = 0
) (
    input wire         clk,
    input wire         rst_n,
    input wire         ch_put_i,
    input wire         ch_get_i,
    input wire [1:0]   ch_idx_i,
    input wire [31:0]  ch_wdata_i,
    input wire [3:0]   ch_valid_o,
    input wire [127:0] ch_data_o,
    input wire [31:0]  ts_o,
    input wire         hreg_we_i,
    input wire [6:0]   hreg_addr_i,
    input wire [31:0]  hreg_wdata_i,
    input wire [31:0]  hreg_rdata_o
);
    localparam [6:0] A_TIME   = 7'h03;
    localparam [6:0] A_CHSTAT = 7'h06;
    localparam [6:0] A_CHDATA = 7'h08;   // 0x08-0x0B

    // ---------------------------------------------------------------------
    // Port-side helpers (state read from the registered outputs)
    // ---------------------------------------------------------------------
    wire        clr    = hreg_we_i && (hreg_addr_i == A_CHSTAT);
    wire [3:0]  clr_m  = clr ? hreg_wdata_i[3:0] : 4'h0;   // channels cleared
    wire [3:0]  sel    = 4'b0001 << ch_idx_i;
    wire        v_idx  = ch_valid_o[ch_idx_i];
    wire [31:0] d_idx  = ch_data_o[32*ch_idx_i +: 32];
    wire        put_ok = ch_put_i && !v_idx;
    wire        get_ok = ch_get_i &&  v_idx;
    wire        rd_chdata = (hreg_addr_i[6:2] == A_CHDATA[6:2]);
    wire        rd_owned  = (hreg_addr_i == A_TIME) || (hreg_addr_i == A_CHSTAT) ||
                            rd_chdata;

    // ---------------------------------------------------------------------
    // Environment. All rules apply outside reset; in reset the inputs are
    // free (the block must ignore them).
    //  env_clr_op    no CHSTAT write in a cycle with a put or get (ruling 1;
    //                host writes only at ph = 1, where sequencer.md
    //                property 3 forbids channel ops). Every stand-alone
    //                instance, ENV_SEQ or not.
    //  env_one_op    at most one of put/get (sequencer.md property 3)
    //  env_put_empty put only to an empty channel (sequencer.md property 4)
    //  env_get_valid get only from a valid channel (sequencer.md property 4)
    // The block has no ph input; writes to other addresses may coincide with
    // lane operations here (allowed, so the proof is stronger).
    // ---------------------------------------------------------------------
    wire env_clr  = !(clr && (ch_put_i || ch_get_i));
    wire env_one  = !(ch_put_i && ch_get_i);
    wire env_put  = !ch_put_i || !ch_valid_o[ch_idx_i];
    wire env_get  = !ch_get_i ||  ch_valid_o[ch_idx_i];

    generate
        if (STANDALONE) begin : g_env_assume
            always @* begin
                if ($initstate && !FREE_INIT) begin
                    env_rst: assume (!rst_n);
                end
                if (rst_n) begin
                    env_clr_op: assume (env_clr);
                end
                if (ENV_SEQ && rst_n) begin
                    env_one_op:    assume (env_one);
                    env_put_empty: assume (env_put);
                    env_get_valid: assume (env_get);
                end
            end
        end else begin : g_env_assert
            always @* begin
                if ($initstate) begin
                    env_rst: assert (!rst_n);
                end
                if (rst_n) begin
                    env_clr_op: assert (env_clr);
                end
                if (ENV_SEQ && rst_n) begin
                    env_one_op:    assert (env_one);
                    env_put_empty: assert (env_put);
                    env_get_valid: assert (env_get);
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Executable model of section 2 (required property 6, linearisation:
    // the operations of each cycle applied in cycle order to a 4-entry
    // valid/data array; within a cycle every lane operation is judged on
    // the valid bits at the start of the cycle; the host clear is applied
    // last, as section 2 gives it priority, although env_clr_op keeps it
    // apart from lane operations).
    // ---------------------------------------------------------------------
    logic [3:0]   m_valid;
    logic [127:0] m_data;
    logic [31:0]  m_ts;
    genvar g;

    wire        m_vi    = m_valid[ch_idx_i];
    wire        m_put   = ch_put_i && !m_vi;          // successful put
    wire        m_get   = ch_get_i &&  m_vi;          // successful get
    wire [3:0]  m_valid_n = ((m_valid & ~(m_get ? sel : 4'h0)) |
                             (m_put ? sel : 4'h0)) & ~clr_m;
    logic [127:0] m_data_n;
    always @* begin
        m_data_n = m_data;
        if (m_put) m_data_n[32*ch_idx_i +: 32] = ch_wdata_i;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_valid <= 4'h0;
            m_data  <= 128'h0;
            m_ts    <= 32'h0;
        end else begin
            m_valid <= m_valid_n;
            m_data  <= m_data_n;
            m_ts    <= m_ts + 32'd1;
        end
    end

    logic [31:0] m_rdata;
    always @* begin
        if (hreg_addr_i == A_TIME)        m_rdata = m_ts;
        else if (hreg_addr_i == A_CHSTAT) m_rdata = {28'h0, m_valid};
        else if (rd_chdata)               m_rdata = m_data[32*hreg_addr_i[1:0] +: 32];
        else                              m_rdata = 32'h0;
    end

    generate
        if (FREE_INIT) begin : g_free_init
            always @* begin
                if ($initstate) begin
                    fi_valid: assume (m_valid == ch_valid_o);
                    fi_data:  assume (m_data  == ch_data_o);
                    fi_ts:    assume (m_ts    == ts_o);
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Exact behaviour: every output equals the model in every cycle,
    // reset cycles included.
    // ---------------------------------------------------------------------
    // (`define CHAN_NO_MODEL_CHECK removes these four, to show that the
    // port-level statements below catch the same bugs on their own.)
`ifndef CHAN_NO_MODEL_CHECK
    always @* begin
        a_m_valid: assert (ch_valid_o   == m_valid);
        a_m_data:  assert (ch_data_o    == m_data);
        a_m_ts:    assert (ts_o         == m_ts);
        a_m_rdata: assert (hreg_rdata_o == m_rdata);
    end
`endif

    // ---------------------------------------------------------------------
    // Port-level statements (no model)
    // ---------------------------------------------------------------------
    // Reset (asynchronous): all state 0, so every read returns 0.
    always @* begin
        if (!rst_n) begin
            a_rst_valid: assert (ch_valid_o   == 4'h0);
            a_rst_data:  assert (ch_data_o    == 128'h0);
            a_ts_rst:    assert (ts_o         == 32'h0);
            a_rst_rdata: assert (hreg_rdata_o == 32'h0);
        end
    end

    // Required property 5 (reads): TIME = ts_o, CHSTAT = {28'h0, valid},
    // CHDATA c = data[c], 0 for every other address.
    always @* begin
        if (hreg_addr_i == A_TIME) begin
            a_rd_time:   assert (hreg_rdata_o == ts_o);
        end
        if (hreg_addr_i == A_CHSTAT) begin
            a_rd_chstat: assert (hreg_rdata_o == {28'h0, ch_valid_o});
        end
        if (rd_chdata) begin
            a_rd_chdata: assert (hreg_rdata_o == ch_data_o[32*hreg_addr_i[1:0] +: 32]);
        end
        if (!rd_owned) begin
            a_rd_zero:   assert (hreg_rdata_o == 32'h0);
        end
    end

    // Required property 1 and reset release
    always @(posedge clk) begin
        if (!$initstate && rst_n) begin
            if (!$past(rst_n)) begin
                a_ts_rel:    assert (ts_o       == 32'h0);
                a_rel_valid: assert (ch_valid_o == 4'h0);
                a_rel_data:  assert (ch_data_o  == 128'h0);
            end else begin
                a_ts_inc:    assert (ts_o == $past(ts_o) + 32'd1);
            end
        end
    end

    // Required properties 2, 3, 4, 5 (host clear), per channel c.
    // i = ch_idx_i at t; "put"/"get" below = the lane op names channel c.
    generate
        for (g = 0; g < 4; g = g + 1) begin : g_ch
            wire [31:0] d   = ch_data_o[32*g +: 32];
            wire        v   = ch_valid_o[g];
            wire        nm  = (ch_idx_i == g);
            wire        put = ch_put_i && nm;
            wire        get = ch_get_i && nm;
            wire        hc  = clr_m[g];

            always @(posedge clk) begin
                if (!$initstate && rst_n && $past(rst_n)) begin
                    // 2. put to an empty channel: valid = 1, data = wdata(t)
                    //    (no exception: env_clr_op excludes a clear with it)
                    if ($past(put && !v)) begin
                        assert (v);  // a_put_valid
                        assert (d == $past(ch_wdata_i));  // a_put_data
                    end
                    // 2. put to a valid channel changes nothing: data kept;
                    //    valid stays 1 unless a get also acted
                    if ($past(put && v)) begin
                        assert (d == $past(d));  // a_putfull_data
                        if ($past(!ch_get_i)) begin
                            assert (v);  // a_putfull_valid
                        end
                    end
                    // 3. get from a valid channel: valid = 0, data unchanged
                    if ($past(get && v)) begin
                        assert (!v);  // a_get_valid
                        assert (d == $past(d));  // a_get_data
                    end
                    // 3. get from an empty channel does nothing (a put in the
                    //    same cycle is judged on its own)
                    if ($past(get && !v && !ch_put_i)) begin
                        assert (!v);  // a_getempty_valid
                        assert (d == $past(d));  // a_getempty_data
                    end
                    // 4. frame: data changes only by a successful put to c
                    if (d != $past(d)) begin
                        assert ($past(put && !v));  // a_frame_data
                    end
                    // 4. valid rises only by a successful put, falls only by a
                    //    successful get or a host clear of c
                    if (v && !$past(v)) begin
                        assert ($past(put));  // a_frame_rise
                    end
                    if (!v && $past(v)) begin
                        assert ($past(get || hc));  // a_frame_fall
                    end
                    // 4. lane operations on other channels (or none) and no
                    //    host clear of c: c unchanged
                    if ($past(!(put || get) && !hc)) begin
                        assert (d == $past(d));  // a_frame_other_data
                        assert (v == $past(v));  // a_frame_other_valid
                    end
                    // 5. host clear of c: valid = 0, data unchanged
                    if ($past(hc)) begin
                        assert (!v);  // a_clr_valid
                        assert (d == $past(d));  // a_clr_data
                    end
                    // 5. CHSTAT write that does not clear c (bit c = 0,
                    //    including writes of [31:4] only): c unchanged
                    if ($past(clr && !hc && !(put || get))) begin
                        assert (v == $past(v));  // a_clr_other_valid
                    end
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Cover helpers
    // ---------------------------------------------------------------------
    logic [3:0] f_got;      // channel c: a value was put and then taken
    logic       f_midrst;   // a reset happened after activity
    logic       f_wrap;     // ts wrapped in this trace (since last reset)
    logic       f_seen_rst; // a reset has been seen
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_got  <= 4'h0;
            f_wrap <= 1'b0;
        end else begin
            if (get_ok) f_got <= f_got | sel;
            if (ts_o == 32'hFFFF_FFFF) f_wrap <= 1'b1;
        end
    end
    always @(posedge clk) begin
        if ($initstate) begin
            f_midrst   <= 1'b0;
            f_seen_rst <= !rst_n;
        end else begin
            if (!rst_n) f_seen_rst <= 1'b1;
            if (!rst_n && $past(rst_n && ch_valid_o != 4'h0)) f_midrst <= 1'b1;
        end
    end

    // ---------------------------------------------------------------------
    // Covers (stand-alone only)
    // ---------------------------------------------------------------------
    generate
        if (STANDALONE && ENV_SEQ && !FREE_INIT) begin : g_cover_env
            // Under the sequencer/host guarantees, from reset.
            always @(posedge clk) begin
                if (!$initstate) begin
                    c_rst_release: cover (rst_n && !$past(rst_n));
                    // host clear of valid channels; all four at once; part;
                    // of an empty channel (no effect); upper data bits only
                    c_hclr:         cover (rst_n && $past(rst_n && (clr_m & ch_valid_o) != 4'h0));
                    c_hclr_all:     cover (rst_n && $past(rst_n && clr && ch_valid_o == 4'hF &&
                                                      hreg_wdata_i[3:0] == 4'hF));
                    c_hclr_part:    cover (rst_n && $past(rst_n && clr && ch_valid_o == 4'hF &&
                                                      hreg_wdata_i[3:0] == 4'b0101));
                    c_hclr_empty:   cover (rst_n && $past(rst_n && clr && ch_valid_o != 4'h0 &&
                                                      hreg_wdata_i[3:0] != 4'h0 &&
                                                      (hreg_wdata_i[3:0] & ch_valid_o) == 4'h0));
                    c_hclr_upper:   cover (rst_n && $past(rst_n && clr && ch_valid_o == 4'hF &&
                                                      hreg_wdata_i[3:0] == 4'h0 &&
                                                      hreg_wdata_i[31:4] != 28'h0));
                    // host writes to other addresses with clear-like data
                    // (TIME, CHDATA0, an alias of CHSTAT): no effect
                    c_hwr_time:     cover (rst_n && $past(rst_n && hreg_we_i && hreg_addr_i == A_TIME &&
                                                      ch_valid_o == 4'hF && hreg_wdata_i == 32'hFFFF_FFFF));
                    c_hwr_chdata:   cover (rst_n && $past(rst_n && hreg_we_i && hreg_addr_i == A_CHDATA &&
                                                      ch_valid_o == 4'hF && hreg_wdata_i == 32'hFFFF_FFFF));
                    c_hwr_alias:    cover (rst_n && $past(rst_n && hreg_we_i && hreg_addr_i == 7'h46 &&
                                                      ch_valid_o == 4'hF && hreg_wdata_i[3:0] == 4'hF));
                    // reset in the middle of a trace with full channels, then
                    // a new put after it
                    c_rst_mid:      cover (!rst_n && $past(rst_n && ch_valid_o == 4'hF));
                    c_rst_mid_put:  cover (rst_n && f_midrst && ch_valid_o != 4'h0);
                end
                // host clear in the same cycle as other activity the
                // environment allows: a CHSTAT read of the old value, ts
                // counting, a clear of all four channels
                c_hclr_read:   cover (rst_n && clr && hreg_rdata_o == 32'hF && hreg_wdata_i[3:0] == 4'hF);
                // every channel put then got in one trace
                c_all_got:     cover (rst_n && f_got == 4'hF);
                // all four full with four different words
                c_all_full:    cover (rst_n && ch_valid_o == 4'hF &&
                                      ch_data_o[31:0]  != ch_data_o[63:32] &&
                                      ch_data_o[31:0]  != ch_data_o[95:64] &&
                                      ch_data_o[31:0]  != ch_data_o[127:96] &&
                                      ch_data_o[63:32] != ch_data_o[95:64] &&
                                      ch_data_o[63:32] != ch_data_o[127:96] &&
                                      ch_data_o[95:64] != ch_data_o[127:96]);
                // reads of every owned register with a non-zero value, and
                // of unowned addresses (incl. aliases) with non-zero state
                c_rd_time:     cover (rst_n && hreg_addr_i == A_TIME   && hreg_rdata_o != 32'h0);
                c_rd_chstat:   cover (rst_n && hreg_addr_i == A_CHSTAT && hreg_rdata_o == 32'hF);
                c_rd_unowned:  cover (rst_n && !rd_owned && ch_valid_o == 4'hF && ts_o != 32'h0);
                c_rd_alias46:  cover (rst_n && hreg_addr_i == 7'h46 && ch_valid_o != 4'h0);
                c_rd_alias48:  cover (rst_n && hreg_addr_i == 7'h48 && ch_data_o[31:0] != 32'h0);
                // environment assumptions are satisfiable with activity
                c_env_put:     cover (rst_n && ch_put_i);
                c_env_get:     cover (rst_n && ch_get_i);
                c_env_hwe:     cover (rst_n && hreg_we_i);
            end
            for (g = 0; g < 4; g = g + 1) begin : g_cov_ch
                wire [31:0] d  = ch_data_o[32*g +: 32];
                wire        v  = ch_valid_o[g];
                wire        nm = (ch_idx_i == g);
                always @(posedge clk) begin
                    if (!$initstate) begin
                        // put then get on channel c, the taken value still
                        // readable afterwards (non-zero, so it is the put's)
                        cover (rst_n && $past(rst_n && ch_get_i && nm && v) &&
                                           !v && d != 32'h0);  // c_put_get
                        // get in the cycle right after the put (latency 1)
                        cover (rst_n && $past(rst_n && ch_get_i && nm && v) &&
                                           $past(rst_n && ch_put_i && nm && !v, 2));  // c_put_get1
                        // refill: a new, different value after a take
                        cover (rst_n && $past(rst_n && ch_put_i && nm && !v &&
                                                      f_got[g] && ch_wdata_i != d) && v);  // c_refill
                        // CHDATA read of channel c
                        cover (rst_n && hreg_addr_i == A_CHDATA + g && hreg_rdata_o != 32'h0);  // c_rd_chdata
                    end
                end
            end
        end
    endgenerate

    generate
        if (STANDALONE && !ENV_SEQ && !FREE_INIT) begin : g_cover_guard
            // Second guard: behaviour the sequencer/host never produce.
            always @(posedge clk) begin
                if (!$initstate) begin
                    // put to a full channel, ignored (new data differs)
                    c_g_putfull:  cover (rst_n && $past(rst_n && ch_put_i && !ch_get_i && !clr &&
                                                     v_idx && ch_wdata_i != d_idx));
                    // get from an empty channel, ignored
                    c_g_getempty: cover (rst_n && $past(rst_n && ch_get_i && !ch_put_i && !clr &&
                                                     !v_idx && d_idx != 32'h0));
                    // put and get together: empty -> put wins; valid -> get
                    c_g_both_empty: cover (rst_n && $past(rst_n && ch_put_i && ch_get_i && !clr &&
                                                       !v_idx && ch_wdata_i != d_idx));
                    c_g_both_full:  cover (rst_n && $past(rst_n && ch_put_i && ch_get_i && !clr &&
                                                       v_idx && ch_wdata_i != d_idx));
                    // other-address host write (incl. the CHSTAT alias 0x46)
                    // with a lane operation
                    c_g_hwr_put:       cover (rst_n && $past(rst_n && put_ok && hreg_we_i &&
                                                          hreg_addr_i == 7'h46 && hreg_wdata_i[3:0] == 4'hF));
                end
                // inputs active during reset (ignored)
                c_g_rst_ops: cover (!rst_n && ch_put_i && hreg_we_i && hreg_addr_i == A_CHSTAT);
            end
        end
    endgenerate

    generate
        if (STANDALONE && FREE_INIT) begin : g_cover_wrap
            // Timestamp wrap, reachable only from the free initial state.
            always @(posedge clk) begin
                // the mechanism: a first cycle out of reset, ts near the top
                c_w_init:  cover ($initstate && rst_n && ts_o == 32'hFFFF_FFFE);
                if (!$initstate) begin
                    // ts passes 0xFFFFFFFF -> 0
                    c_w_wrap:    cover (rst_n && $past(rst_n) && $past(ts_o) == 32'hFFFF_FFFF &&
                                        ts_o == 32'h0);
                    // TIME read across the wrap
                    c_w_wrap_rd: cover (rst_n && $past(rst_n && hreg_addr_i == A_TIME &&
                                                    hreg_rdata_o == 32'hFFFF_FFFF) &&
                                        hreg_addr_i == A_TIME && hreg_rdata_o == 32'h0);
                    // channel activity at the wrap edge: a put, then a host
                    // clear in the next cycle
                    c_w_wrap_ops: cover (rst_n && $past(rst_n && ts_o == 32'hFFFF_FFFF && put_ok) &&
                                         (clr_m & ch_valid_o) != 4'h0);
                    // counting goes on after the wrap
                    c_w_wrap_cont: cover (rst_n && f_wrap && ts_o == 32'd2);
                    // reset after a wrap brings ts back to 0
                    c_w_wrap_rst: cover (rst_n && !$past(rst_n) && f_seen_rst &&
                                         $past(ts_o, 2) == 32'hFFFF_FFFF);
                end
            end
        end
    endgenerate

endmodule
