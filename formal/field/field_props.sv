`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_field (operation unit, field execution).
//
//  Written from docs/blocks/field.md, lane.md (1, 2.1, 6, 7, 12.2), tick.md
//  (strobe guarantees), stuff.md, crc.md and docs/decisions.md ONLY. The RTL
//  was not read. Every property is about the block's ports.
//
//  Three layers, all on the ports:
//
//   1. EXACT MODEL (a_mdl_*). field_spec (field_model.sv) is an executable
//      model of field.md sections 2-8, driven from the input ports. Every
//      output is compared with it in every cycle (value outputs such as
//      tx_bit_o only while their strobe is 1). Covers required properties 1,
//      2, 4, 9, 10, 13, 14, 16 exactly and makes the shared-bus lemma a
//      consequence (the model reads park_*/cmd_data_i only under their
//      strobes); the lemma is also proved directly by two copies (field_fv).
//
//   2. END-TO-END, symbolic data (a_e2e_*). Independent of the model: small
//      port-level trackers count data launches, data captures and data
//      samples of the current field (which wire bit is a stuff bit is known
//      from stuff_next_i at its launch / sample), and ONE watched bit
//      position w_k (an arbitrary constant, $anyconst) is followed:
//        - the source bit that must go out as data bit w_k: it is sent as
//          the w_k-th data launch, fed to the CRC as the w_k-th data bit,
//          compared by EXPECT as the w_k-th sample, and found at its place
//          in SR after a DRIVE;
//        - the w_k-th sampled data bit: found at n-1-w_k (msb) / w_k (lsb)
//          in SR and in the result after SAMPLE / EXCHANGE / EXPECT.
//      Because w_k is arbitrary and the data (park_data_i, crc_value_i,
//      sr_o, rxb_i) is free, each statement covers every bit of every value.
//      The watched bits are tracked to their current SR position every
//      cycle (a_aux_*). The value group (block "Layer 2, value group") also
//      carries checker-internal lemmas (a_lem_*) that make it k-inductive.
//
//   3. PORT-LEVEL statements named after field.md 9 (a_p1_* .. a_p16_*) and
//      the brief: a_f_* promotion timing / gap-free, a_g_* result register
//      and tag rule, a_h_* abort/halt, a_i_* PIN/RAW, a_j_* idle, a_l_*
//      reset. They use the same trackers, not the model.
//
//  Proof structure (field.sby): task `prove` (PDR) proves layer 1, the bus
//  lemma and layer 3 (+ the layer-2 statements outside the value group);
//  task `prove_e2e` (k-induction) proves the value group with those ASSUMED
//  (macro CHK_A, MODEL_CHECK = 2). With STANDALONE = 0 / plain bmc, all
//  statements are asserted directly on the block.
//
//  SPEC DEFECT handled here (see e_chk): a BAUD sample still pending when a
//  DRIVE/EXCHANGE starts (only after a ct during an overrun HOLD of a BAUD
//  seek) is taken by an EXCHANGE as its first data bit and shifts every
//  launch position (shp); the end-to-end layer is not claimed for such a
//  field, the exact model checks the literal text there.
//
//  Environment (assumed with STANDALONE = 1, asserted with STANDALONE = 0).
//  Each is quoted from its source so it can be checked against the tick
//  and sequencer proofs:
//   env_rst      rst_n low in the first cycle (conventions.md).
//   Sequencer (lane.md 2.1 "Sequencer guarantees", sequencer.md 7.1):
//   env_s_halt   all strobes 0 while run_i = 0.
//   env_s_one    at most one of park_we_i, cmd_i != 0, res_ack_i, abort_i.
//   env_s_park   park_we_i never with park_full_o = 1.
//   env_s_ack    res_ack_i only with res_valid_o = 1.
//   env_s_cmd    cmd_i != 0 only with busy_o = park_full_o = 0
//                (busy_o = cu_v_o || bt_run_i).
//   env_s_op     park_op_i != 3 ("3 never sent").
//   Configuration (field.md 9):
//   env_cfg      cfg_mode_i, cfg_honour_i change only after a cycle with
//                cu_v_o = pk_v_o = 0 or run_i = 0.
//   Tick (field.md 9, tick.md):
//   env_t_ltct   lt_i and ct_i never in one cycle (field.md 9; tick.md 4.1
//                phases, 4.2 el/ec, 4.3; NOT an explicit tick.md 10 item).
//   env_t_ltreq  with BAUDT (BAUD, or PULSE with cu_tx_o) lt_i only with
//                launch_req_o (field.md 9; tick.md 4.1 "Stopped"/"Phase 1";
//                tick.md 10.3 states only the converse).
//   env_t_sp*    BAUDT without honour: spend_i rises exactly the cycle after
//                ct_i && cu_smp_o (with G), falls exactly the cycle after
//                st_i, abort or halt (or after an idle cycle: TICK write);
//                st_i only while spend_i; no ct_i while spend_i; otherwise
//                spend_i = 0 (field.md 9; tick.md 4.1 cte, 5, 10.8, 10.15).
//   env_t_stct   NOT in field.md 9 (needed): without BAUDT, st_i = ct_i
//                (tick.md 5 "!BAUDT: st as in section 4 (same cycle as
//                ct)"; 4.2 ct = st = ec, 4.3, 4.4; tick.md 10.9, 10.11,
//                10.12); BAUDT with honour, st_i = ct_i && cu_smp_o (tick.md
//                5; no tick.md 10 item states it). Without it an EXCHANGE
//                whose ct has no st keeps the bit and sends it twice.
//   env_t_alt    NOT in field.md 9: with BAUDT a ct_i comes only after an
//                lt_i since the previous ct_i, abort or halt (tick.md 4.1:
//                phase 0 and the honour wait are entered only from a launch;
//                tick.md 10.2 / 10.6 state it only for prescale 1).
//   Stuffing (field.md 9):
//   env_stf      stuff_next_i / stuff_bit_i change only after a cycle with
//                stf_valid_o or a stuffing clear (run_i = 0, or STUFFCLR /
//                SETM STUFF, which the sequencer issues only while busy_o =
//                park_full_o = 0); 0 in and right after reset (stuff.md).
//  Everything else is free: lt/ct outside the rules above, bt_run_i, bt_stop_i,
//  ot_done_i, d16_i, dedge_i, rxb_i, rxv_i, ts_i, crc_state_i, crc_value_i
//  (the CRC may change in any cycle: stronger than the real unit).
//
//  Overseer rulings 32-38 (docs/decisions.md) are applied (field_model.sv
//  header): seek only for SAMPLE/EXPECT; early wins over a start-cycle ct
//  in EXCHANGE; no data bit while tail = 1; hold clears at completion; a
//  held RAW SAMPLE completes on acceptance; abort keeps lstf/pstf; ack in
//  an abort cycle ignored, park + promotion sets pk_v, CRCCHK wins.
// ===========================================================================

// Layer-2/3 statements proved by task `prove` are re-used as assumptions
// by task `prove_e2e` (FV_STAGE_B): same environment, so the split is sound.
`ifdef FV_STAGE_B
  `define CHK_A assume
`else
  `define CHK_A assert
`endif

module field_props #(
    parameter STANDALONE  = 1,   // 1: env assumed + covers; 0: env asserted
    parameter MODEL_CHECK = 1    // layer 1: 0 off, 1 asserted, 2 assumed (stage B)
) (
    input wire        clk,
    input wire        rst_n,
    input wire        run_i,
    input wire        abort_i,
    input wire        park_we_i,
    input wire [1:0]  park_op_i,
    input wire [27:0] park_ctl_i,
    input wire [31:0] park_data_i,
    input wire        res_ack_i,
    input wire [1:0]  cmd_i,
    input wire [31:0] cmd_data_i,
    input wire [15:0] ts_i,
    input wire [1:0]  cfg_mode_i,
    input wire        lt_i,
    input wire        ct_i,
    input wire        st_i,
    input wire        spend_i,
    input wire        bt_run_i,
    input wire        bt_stop_i,
    input wire        ot_done_i,
    input wire        cfg_honour_i,
    input wire [15:0] d16_i,
    input wire        dedge_i,
    input wire        rxb_i,
    input wire        rxv_i,
    input wire        stuff_next_i,
    input wire        stuff_bit_i,
    input wire [31:0] crc_state_i,
    input wire [31:0] crc_value_i,
    input wire        park_full_o,
    input wire        res_valid_o,
    input wire [3:0]  res_dst_o,
    input wire [31:0] res_data_o,
    input wire [15:0] res_ts_o,
    input wire [3:0]  dst_busy_o,
    input wire        m_o,
    input wire [31:0] sr_o,
    input wire        cu_v_o,
    input wire        cu_fld_o,
    input wire        cu_tx_o,
    input wire        cu_smp_o,
    input wire        pk_v_o,
    input wire        launch_req_o,
    input wire        fld_next_o,
    input wire        fld_first_o,
    input wire        ot_start_o,
    input wire        ot_raw_o,
    input wire [15:0] ot_n_o,
    input wire        tx_go_o,
    input wire        tx_bit_o,
    input wire        tx_rel_o,
    input wire        raw_go_o,
    input wire        raw_lvl_o,
    input wire        pin_we_o,
    input wire [3:0]  pin_mask_o,
    input wire [3:0]  pin_val_o,
    input wire [3:0]  pin_oe_o,
    input wire        crc_en_o,
    input wire        crc_bit_o,
    input wire        stf_valid_o,
    input wire        stf_bit_o,
    input wire        cu_done_o,
    input wire        hold_o
);
`include "field_spec.svh"

    localparam [1:0] BAUD = 2'd0, EDGE = 2'd1, RECOVER = 2'd2, PULSE = 2'd3;
    localparam [1:0] DRIVE = 2'd0, SAMPLE = 2'd1, EXCH = 2'd2, EXPECT = 2'd3;
    localparam [1:0] OP_FIELD = 2'd0, OP_RAW = 2'd1, OP_PIN = 2'd2;
    localparam [3:0] NONE = 4'd15, HOSTQ = 4'd4, CAP = 4'd5, S_CRC = 4'd6,
                     S_SR = 4'd13, IMM = 4'd12, MEM = 4'd7;

    // a reset-free "previous cycle valid" flag
    logic f_pv;
    initial f_pv = 1'b0;
    always @(posedge clk) f_pv <= 1'b1;

    wire G       = run_i && !abort_i;
    wire park_ev = run_i && park_we_i && !abort_i;
    wire idle_sq = !cu_v_o && !pk_v_o && !bt_run_i;     // busy_o = park_full_o = 0
    wire strobes = tx_go_o || tx_rel_o || raw_go_o || pin_we_o || crc_en_o ||
                   stf_valid_o || ot_start_o;
    wire baudt   = (cfg_mode_i == BAUD) || (cfg_mode_i == PULSE && cu_tx_o);

    // =====================================================================
    // Environment
    // =====================================================================
    logic p_run, p_abort, p_G, p_idle, p_sv, p_spset, p_st, p_stfv, p_cuv0pk0;
    logic [2:0] p_cfg;
    logic p_stn, p_stb, p_rst, p_cudone;
    always @(posedge clk) begin
        p_run   <= run_i;
        p_abort <= abort_i;
        p_G     <= G;
        p_idle  <= idle_sq;
        p_spset <= G && ct_i && cu_smp_o && baudt && !cfg_honour_i;
        p_st    <= st_i;
        p_sv    <= spend_i;
        p_stfv  <= stf_valid_o;
        p_cuv0pk0 <= (!cu_v_o && !pk_v_o) || !run_i;
        p_cfg   <= {cfg_mode_i, cfg_honour_i};
        p_stn   <= stuff_next_i;
        p_stb   <= stuff_bit_i;
        p_rst   <= rst_n;
        p_cudone <= cu_done_o;
    end

    wire spend_exp = p_G && (p_spset || (p_sv && !p_st));
    wire e_s_halt  = run_i || (!park_we_i && !res_ack_i && !abort_i && cmd_i == 2'd0);
    wire e_s_one   = ({2'b0, park_we_i} + {2'b0, cmd_i != 2'd0} + {2'b0, res_ack_i} +
                      {2'b0, abort_i}) <= 3'd1;
    wire e_s_park  = !park_we_i || !park_full_o;
    wire e_s_ack   = !res_ack_i || res_valid_o;
    wire e_s_cmd   = (cmd_i == 2'd0) || (!cu_v_o && !bt_run_i && !pk_v_o);
    wire e_s_op    = !park_we_i || park_op_i != 2'd3;
    wire e_cfg     = !f_pv || !rst_n || !p_rst || ({cfg_mode_i, cfg_honour_i} == p_cfg) || p_cuv0pk0;
    wire e_t_ltct  = !(lt_i && ct_i);
    wire e_t_ltreq = !(baudt && lt_i) || launch_req_o;
    wire e_t_sp0   = !spend_i || (rst_n && (cfg_mode_i == BAUD || cfg_mode_i == PULSE) && !cfg_honour_i);
    wire e_t_sprst = rst_n || !spend_i;
    wire e_t_sprel = !f_pv || !rst_n || p_rst || !spend_i;     // 0 right after reset
    wire e_t_spup  = !f_pv || !rst_n || !p_rst || !spend_i || spend_exp;
    wire e_t_spdn  = !f_pv || !rst_n || !p_rst || spend_i || !spend_exp || p_idle;
    wire e_t_st    = !(baudt && !cfg_honour_i && st_i) || spend_i;
    wire e_t_ct    = !(spend_i && ct_i);
    // BAUD timer phase: a ct needs an lt since the last ct (tick.md 4.1:
    // phase 0 / honour wait are entered only from a launch tick)
    logic t_ph0;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) t_ph0 <= 1'b0;
        else        t_ph0 <= G && (lt_i ? 1'b1 : ((ct_i && baudt) ? 1'b0 : t_ph0));
    wire e_t_alt   = !(baudt && ct_i) || t_ph0;
    wire e_t_stct  = baudt ? (!cfg_honour_i ? 1'b1 : (st_i == (ct_i && cu_smp_o))) : (st_i == ct_i);
    wire e_stf_rst = rst_n || !stuff_next_i;
    wire e_stf_rel = !f_pv || !rst_n || p_rst || !stuff_next_i;
    wire e_stfchg  = !f_pv || !rst_n || !p_rst ||
                     ({stuff_next_i, stuff_bit_i} == {p_stn, p_stb}) ||
                     p_stfv || !p_run || p_idle;

    generate
        if (STANDALONE) begin : g_env_assume
            always @* begin
                if (!f_pv) env_rst: assume (!rst_n);
                env_s_halt:  assume (e_s_halt);
                env_s_one:   assume (e_s_one);
                env_s_park:  assume (e_s_park);
                env_s_ack:   assume (e_s_ack);
                env_s_cmd:   assume (e_s_cmd);
                env_s_op:    assume (e_s_op);
                env_cfg:     assume (e_cfg);
                env_t_ltct:  assume (e_t_ltct);
                env_t_ltreq: assume (e_t_ltreq);
                env_t_sp0:   assume (e_t_sp0);
                env_t_sprst: assume (e_t_sprst);
                env_t_sprel: assume (e_t_sprel);
                env_t_spup:  assume (e_t_spup);
                env_t_spdn:  assume (e_t_spdn);
                env_t_st:    assume (e_t_st);
                env_t_ct:    assume (e_t_ct);
                env_t_stct:  assume (e_t_stct);
                env_t_alt:   assume (e_t_alt);
                env_stf_rst: assume (e_stf_rst);
                env_stf_rel: assume (e_stf_rel);
                env_stf:     assume (e_stfchg);
            end
        end else begin : g_env_assert
            always @* begin
                if (!f_pv) env_rst: assert (!rst_n);
                env_s_halt:  assert (e_s_halt);
                env_s_one:   assert (e_s_one);
                env_s_park:  assert (e_s_park);
                env_s_ack:   assert (e_s_ack);
                env_s_cmd:   assert (e_s_cmd);
                env_s_op:    assert (e_s_op);
                env_cfg:     assert (e_cfg);
                env_t_ltct:  assert (e_t_ltct);
                env_t_ltreq: assert (e_t_ltreq);
                env_t_sp0:   assert (e_t_sp0);
                env_t_sprst: assert (e_t_sprst);
                env_t_sprel: assert (e_t_sprel);
                env_t_spup:  assert (e_t_spup);
                env_t_spdn:  assert (e_t_spdn);
                env_t_st:    assert (e_t_st);
                env_t_ct:    assert (e_t_ct);
                env_t_stct:  assert (e_t_stct);
                env_t_alt:   assert (e_t_alt);
                env_stf_rst: assert (e_stf_rst);
                env_stf_rel: assert (e_stf_rel);
                env_stf:     assert (e_stfchg);
            end
        end
    endgenerate

    // =====================================================================
    // Layer 1: exact model, every output every cycle
    // =====================================================================
    wire        m_park_full, m_res_valid, m_m, m_cu_v, m_cu_fld, m_cu_tx, m_cu_smp,
                m_pk_v, m_launch_req, m_fld_next, m_fld_first, m_ot_start, m_ot_raw,
                m_tx_go, m_tx_bit, m_tx_rel, m_raw_go, m_raw_lvl, m_pin_we,
                m_crc_en, m_crc_bit, m_stf_valid, m_stf_bit, m_cu_done, m_hold,
                m_prom, m_tail, m_lstf, m_enter_hold;
    wire [3:0]  m_res_dst, m_dst_busy, m_pin_mask, m_pin_val, m_pin_oe;
    wire [31:0] m_res_data, m_sr;
    wire [15:0] m_res_ts, m_ot_n;
    wire [5:0]  m_bc, m_nl;
    wire [1:0]  m_pk_op, m_cu_op;
    wire [27:0] m_pk_ctl, m_cu_ctl;
    wire [31:0] m_pk_data;
    wire        m_first, m_eff, m_lnch, m_pstf, m_rel, m_mm;

    field_spec u_spec (
        .clk(clk), .rst_n(rst_n), .run_i(run_i), .abort_i(abort_i),
        .park_we_i(park_we_i), .park_op_i(park_op_i), .park_ctl_i(park_ctl_i),
        .park_data_i(park_data_i), .res_ack_i(res_ack_i), .cmd_i(cmd_i),
        .cmd_data_i(cmd_data_i), .ts_i(ts_i), .cfg_mode_i(cfg_mode_i),
        .lt_i(lt_i), .ct_i(ct_i), .st_i(st_i), .spend_i(spend_i),
        .bt_run_i(bt_run_i), .bt_stop_i(bt_stop_i), .ot_done_i(ot_done_i),
        .cfg_honour_i(cfg_honour_i), .d16_i(d16_i), .dedge_i(dedge_i),
        .rxb_i(rxb_i), .rxv_i(rxv_i), .stuff_next_i(stuff_next_i),
        .stuff_bit_i(stuff_bit_i), .crc_state_i(crc_state_i), .crc_value_i(crc_value_i),
        .m_park_full(m_park_full), .m_res_valid(m_res_valid), .m_res_dst(m_res_dst),
        .m_res_data(m_res_data), .m_res_ts(m_res_ts), .m_dst_busy(m_dst_busy),
        .m_m(m_m), .m_sr(m_sr), .m_cu_v(m_cu_v), .m_cu_fld(m_cu_fld), .m_cu_tx(m_cu_tx),
        .m_cu_smp(m_cu_smp), .m_pk_v(m_pk_v), .m_launch_req(m_launch_req),
        .m_fld_next(m_fld_next), .m_fld_first(m_fld_first), .m_ot_start(m_ot_start),
        .m_ot_raw(m_ot_raw), .m_ot_n(m_ot_n), .m_tx_go(m_tx_go), .m_tx_bit(m_tx_bit),
        .m_tx_rel(m_tx_rel), .m_raw_go(m_raw_go), .m_raw_lvl(m_raw_lvl),
        .m_pin_we(m_pin_we), .m_pin_mask(m_pin_mask), .m_pin_val(m_pin_val),
        .m_pin_oe(m_pin_oe), .m_crc_en(m_crc_en), .m_crc_bit(m_crc_bit),
        .m_stf_valid(m_stf_valid), .m_stf_bit(m_stf_bit), .m_cu_done(m_cu_done),
        .m_hold(m_hold), .m_prom(m_prom), .m_bc_o(m_bc), .m_tail_o(m_tail),
        .m_lstf_o(m_lstf), .m_enter_hold(m_enter_hold),
        .m_pk_op_o(m_pk_op), .m_pk_ctl_o(m_pk_ctl), .m_pk_data_o(m_pk_data),
        .m_cu_op_o(m_cu_op), .m_cu_ctl_o(m_cu_ctl), .m_first_o(m_first), .m_eff_o(m_eff),
        .m_lnch_o(m_lnch), .m_pstf_o(m_pstf), .m_rel_o(m_rel), .m_mm_o(m_mm), .m_nl_o(m_nl)
    );

    generate
        if (MODEL_CHECK == 1) begin : g_mdl
            always @* begin
                a_mdl_park_full:  assert (park_full_o  == m_park_full);
                a_mdl_pk_v:       assert (pk_v_o       == m_pk_v);
                a_mdl_cu_v:       assert (cu_v_o       == m_cu_v);
                a_mdl_cu_fld:     assert (cu_fld_o     == m_cu_fld);
                a_mdl_cu_tx:      assert (cu_tx_o      == m_cu_tx);
                a_mdl_cu_smp:     assert (cu_smp_o     == m_cu_smp);
                a_mdl_res_valid:  assert (res_valid_o  == m_res_valid);
                a_mdl_res_dst:    assert (res_dst_o    == m_res_dst);
                a_mdl_res_data:   assert (res_data_o   == m_res_data);
                a_mdl_res_ts:     assert (res_ts_o     == m_res_ts);
                a_mdl_dst_busy:   assert (dst_busy_o   == m_dst_busy);
                a_mdl_m:          assert (m_o          == m_m);
                a_mdl_sr:         assert (sr_o         == m_sr);
                a_mdl_launch_req: assert (launch_req_o == m_launch_req);
                a_mdl_fld_next:   assert (fld_next_o   == m_fld_next);
                a_mdl_fld_first:  assert (fld_first_o  == m_fld_first);
                a_mdl_ot_start:   assert (ot_start_o   == m_ot_start);
                a_mdl_tx_go:      assert (tx_go_o      == m_tx_go);
                a_mdl_tx_rel:     assert (tx_rel_o     == m_tx_rel);
                a_mdl_raw_go:     assert (raw_go_o     == m_raw_go);
                a_mdl_pin_we:     assert (pin_we_o     == m_pin_we);
                a_mdl_crc_en:     assert (crc_en_o     == m_crc_en);
                a_mdl_stf_valid:  assert (stf_valid_o  == m_stf_valid);
                a_mdl_cu_done:    assert (cu_done_o    == m_cu_done);
                a_mdl_hold:       assert (hold_o       == m_hold);
                if (m_ot_start) begin
                    a_mdl_ot_raw: assert (ot_raw_o == m_ot_raw);
                    a_mdl_ot_n:   assert (ot_n_o   == m_ot_n);
                end
                if (m_tx_go)     a_mdl_tx_bit:  assert (tx_bit_o  == m_tx_bit);
                if (m_raw_go)    a_mdl_raw_lvl: assert (raw_lvl_o == m_raw_lvl);
                if (m_pin_we) begin
                    a_mdl_pin_mask: assert (pin_mask_o == m_pin_mask);
                    a_mdl_pin_val:  assert (pin_val_o  == m_pin_val);
                    a_mdl_pin_oe:   assert (pin_oe_o   == m_pin_oe);
                end
                if (m_crc_en)    a_mdl_crc_bit: assert (crc_bit_o == m_crc_bit);
                if (m_stf_valid) a_mdl_stf_bit: assert (stf_bit_o == m_stf_bit);
            end
        end
        if (MODEL_CHECK == 2) begin : g_mdl_assume
            always @* begin
                u_mdl_park_full:  assume (park_full_o  == m_park_full);
                u_mdl_pk_v:       assume (pk_v_o       == m_pk_v);
                u_mdl_cu_v:       assume (cu_v_o       == m_cu_v);
                u_mdl_cu_fld:     assume (cu_fld_o     == m_cu_fld);
                u_mdl_cu_tx:      assume (cu_tx_o      == m_cu_tx);
                u_mdl_cu_smp:     assume (cu_smp_o     == m_cu_smp);
                u_mdl_res_valid:  assume (res_valid_o  == m_res_valid);
                u_mdl_res_dst:    assume (res_dst_o    == m_res_dst);
                u_mdl_res_data:   assume (res_data_o   == m_res_data);
                u_mdl_res_ts:     assume (res_ts_o     == m_res_ts);
                u_mdl_dst_busy:   assume (dst_busy_o   == m_dst_busy);
                u_mdl_m:          assume (m_o          == m_m);
                u_mdl_sr:         assume (sr_o         == m_sr);
                u_mdl_launch_req: assume (launch_req_o == m_launch_req);
                u_mdl_fld_next:   assume (fld_next_o   == m_fld_next);
                u_mdl_fld_first:  assume (fld_first_o  == m_fld_first);
                u_mdl_ot_start:   assume (ot_start_o   == m_ot_start);
                u_mdl_tx_go:      assume (tx_go_o      == m_tx_go);
                u_mdl_tx_rel:     assume (tx_rel_o     == m_tx_rel);
                u_mdl_raw_go:     assume (raw_go_o     == m_raw_go);
                u_mdl_pin_we:     assume (pin_we_o     == m_pin_we);
                u_mdl_crc_en:     assume (crc_en_o     == m_crc_en);
                u_mdl_stf_valid:  assume (stf_valid_o  == m_stf_valid);
                u_mdl_cu_done:    assume (cu_done_o    == m_cu_done);
                u_mdl_hold:       assume (hold_o       == m_hold);
                if (m_ot_start) begin
                    u_mdl_ot_raw: assume (ot_raw_o == m_ot_raw);
                    u_mdl_ot_n:   assume (ot_n_o   == m_ot_n);
                end
                if (m_tx_go)     u_mdl_tx_bit:  assume (tx_bit_o  == m_tx_bit);
                if (m_raw_go)    u_mdl_raw_lvl: assume (raw_lvl_o == m_raw_lvl);
                if (m_pin_we) begin
                    u_mdl_pin_mask: assume (pin_mask_o == m_pin_mask);
                    u_mdl_pin_val:  assume (pin_val_o  == m_pin_val);
                    u_mdl_pin_oe:   assume (pin_oe_o   == m_pin_oe);
                end
                if (m_crc_en)    u_mdl_crc_bit: assume (crc_bit_o == m_crc_bit);
                if (m_stf_valid) u_mdl_stf_bit: assume (stf_bit_o == m_stf_bit);
            end
        end
    endgenerate

    // =====================================================================
    // Port-level shadows (field.md 9: "decodes are shadowed by the checker
    // from the park buses and promotions")
    // =====================================================================
    logic [1:0]  sp_op, sc_op;
    logic [27:0] sp_ctl, sc_ctl;
    logic [31:0] sp_data;
    wire  x_srcdep = f_srcdep(sp_op, sp_ctl);
    // PROMOTE(p) on ports, with cu_done_o as the completion (property 1)
    wire  x_prom   = run_i && !abort_i && pk_v_o && (!cu_v_o || cu_done_o) &&
                     !(x_srcdep && cu_v_o);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sp_op <= 2'd0; sp_ctl <= 28'h0; sp_data <= 32'h0;
            sc_op <= 2'd0; sc_ctl <= 28'h0;
        end else begin
            if (park_ev) begin sp_op <= park_op_i; sp_ctl <= park_ctl_i; sp_data <= park_data_i; end
            if (x_prom)  begin sc_op <= sp_op;     sc_ctl <= sp_ctl; end
        end
    end

    // decodes of the current op (shadow)
    wire [1:0]  e_kind  = sc_ctl[27:26];
    wire [5:0]  e_n     = f_n(sc_ctl);
    wire [4:0]  e_np    = f_np(sc_ctl);
    wire        e_lsb   = sc_ctl[20];
    wire        e_crcf  = sc_ctl[19];
    wire        e_stf   = sc_ctl[18];
    wire        e_early = sc_ctl[9];
    wire        e_seek  = f_seek(sc_ctl);
    wire [7:0]  e_tag   = f_tag(sc_ctl);
    wire [3:0]  e_dst   = f_dst(sc_op, sc_ctl);
    wire        e_TX    = f_tx(sc_ctl);
    wire        e_RX    = (e_kind == SAMPLE) || (e_kind == EXPECT);
    wire        e_bt    = f_bt(cfg_mode_i, sc_ctl);
    wire        e_bnh   = e_bt && !cfg_honour_i;
    wire        e_stx   = e_stf && e_TX;
    wire        e_srx   = e_stf && e_RX && !e_bt;
    wire [5:0]  e_opos0 = e_lsb ? 6'd0 : (e_n - 6'd1);
    wire        e_isF   = cu_v_o && sc_op == OP_FIELD;
    wire        e_isPIN = cu_v_o && sc_op == OP_PIN;
    wire        e_isRD  = cu_v_o && sc_op == OP_RAW &&  sc_ctl[27];
    wire        e_isRS  = cu_v_o && sc_op == OP_RAW && !sc_ctl[27];
    wire        e_needs = (e_isF || e_isRS) && e_dst != NONE;
    wire        e_act   = G && e_isF && !hold_o;
    wire        e_acc   = !res_valid_o || res_ack_i;
    wire        e_tagd  = (e_dst == HOSTQ || e_dst == CAP) && e_tag != 8'h0;
    wire        e_bmode = (cfg_mode_i == BAUD) || (cfg_mode_i == PULSE);

    // decodes of the parked op, used at promotion
    wire [5:0]  p_n     = f_n(sp_ctl);
    wire        p_lsb   = sp_ctl[20];
    wire [3:0]  p_src   = sp_ctl[17:14];
    wire [31:0] p_v     = (p_src == S_CRC) ? crc_value_i : (p_src == S_SR) ? sr_o : sp_data;
    wire        p_ldsr  = (sp_op == OP_FIELD) || (sp_op == OP_RAW && sp_ctl[27]);
    wire [31:0] w_load  = f_load(sp_op, sp_ctl, sp_data, crc_value_i, sr_o, sr_o);

    // =====================================================================
    // Field trackers (port-level)
    // =====================================================================
    logic       e_first, e_ol, e_ostf, e_obit, e_pstf, e_pbit, e_rel, e_mm, e_pw, e_inh, e_lnchd;
    logic [5:0] e_dl, e_ds, e_nl, e_tot;

    wire e_EL     = e_first && e_early && !e_bt;
    wire e_stuffL = e_stx && stuff_next_i;
    wire e_left   = (e_dl < e_n) || (e_stx && stuff_next_i && e_dl == e_n);
    wire e_LE     = e_act && e_TX && (lt_i || e_EL) && !e_ol && e_left;
    wire e_shp    = spend_i && !e_pstf;
    wire [5:0] e_opos = e_lsb ? (e_shp ? 6'd1 : 6'd0) : (e_shp ? (e_n - 6'd2) : (e_n - 6'd1));
    wire e_sv     = st_i && (rxv_i || e_TX);
    // data capture of a DRIVE, data / discarded sample of an EXCHANGE
    wire e_dcap   = e_act && e_kind == DRIVE && ct_i && e_ol;
    wire e_dd     = e_dcap && !e_ostf;
    wire e_xct    = e_act && e_kind == EXCH && ct_i;
    wire e_xs_b   = e_act && e_kind == EXCH && e_bnh && e_sv;
    wire e_xs_o   = e_act && e_kind == EXCH && !e_bnh && e_sv && ct_i && e_ol;
    wire e_xsmp   = e_xs_b || e_xs_o;
    wire e_xd     = e_xs_b ? (e_xs_b && !e_pstf) : (e_xs_o && !e_ostf);
    wire e_xbit   = e_xs_b ? e_pbit : e_obit;           // the driven bit of this sample
    // SAMPLE / EXPECT
    wire e_tail   = e_srx && !e_seek && e_ds == e_n;    // all data bits in, waiting
    wire e_rstuff = e_act && e_RX && e_sv && e_srx && stuff_next_i;
    wire e_rd     = e_act && e_RX && e_sv && !(e_srx && stuff_next_i) && !e_tail &&
                    (e_seek || e_ds < e_n);
    wire e_data   = e_dd || e_xd || e_rd;
    wire [5:0] e_ds1 = (e_ds == 6'd63) ? 6'd63 : e_ds + 6'd1;
    wire [15:0] e_win = f_win(sr_o[15:0], rxb_i, e_np, e_lsb);
    wire e_match  = (e_ds1 >= {1'b0, e_np}) &&
                    ((e_win & f_mask16(e_np)) == (sr_o[31:16] & f_mask16(e_np)));
    wire e_ol_nx  = tx_go_o ? 1'b1 : ((e_dcap || e_xct) ? 1'b0 : e_ol);
    wire [5:0] e_ds_af = e_data ? e_ds1 : e_ds;
    wire e_bmis   = e_rd && !e_seek && e_kind == EXPECT && (rxb_i != sr_o[e_opos0[4:0]]);
    wire e_mm_nx  = e_mm || e_bmis;
    wire e_gback  = e_act && e_RX && e_bt && st_i && !rxv_i;
    // SPEC DEFECT corner (report): a BAUD sample still pending when a
    // transmitting field starts. It can only come from a ct during an
    // overrun HOLD of a BAUD seek that launched one bit too many (lane.md
    // I20 + 7.5). field.md clears pstf at promotion, so an EXCHANGE takes the
    // stale sample as its first data bit and any DRIVE/EXCHANGE launch uses
    // shp = 1 (wrong bit). The end-to-end layer is not stated for such a
    // field (e_chk = 0); the exact model still checks the literal spec text.
    wire e_chk    = !(e_TX && (e_inh || (e_first && spend_i)));
    wire e_ltc    = e_act && e_RX && e_bt && lt_i;

    // completion of a FIELD by the spec (6.1), from the trackers
    logic e_fin;
    always_comb begin
        e_fin = 1'b0;
        if (e_act) begin
            case (e_kind)
                DRIVE: e_fin = e_dcap && (e_ds_af == e_n) && !e_stuffL;
                EXCH:  e_fin = e_xsmp && (e_ds_af == e_n) && !e_stuffL && !e_ol_nx;
                default:
                    if (e_seek) e_fin = e_rd && e_match;
                    else        e_fin = (e_rd && e_ds1 == e_n && !e_srx) ||
                                        (e_tail && (!stuff_next_i || e_rstuff));
            endcase
        end
    end

    // PIN / RAW effect cycles (7.1, 7.2)
    wire e_pe = G && e_isPIN && !e_pw &&
                (e_bmode ? (!bt_run_i || bt_stop_i) : (sc_ctl[15] ? lt_i : 1'b1));
    wire e_re = G && e_isRD && !e_pw && (e_bmode ? (!bt_run_i || bt_stop_i) : 1'b1);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            e_first <= 1'b0; e_ol <= 1'b0; e_ostf <= 1'b0; e_obit <= 1'b0;
            e_pstf <= 1'b0; e_pbit <= 1'b0; e_rel <= 1'b0; e_mm <= 1'b0; e_pw <= 1'b0;
            e_dl <= 6'd0; e_ds <= 6'd0; e_nl <= 6'd0; e_tot <= 6'd0; e_inh <= 1'b0; e_lnchd <= 1'b0;
        end else begin
            e_first <= x_prom;
            if (x_prom)                 e_lnchd <= 1'b0;
            else if (tx_go_o)           e_lnchd <= 1'b1;
            if (x_prom)                 e_inh <= 1'b0;
            else if (e_first && spend_i) e_inh <= 1'b1;
            if (x_prom || !G) begin
                e_ol <= 1'b0; e_ostf <= 1'b0; e_pstf <= 1'b0; e_rel <= 1'b0; e_pw <= 1'b0;
                if (x_prom) begin
                    e_dl <= 6'd0; e_ds <= 6'd0; e_nl <= 6'd0; e_mm <= 1'b0; e_tot <= 6'd0;
                end
            end else begin
                if (tx_go_o) begin
                    e_ol <= 1'b1; e_ostf <= e_stuffL; e_obit <= tx_bit_o;
                end else if (e_dcap || e_xct) begin
                    e_ol <= 1'b0;
                end
                if (e_xct && e_bnh) begin e_pstf <= e_ostf || !e_ol; e_pbit <= e_obit; end
                if (tx_rel_o) e_rel <= 1'b1;
                if (pin_we_o || raw_go_o) e_pw <= 1'b1;
                if (tx_go_o && !e_stuffL) e_dl <= e_dl + 6'd1;
                e_ds <= e_ds_af;
                e_mm <= e_mm_nx;
                if (e_ltc && !e_gback)      e_nl <= e_nl + 6'd1;
                else if (e_gback && !e_ltc) e_nl <= (e_nl == 6'd0) ? 6'd0 : e_nl - 6'd1;
                if (e_act && e_RX && e_sv && e_tot != 6'd63) e_tot <= e_tot + 6'd1;
            end
        end
    end

    // =====================================================================
    // Watched bit (symbolic position)
    // =====================================================================
    (* anyconst *) wire [4:0] w_k;            // watched data-bit index (wire order)
    wire [5:0] w_k6 = {1'b0, w_k};
    // position of data bit w_k of the parked op in its source: lsb ? k : n-1-k
    wire [5:0] w_jp = p_lsb ? w_k6 : (p_n - 6'd1 - w_k6);
    wire [5:0] w_jc = e_lsb ? w_k6 : (e_n - 6'd1 - w_k6);     // current op
    wire       w_in = w_k6 < e_n;                              // k is a bit of the field

    logic       ws_on, ws_val, wr_on, wr_val;
    logic [4:0] ws_pos, wr_pos;
    // movement of a bit at position p by one ROT / INS (5.4)
    wire [4:0] e_nm1 = e_n[4:0] - 5'd1;
    function automatic [4:0] f_up(input [4:0] p, input [4:0] nm1, input lsb);
        if (!lsb) f_up = (p == nm1) ? 5'd0 : p + 5'd1;
        else      f_up = (p == 5'd0) ? nm1 : p - 5'd1;
    endfunction
    wire e_ins = e_xd || (e_rd && !e_seek);       // an INS of a sampled bit

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ws_on <= 1'b0; ws_val <= 1'b0; ws_pos <= 5'd0;
            wr_on <= 1'b0; wr_val <= 1'b0; wr_pos <= 5'd0;
        end else if (x_prom) begin
            ws_on  <= (sp_op == OP_FIELD) && !f_seek(sp_ctl) && (w_k6 < p_n);
            ws_val <= p_v[w_jp[4:0]];
            ws_pos <= w_jp[4:0];
            wr_on  <= 1'b0;
        end else begin
            if (e_dd && ws_on)  ws_pos <= f_up(ws_pos, e_nm1, e_lsb);         // ROT
            if (e_ins && ws_on) begin
                if (ws_pos == e_opos0[4:0]) ws_on <= 1'b0;                     // shifted out
                else                        ws_pos <= f_up(ws_pos, e_nm1, e_lsb);
            end
            if (e_ins && e_ds == w_k6) begin
                wr_on  <= 1'b1; wr_val <= rxb_i;
                wr_pos <= e_lsb ? e_nm1 : 5'd0;
            end else if (e_ins && wr_on) begin
                wr_pos <= f_up(wr_pos, e_nm1, e_lsb);
            end
        end
    end

    // =====================================================================
    // Completion bookkeeping: what must be visible in the next cycle
    // =====================================================================
    wire e_cdone = cu_done_o && !hold_o;                // first-time completion
    wire e_ehold = G && cu_v_o && !hold_o && !cu_done_o && e_needs && !e_acc &&
                   (e_isRS ? dedge_i : e_fin);
    logic q_fin, q_hfin, q_ldsr, q_ld, q_kind_ok, q_seek, q_lsb, q_RS, q_tagd, q_wsv, q_wrv,
          q_wson, q_wron, q_exp, q_mm, q_in, q_crcchk, q_crcm, q_drv, q_fld;
    logic [5:0]  q_n, q_wk, q_jc;
    logic [7:0]  q_tag;
    logic [3:0]  q_dst;
    logic [15:0] q_win, q_ts, q_d16;
    logic [31:0] q_sr;
    always @(posedge clk) begin
        q_fin   <= rst_n && (e_cdone || e_ehold) && (e_isF || e_isRS) && e_chk;
        q_hfin  <= rst_n && cu_done_o && hold_o;
        q_ldsr  <= x_prom && p_ldsr;                      // SR overwritten by a load
        q_ld    <= cu_done_o && e_needs;
        q_seek  <= e_seek;
        q_lsb   <= e_lsb;
        q_RS    <= e_isRS;
        q_fld   <= e_isF;
        q_drv   <= e_kind == DRIVE;
        q_tagd  <= e_tagd;
        q_tag   <= e_tag;
        q_dst   <= e_dst;
        q_n     <= e_n;
        q_wk    <= w_k6;
        q_jc    <= w_jc;
        q_in    <= w_in;
        q_wsv   <= ws_val;
        q_wson  <= ws_on;
        q_wrv   <= (e_ins && e_ds == w_k6) ? rxb_i : wr_val;
        q_wron  <= wr_on || (e_ins && e_ds == w_k6);
        q_win   <= e_win;
        q_ts    <= ts_i;
        q_d16   <= d16_i;
        q_sr    <= sr_o;
        q_exp   <= rst_n && cu_done_o && e_isF && e_kind == EXPECT;
        q_mm    <= hold_o ? e_mm : e_mm_nx;
        q_crcchk <= rst_n && run_i && cmd_i == 2'd2;
        q_crcm  <= crc_state_i == cmd_data_i;
    end

    // expected V of a first-time completion, bit by bit where it is known
    wire [31:0] q_mask = f_mask(q_n);
    // SR / result position of the watched bit: DRIVE: source position
    // pos(k); receiving kinds: n-1-k (msb) or k (lsb)
    wire [4:0]  q_rpos = q_drv ? q_jc[4:0] : (q_lsb ? q_wk[4:0] : (q_n[4:0] - 5'd1 - q_wk[4:0]));

`ifndef FV_NO_E2E
    // =====================================================================
    // Assertions: end-to-end (symbolic watched bit) and port level
    // =====================================================================
    always @* begin
        if (rst_n) begin
            if (e_chk) begin
                // ---------------- (a) DRIVE / EXCHANGE launches -----------------
                // a launch happens exactly at LE (5.1): launch absorption (p9),
                // trailing stuff bit (p7), early only in EDGE/RECOVER (ruling 6)
                a_p9_launch_exact: `CHK_A (tx_go_o == e_LE);
                if (tx_go_o && e_stuffL) begin
                    // stuff bit inserted exactly where the stuffing unit demands
                    a_e2e_stuff_bit: `CHK_A (tx_bit_o == stuff_bit_i);
                end
                if (tx_go_o && !e_stuffL) begin
                    a_e2e_data_in_range: `CHK_A (e_dl < e_n);
                end
                // ---------------- (e) CRC feed --------------------------------
                a_p8_crc_en: `CHK_A (crc_en_o == (e_crcf && e_data));
                if (crc_en_o && e_dd) a_p8_crc_drive:  `CHK_A (crc_bit_o == e_obit);
                if (crc_en_o && e_rd) a_p8_crc_sample: `CHK_A (crc_bit_o == rxb_i);
                // ---------------- (p7) stuffing unit feed ---------------------
                a_p7_stf_valid: `CHK_A (stf_valid_o == ((tx_go_o && e_stx) || e_rstuff ||
                                                         (e_rd && e_srx)));
                if (stf_valid_o) a_p7_stf_bit: `CHK_A (stf_bit_o == (tx_go_o ? tx_bit_o : rxb_i));
                // ---------------- (d) EXPECT compares source bit pos(k) --------
                // ---------------- watched bits stay where they belong ----------
                if (e_isF && ws_on) a_aux_ws_sr: `CHK_A (sr_o[ws_pos] == ws_val);
                if (e_isF && !e_seek) a_aux_sr_mask: `CHK_A ((sr_o & ~f_mask(e_n)) == 32'h0);
                // ---------------- (p2) completion after exactly n data bits ----
            end
            // ---------------- (p10) release -------------------------------
            a_p10_release: `CHK_A (tx_rel_o == (e_act && e_RX && !e_rel &&
                (lt_i || (e_first && ((e_early && !e_bt) || cfg_mode_i == RECOVER ||
                                      cfg_mode_i == PULSE)))));
            // ---------------- (p12) hold ---------------------------------
            if (hold_o) begin
                a_p12_hold_cur:  `CHK_A (cu_v_o && (e_isF || e_isRS) && e_dst != NONE);
                a_p12_hold_quiet: `CHK_A (!tx_go_o && !tx_rel_o && !crc_en_o && !stf_valid_o &&
                                          !raw_go_o && !pin_we_o && !ot_start_o);
                a_p12_hold_done: `CHK_A (cu_done_o == (G && e_acc));
                a_p12_hold_lreq: `CHK_A (!launch_req_o);
            end
            // ---------------- (i) PIN / RAW --------------------------------
            a_p13_pin_we:  `CHK_A (pin_we_o == e_pe);
            a_i_raw_go:    `CHK_A (raw_go_o == e_re);
            a_i_ot_start:  `CHK_A (ot_start_o == ((pin_we_o && sc_ctl[11:0] != 12'd0) || raw_go_o));
            if (pin_we_o) begin
                a_p13_pin_bits: `CHK_A (pin_mask_o == sc_ctl[27:24] && pin_val_o == sc_ctl[23:20] &&
                                        pin_oe_o == sc_ctl[19:16]);
            end
            if (ot_start_o && pin_we_o) a_p13_ot_pin: `CHK_A (!ot_raw_o && ot_n_o == {4'h0, sc_ctl[11:0]});
            if (raw_go_o) begin
                a_i_raw_lvl: `CHK_A (raw_lvl_o == sc_ctl[26]);
                a_i_ot_raw:  `CHK_A (ot_raw_o && ot_n_o == sr_o[15:0]);
            end
            if (e_isPIN) a_p13_pin_done: `CHK_A (cu_done_o == (G &&
                ((pin_we_o && (sc_ctl[11:0] == 12'd0 || ot_done_i)) || (e_pw && ot_done_i))));
            if (e_isRD)  a_i_rawd_done:  `CHK_A (cu_done_o == (G &&
                ((raw_go_o && ot_done_i) || (e_pw && ot_done_i))));
            if (e_isRS && !hold_o) a_i_raws_done: `CHK_A (cu_done_o == (G && dedge_i && (!e_needs || e_acc)));
            // ---------------- strobes belong to their op -------------------
            if (tx_go_o || tx_rel_o || crc_en_o || stf_valid_o) a_own_field: `CHK_A (e_isF);
            if (tx_go_o)  a_own_tx:  `CHK_A (e_TX);
            if (tx_rel_o) a_own_rx:  `CHK_A (e_RX);
            if (pin_we_o) a_own_pin: `CHK_A (e_isPIN);
            if (raw_go_o) a_own_raw: `CHK_A (e_isRD);
            // ---------------- (h) abort / halt cycle: no progress ----------
            if (!G) begin
                a_h_no_strobe: `CHK_A (!strobes && !cu_done_o);
            end
            // ---------------- (j) idle: nothing starts, no result ---------
            if (!cu_v_o) a_j_idle_quiet: `CHK_A (!strobes && !cu_done_o);
            // ---------------- (p16) status outputs -------------------------
            a_p16_cu_fld:  `CHK_A (cu_fld_o == e_isF);
            a_p16_cu_tx:   `CHK_A (cu_tx_o  == (e_isF && e_TX));
            a_p16_cu_smp:  `CHK_A (cu_smp_o == (e_isF && e_kind != DRIVE));
            a_p16_first:   `CHK_A (fld_first_o == (e_first && cu_v_o && sc_op == OP_FIELD));
            a_p16_full:    `CHK_A (park_full_o == pk_v_o);
            a_p16_dst_busy: `CHK_A (dst_busy_o ==
                ((cu_v_o ? f_rbit(f_dst(sc_op, sc_ctl)) : 4'h0) |
                 (pk_v_o ? f_rbit(f_dst(sp_op, sp_ctl)) : 4'h0) |
                 (res_valid_o ? f_rbit(res_dst_o) : 4'h0)));
            a_p16_fld_next: `CHK_A (fld_next_o == (pk_v_o && sp_op == OP_FIELD &&
                f_bt(cfg_mode_i, sp_ctl) && (!cu_v_o || (sc_op == OP_FIELD && !launch_req_o))));
            if (launch_req_o) a_p16_lreq_imp: `CHK_A (e_isF && e_bt && !hold_o);
            if (e_isF && e_bt && !hold_o && e_TX)
                a_p16_lreq_tx: `CHK_A (launch_req_o == (!e_ol && e_left));
            if (e_isF && e_bt && !hold_o && e_RX)
                a_p16_lreq_rx: `CHK_A (launch_req_o == (e_seek || e_nl < e_n));
            // (f) a BAUD-timed field can launch in its start cycle (gap-free)
            if (fld_first_o && e_bt) a_f_lreq_first: `CHK_A (launch_req_o);
            // ---------------- (g) result register ---------------------------
            if (res_valid_o) a_g_res_dst_ok: `CHK_A (res_dst_o != NONE);
        end else begin
            // ---------------- (l) reset values -----------------------------
            a_l_rst_state: `CHK_A (!park_full_o && !pk_v_o && !cu_v_o && !res_valid_o &&
                                   res_dst_o == 4'd0 && res_data_o == 32'h0 && res_ts_o == 16'h0 &&
                                   dst_busy_o == 4'd0 && !m_o && sr_o == 32'h0 && !hold_o &&
                                   !fld_first_o && !cu_fld_o && !cu_tx_o && !cu_smp_o &&
                                   !launch_req_o && !fld_next_o);
            a_l_rst_strobes: `CHK_A (!strobes && !cu_done_o);
        end
    end

`endif // FV_NO_E2E

    // ---------------- two-cycle statements ----------------
    logic r_park_idle, r_two, r_gap, r_dep, r_dep2;
    logic [1:0] r_op;
    always @(posedge clk) begin
        r_park_idle <= rst_n && park_ev && !cu_v_o && !pk_v_o;
        r_two       <= rst_n && r_park_idle && G && !cu_v_o;
        r_gap       <= rst_n && cu_done_o && x_prom && cu_v_o;
        r_dep       <= rst_n && cu_done_o && pk_v_o && x_srcdep && G;
        r_dep2      <= rst_n && r_dep && G && pk_v_o && !cu_v_o;
        r_op        <= sp_op;
    end

`ifndef FV_NO_E2E
    always @(posedge clk) begin
        if (f_pv && rst_n && p_rst) begin
            // ---------------- (p1) park / promote exactly --------------------
            a_p1_pk_v: `CHK_A (pk_v_o == ($past(run_i && !abort_i) &&
                               ($past(park_we_i) || ($past(pk_v_o) && !$past(x_prom)))));
            a_p1_cu_v: `CHK_A (cu_v_o == ($past(run_i && !abort_i) &&
                               ($past(x_prom) || ($past(cu_v_o) && !$past(cu_done_o)))));
            a_p1_first: `CHK_A (fld_first_o == ($past(x_prom) && $past(sp_op) == OP_FIELD));
            // SR load at promotion (4.1)
            if ($past(x_prom && p_ldsr))
                a_p1_sr_load: `CHK_A (sr_o == $past(w_load));
            // ---------------- (f) promotion timing ------------------------
            // from an idle lane: park in t, current (start cycle) in t+2
            if (r_two) a_f_two_after_park: `CHK_A (cu_v_o && !pk_v_o && (r_op != OP_FIELD || fld_first_o));
            // gap-free: parked op promoted at the completion -> current next
            if (r_gap) a_f_gapfree: `CHK_A (cu_v_o && e_first);
            // pk_srcdep waits exactly one cycle
            if (r_dep) a_f_srcdep_wait: `CHK_A (!cu_v_o);
            if (r_dep2) a_f_srcdep_two: `CHK_A (cu_v_o && e_first);
            // ---------------- (a)/(b)/(c) value after completion -------------
            if (q_fin && !q_ldsr && q_RS) begin
                a_i_raws_sr: `CHK_A (sr_o == {16'h0, q_d16});
            end
            // ---------------- (g) result register ------------------------
            // one load per completing op with a destination, none otherwise
            a_g_load: `CHK_A ((res_valid_o && (!$past(res_valid_o) || $past(res_ack_i && !abort_i)) && $past(run_i))
                              == q_ld);
            if (q_ld) begin
                a_g_res_dst: `CHK_A (res_dst_o == q_dst);
                a_g_res_ts:  `CHK_A (res_ts_o == q_ts);    // ts of the completion cycle
                // tag rule: tag only for HOSTQ/CAP, only from a non-IMM source
                if (q_hfin) begin
                    a_g_hold_value: `CHK_A (res_data_o == (q_tagd ? {q_tag, q_sr[23:0]} : q_sr));
                end else if (!q_ldsr) begin
                    a_g_res_value: `CHK_A (res_data_o == (q_tagd ? {q_tag, sr_o[23:0]} : sr_o));
                end else if (q_RS || q_seek) begin
                    a_g_res_hi: `CHK_A (res_data_o[31:24] == (q_tagd ? q_tag : 8'h0));
                end
            end else begin
                a_g_res_keep: `CHK_A (res_dst_o == $past(res_dst_o) && res_data_o == $past(res_data_o) &&
                                      res_ts_o == $past(res_ts_o));
                a_g_res_valid: `CHK_A (res_valid_o == ($past(run_i) && $past(res_valid_o) &&
                                                       !$past(res_ack_i && !abort_i)));
            end
            // ---------------- (p12) hold --------------------------------
            if (hold_o && !$past(hold_o))
                a_p12_hold_rise: `CHK_A ($past(cu_v_o && res_valid_o && !res_ack_i && G && e_needs));
            if ($past(hold_o) && hold_o)
                a_p12_hold_sr: `CHK_A (sr_o == $past(sr_o));      // the held value is kept
            if ($past(hold_o && !cu_done_o && pk_v_o && G))
                a_p12_hold_noprom: `CHK_A (pk_v_o);
            // ---------------- (d) EXPECT and M (p5, p14) ----------------
            if (q_exp) begin
                a_p5_expect_m: `CHK_A (m_o == (q_seek ? 1'b1 : !q_mm));
            end else if (q_crcchk) begin
                a_p14_crcchk: `CHK_A (m_o == q_crcm);
            end else begin
                a_p14_m_keep: `CHK_A (m_o == $past(m_o));
            end
            // ---------------- (h) abort / halt ---------------------------
            if ($past(run_i && abort_i)) begin
                a_p15_abort: `CHK_A (!cu_v_o && !pk_v_o && !hold_o && sr_o == $past(sr_o) &&
                                     m_o == $past(m_o) && res_valid_o == $past(res_valid_o) &&
                                     res_data_o == $past(res_data_o) && res_dst_o == $past(res_dst_o) &&
                                     res_ts_o == $past(res_ts_o));
            end
            if (!$past(run_i)) begin
                a_h_halt: `CHK_A (!cu_v_o && !pk_v_o && !hold_o && !res_valid_o &&
                                  sr_o == $past(sr_o) && m_o == $past(m_o));
            end
            // ---------------- (j) idle stability ---------------------------
            if ($past(!cu_v_o && !pk_v_o && !park_ev)) begin
                a_j_idle: `CHK_A (!cu_v_o && !pk_v_o && sr_o == $past(sr_o));
            end
            // ---------------- (l) first cycle after reset -------------------
        end
        if (f_pv && rst_n && !p_rst) begin
            a_l_rel: `CHK_A (!pk_v_o && !cu_v_o && !res_valid_o && sr_o == 32'h0 && !m_o &&
                             !hold_o && !fld_first_o && res_data_o == 32'h0);
        end
    end
`endif // FV_NO_E2E


`ifndef FV_NO_HARD
    // =====================================================================
    // Layer 2, value group: the end-to-end statements whose proof needs the
    // counting / position invariants below (`prove_e2e`, k-induction, with
    // layer 1 assumed; field.sby explains the assume-guarantee split).
    // =====================================================================
    // position of the watched source bit after e_ds data bits (5.4):
    // ROT for DRIVE (cyclic), INS for the others (no wrap while watched)
    wire [6:0] w_t_up = {1'b0, w_jc} + {1'b0, e_ds};
    wire [6:0] w_t_dn = {1'b0, w_jc} + {1'b0, e_n} - {1'b0, e_ds};
    wire [5:0] w_pos_exp = !e_lsb ? ((w_t_up >= {1'b0, e_n}) ? (w_t_up - {1'b0, e_n}) : w_t_up)
                                  : ((w_jc >= e_ds) ? (w_jc - e_ds) : w_t_dn[5:0]);
    wire       w_inskind = e_kind == EXCH || (e_RX && !e_seek);
    wire [5:0] w_rpos_exp = !e_lsb ? (e_ds - 6'd1 - w_k6) : (e_n - e_ds + w_k6);
    wire       e_pend  = e_kind == EXCH && e_bnh && spend_i && !e_pstf;   // pending data sample

    always @* begin
        if (rst_n) begin
            if (e_chk) begin
                if (tx_go_o && !e_stuffL && e_dl == w_k6) begin
                    // the w_k-th data bit on the wire is source bit pos(w_k)
                    a_e2e_drive_bit: assert (tx_bit_o == ws_val);
                    a_aux_drive_pos: assert (ws_on && e_opos[4:0] == ws_pos);
                end
                if (crc_en_o && e_xd) a_p8_crc_exch: assert (crc_bit_o == e_xbit);
                if (e_dd && e_ds == w_k6) a_aux_dcap_bit: assert (e_obit == ws_val);
                if (e_rd && !e_seek && e_ds == w_k6 && w_in) begin
                    a_aux_cmp_pos: assert (ws_on && ws_pos == e_opos0[4:0]);
                    // EXPECT compares sampled bit k with source bit pos(k)
                    a_e2e_expect_bit: assert (sr_o[e_opos0[4:0]] == ws_val);
                end
                if (e_xd && e_ds == w_k6 && w_in) begin
                    a_aux_xs_pos: assert (ws_on && ws_pos == e_opos0[4:0]);
                end
                if (e_isF && wr_on) a_aux_wr_sr: assert (sr_o[wr_pos] == wr_val);
                if (e_isF && !hold_o) begin
                    // (p2) completion after exactly n data bits
                    a_p2_done_field: assert (cu_done_o == (e_fin && (!e_needs || e_acc)));
                end
                if (e_isF) a_p2_bits_le_n: assert (e_seek || e_ds <= e_n);
                // ---------------- lemmas (checker-internal invariants) -------
                if (e_isF && e_TX) begin
                    a_lem_cnt: assert (e_dl == e_ds + {5'd0, e_ol && !e_ostf} + {5'd0, e_pend});
                    a_lem_dl:  assert (e_dl <= e_n);
                end
                if (e_isF && ws_on) a_lem_ws_pos: assert (ws_pos == w_pos_exp[4:0]);
                if (e_isF) a_lem_ws_on: assert (ws_on == (!e_seek && w_in && (e_kind == DRIVE || e_ds <= w_k6)));
                if (e_isF) a_lem_wr_on: assert (wr_on == (w_inskind && e_ds > w_k6));
                if (e_isF && wr_on) a_lem_wr_pos: assert (wr_pos == w_rpos_exp[4:0]);
                if (e_isF && e_ol && !e_ostf && (e_kind == DRIVE || e_kind == EXCH))
                    a_lem_obit: assert (e_obit == sr_o[(e_kind == EXCH && e_pend) ? e_opos[4:0] : e_opos0[4:0]]);
                if (e_isF && e_pend) a_lem_pbit: assert (e_pbit == sr_o[e_opos0[4:0]]);
                // a pending BAUD sample during a clean transmitting field is
                // the EXCHANGE's own (a DRIVE never has one)
                if (e_isF && e_TX && spend_i) a_lem_spend: assert (e_kind == EXCH && e_bnh);
                if (e_isF && e_first) a_lem_start: assert (e_ds == 6'd0 && e_dl == 6'd0 && !e_ol &&
                                                           e_nl == 6'd0 && !e_mm && !hold_o && !e_rel);
                // a receive field without destuffing completes at its n-th bit
                if (e_isF && e_RX && !e_seek && !e_srx && !hold_o) a_lem_rx_lt_n: assert (e_ds < e_n);
                // a held field has all its bits and nothing on the wire
                if (e_isF && hold_o && !e_seek) a_lem_hold_n: assert (e_ds == e_n);
                if (e_isF && hold_o && e_TX) a_lem_hold_tx: assert (!e_ol && !spend_i && e_dl == e_n);
                // once a BAUD-timed transmit field has launched, the timer is
                // in phase 0 exactly while a wire bit is outstanding
                if (e_isF && e_TX && e_bt && e_lnchd) a_lem_ph0: assert (t_ph0 == e_ol);
                if (e_isF && (e_dl != 6'd0 || e_ol)) a_lem_lnchd: assert (e_lnchd);
            end
            // tracker <-> spec-model state (layer 1 makes these the block's)
            a_lem_pk: assert (sp_op == m_pk_op && sp_ctl == m_pk_ctl && sp_data == m_pk_data);
            a_lem_cu: assert (sc_op == m_cu_op && sc_ctl == m_cu_ctl);
            a_lem_first: assert (e_first == m_first);
            if (e_isF && e_chk) begin
                a_lem_ol:   assert (e_ol == m_lnch);
                if (e_ol) a_lem_ostf: assert (e_ostf == m_lstf);
                if (e_kind == EXCH && e_bnh) a_lem_pstf: assert (e_pstf == m_pstf);
                if (e_TX) a_lem_nl_tx: assert (e_dl == m_nl);
                if (e_RX && e_bt) a_lem_nl_rx: assert (e_nl == m_nl);
                a_lem_bc:   assert (m_bc == (e_seek ? ((e_ds >= {1'b0, e_np}) ? {1'b0, e_np} : e_ds) : e_ds));
                a_lem_rel:  assert (e_rel == m_rel);
                if (e_kind == EXPECT && !e_seek) a_lem_mm: assert (e_mm == m_mm);
                a_lem_tail: assert (m_tail == e_tail);
                if (e_ol) a_lem_ol_tx: assert (e_TX);
            end
            if (e_isPIN || e_isRD) a_lem_eff: assert (e_pw == m_eff);
        end
    end

    always @(posedge clk) begin
        if (f_pv && rst_n && p_rst) begin
            if (q_fin && !q_ldsr && !q_RS) begin
                if (q_seek) begin
                    // (p6) the matched window, right-aligned
                    a_p6_seek_sr: assert (sr_o == {16'h0, q_win});
                end else begin
                    // (p3) value after the field: bits n-31 zero, the watched
                    // source bit (DRIVE) / sampled bit (others) in place
                    a_p3_sr_high0: assert ((sr_o & ~q_mask) == 32'h0);
                    if (q_in && q_drv) a_e2e_drive_sr: assert (q_wson && sr_o[q_jc[4:0]] == q_wsv);
                    if (q_in && !q_drv) a_e2e_sample_sr: assert (q_wron &&
                        sr_o[q_lsb ? q_wk[4:0] : (q_n[4:0] - 5'd1 - q_wk[4:0])] == q_wrv);
                end
            end
            if (q_ld && q_fld && q_fin && !q_seek && q_in && !q_hfin && !(q_tagd && q_rpos >= 5'd24))
                a_e2e_res_bit: assert (res_data_o[q_rpos] == (q_drv ? q_wsv : q_wrv));
        end
    end
`endif // FV_NO_HARD

    // =====================================================================
    // Covers (stand-alone only)
    // =====================================================================
    // per-field record for covers: a trailing stuff bit was sent, a stuff
    // bit received inside / after the data, a launch given back, a stuff
    // bit launched while the last data sample was pending
    logic c_trail, c_rsin, c_rsaft, c_gback, c_xtrail, c_early;
    logic c_rs1;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_trail <= 0; c_rsin <= 0; c_rsaft <= 0; c_gback <= 0; c_xtrail <= 0;
            c_early <= 0; c_rs1 <= 0;
        end else if (x_prom) begin
            c_trail <= 0; c_rsin <= 0; c_rsaft <= 0; c_gback <= 0; c_xtrail <= 0;
            c_early <= 0;
            c_rs1 <= e_isRS && cu_done_o;
        end else begin
            if (tx_go_o && e_stuffL && e_dl == e_n) c_trail <= 1;
            if (e_rstuff && e_ds < e_n)  c_rsin <= 1;
            if (e_rstuff && e_ds == e_n) c_rsaft <= 1;
            if (e_gback) c_gback <= 1;
            if (tx_go_o && e_stuffL && e_dl == e_n && spend_i && e_bt) c_xtrail <= 1;
            if (tx_go_o && e_first) c_early <= 1;
        end
    end

    // event log for the worked timing of field.md 10.1 (cycle numbers from
    // reset release, p = 9, s = 10)
    logic [7:0] cyc;
    logic t1_ok;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin cyc <= 8'd0; t1_ok <= 1'b1; end
        else begin
            if (cyc != 8'hFF) cyc <= cyc + 8'd1;
            // cycle numbering: the first cycle after reset is cyc = 1
            case (cyc + 8'd1)
                8'd9:  if (!(x_prom && sp_op == OP_FIELD && sp_ctl[27:26] == EXCH &&
                             sp_ctl[25:21] == 5'd1 && !sp_ctl[20] && sp_ctl[9] &&
                             cfg_mode_i == EDGE)) t1_ok <= 1'b0;
                8'd10: if (!(fld_first_o && tx_go_o && tx_bit_o == sr_o[1])) t1_ok <= 1'b0;
                8'd23: if (!(ct_i && st_i && rxv_i && !cu_done_o)) t1_ok <= 1'b0;
                8'd29: if (!(lt_i && tx_go_o && tx_bit_o == sr_o[1])) t1_ok <= 1'b0;
                8'd35: if (!(ct_i && st_i && cu_done_o && x_prom)) t1_ok <= 1'b0;
                default: if ((cyc + 8'd1) > 8'd10 && (cyc + 8'd1) < 8'd35 &&
                             (tx_go_o || crc_en_o || (ct_i && cu_fld_o))) t1_ok <= 1'b0;
            endcase
        end
    end

    // field.md 10.2: DRIVE 8, lsb, crc, stuff, source 0x0F; wire bits
    // 1 1 1 [stuff 0] 1 0 0 0 0; CRC fed 1 1 1 1 0 0 0 0; then DRIVE 16, msb,
    // source CRC (pk_srcdep) promoted the cycle after completion. Event
    // order and values are those of the table; the spacing of the ticks is
    // the tick block's (40 cycles per bit) and is compressed here.
    logic [3:0] t2_w;       // wire bits sent
    logic [3:0] t2_c;       // CRC bits fed
    logic       t2_ok, t2_fld, t2_done, t2_next;
    wire [8:0]  t2_wire = 9'b0_0001_0111;   // wire bits k = 0..8, k at bit k: 1 1 1 0 1 0 0 0 0
    wire [8:0]  t2_stf  = 9'b0_0000_1000;   // k = 3 is the stuff bit
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin t2_w <= 0; t2_c <= 0; t2_ok <= 1; t2_fld <= 0; t2_done <= 0; t2_next <= 0; end
        else begin
            if (x_prom && !t2_fld) begin
                t2_fld <= sp_op == OP_FIELD && sp_ctl[27:26] == DRIVE && sp_ctl[25:21] == 5'd7 &&
                          sp_ctl[20] && sp_ctl[19] && sp_ctl[18] && p_v[7:0] == 8'h0F &&
                          sp_ctl[13:10] == NONE;
            end
            if (t2_fld && !t2_done && cu_v_o) begin
                if (tx_go_o) begin
                    t2_w <= t2_w + 4'd1;
                    if (t2_w > 4'd8 || tx_bit_o != t2_wire[t2_w] || e_stuffL != t2_stf[t2_w]) t2_ok <= 0;
                end
                if (crc_en_o) begin
                    t2_c <= t2_c + 4'd1;
                    if (crc_bit_o != (t2_c < 4'd4)) t2_ok <= 0;
                end
                if (cu_done_o) begin
                    t2_done <= 1;
                    if (!(t2_w == 4'd9 && t2_c == 4'd7 && crc_en_o)) t2_ok <= 0;
                end
            end
            if (t2_done && x_prom && sp_op == OP_FIELD && sp_ctl[17:14] == S_CRC &&
                sp_ctl[25:21] == 5'd15 && !sp_ctl[20] && p_cudone && !cu_v_o)
                t2_next <= 1;
        end
    end

    // back-to-back fields: completion and promotion in one cycle
    logic c_b2b_baud, c_b2b_edge;
    always @(posedge clk) begin
        c_b2b_baud <= rst_n && cu_done_o && x_prom && e_isF && sp_op == OP_FIELD && cfg_mode_i == BAUD;
        c_b2b_edge <= rst_n && cu_done_o && x_prom && e_isF && sp_op == OP_FIELD && cfg_mode_i == EDGE;
    end

    wire cdF = rst_n && cu_done_o && e_isF;
    generate
        if (STANDALONE) begin : g_cover
            always @(posedge clk) begin
                if (rst_n) begin
                    // each kind at n = 1, 8, 32, each bit order
                    c_drive_1_msb:  cover (cdF && e_kind == DRIVE  && e_n == 6'd1  && !e_lsb);
                    c_drive_1_lsb:  cover (cdF && e_kind == DRIVE  && e_n == 6'd1  &&  e_lsb);
                    c_drive_8_msb:  cover (cdF && e_kind == DRIVE  && e_n == 6'd8  && !e_lsb);
                    c_drive_8_lsb:  cover (cdF && e_kind == DRIVE  && e_n == 6'd8  &&  e_lsb);
                    c_drive_32_msb: cover (cdF && e_kind == DRIVE  && e_n == 6'd32 && !e_lsb);
                    c_drive_32_lsb: cover (cdF && e_kind == DRIVE  && e_n == 6'd32 &&  e_lsb);
                    c_smp_1_msb:    cover (cdF && e_kind == SAMPLE && e_n == 6'd1  && !e_lsb);
                    c_smp_1_lsb:    cover (cdF && e_kind == SAMPLE && e_n == 6'd1  &&  e_lsb);
                    c_smp_8_msb:    cover (cdF && e_kind == SAMPLE && e_n == 6'd8  && !e_lsb);
                    c_smp_8_lsb:    cover (cdF && e_kind == SAMPLE && e_n == 6'd8  &&  e_lsb);
                    c_smp_32_msb:   cover (cdF && e_kind == SAMPLE && e_n == 6'd32 && !e_lsb);
                    c_smp_32_lsb:   cover (cdF && e_kind == SAMPLE && e_n == 6'd32 &&  e_lsb);
                    c_exch_1_msb:   cover (cdF && e_kind == EXCH   && e_n == 6'd1  && !e_lsb);
                    c_exch_1_lsb:   cover (cdF && e_kind == EXCH   && e_n == 6'd1  &&  e_lsb);
                    c_exch_8_msb:   cover (cdF && e_kind == EXCH   && e_n == 6'd8  && !e_lsb);
                    c_exch_8_lsb:   cover (cdF && e_kind == EXCH   && e_n == 6'd8  &&  e_lsb);
                    c_exch_32_msb:  cover (cdF && e_kind == EXCH   && e_n == 6'd32 && !e_lsb);
                    c_exch_32_lsb:  cover (cdF && e_kind == EXCH   && e_n == 6'd32 &&  e_lsb);
                    c_exp_1_msb:    cover (cdF && e_kind == EXPECT && e_n == 6'd1  && !e_lsb && !e_seek);
                    c_exp_1_lsb:    cover (cdF && e_kind == EXPECT && e_n == 6'd1  &&  e_lsb && !e_seek);
                    c_exp_8_msb:    cover (cdF && e_kind == EXPECT && e_n == 6'd8  && !e_lsb && !e_seek);
                    c_exp_8_lsb:    cover (cdF && e_kind == EXPECT && e_n == 6'd8  &&  e_lsb && !e_seek);
                    c_exp_32_msb:   cover (cdF && e_kind == EXPECT && e_n == 6'd32 && !e_lsb && !e_seek);
                    c_exp_32_lsb:   cover (cdF && e_kind == EXPECT && e_n == 6'd32 &&  e_lsb && !e_seek);
                    // BAUD-timed variants
                    c_drive_baud:   cover (cdF && e_kind == DRIVE  && cfg_mode_i == BAUD && e_n == 6'd2);
                    c_exch_baud:    cover (cdF && e_kind == EXCH   && cfg_mode_i == BAUD && !cfg_honour_i && e_n == 6'd2);
                    c_exch_honour:  cover (cdF && e_kind == EXCH   && cfg_mode_i == BAUD &&  cfg_honour_i && e_n == 6'd2);
                    c_smp_baud:     cover (cdF && e_kind == SAMPLE && cfg_mode_i == BAUD && e_n == 6'd2);
                    c_exch_pulse:   cover (cdF && e_kind == EXCH   && cfg_mode_i == PULSE && e_n == 6'd2);
                    c_smp_pulse:    cover (cdF && e_kind == SAMPLE && cfg_mode_i == PULSE);
                    c_smp_recover:  cover (cdF && e_kind == SAMPLE && cfg_mode_i == RECOVER);
                    // stuffing
                    c_tx_trailing_stuff:  cover (cdF && e_kind == DRIVE && c_trail);
                    c_exch_trailing_stuff: cover (cdF && e_kind == EXCH && c_trail);
                    c_exch_baud_trail_pend: cover (cdF && e_kind == EXCH && c_xtrail);
                    c_rx_stuff_inside:    cover (cdF && e_RX && c_rsin);
                    c_rx_stuff_after:     cover (cdF && e_RX && c_rsaft);
                    c_rx_tail_nostuff:    cover (cdF && e_tail && !stuff_next_i);
                    // EXPECT outcomes, seek
                    c_expect_match:    cover (q_exp && !q_seek && m_o);
                    c_expect_mismatch: cover (q_exp && !q_seek && !m_o);
                    c_expect_crc_src:  cover (cdF && e_kind == EXPECT && sc_ctl[17:14] == S_CRC && e_crcf);
                    c_seek_match_long: cover (cdF && e_seek && e_kind == EXPECT && e_np == 5'd4 && e_tot > 6'd6);
                    c_seek_16:         cover (cdF && e_seek && e_n == 6'd20);
                    c_seek_baud:       cover (cdF && e_seek && cfg_mode_i == BAUD);
                    // hold (overrun) entered and released
                    c_hold:          cover (hold_o);
                    c_hold_release:  cover (hold_o && cu_done_o);
                    c_hold_raws:     cover (hold_o && e_isRS && cu_done_o);
                    c_hold_prom:     cover (hold_o && cu_done_o && x_prom);
                    // abort mid-field, abort of current and parked in one cycle
                    c_abort_mid:     cover (run_i && abort_i && e_isF && e_ds > 6'd0 && e_ds < e_n);
                    c_abort_both:    cover (run_i && abort_i && cu_v_o && pk_v_o);
                    c_abort_hold:    cover (run_i && abort_i && hold_o);
                    // promotion
                    c_b2b_baud_cov:  cover (c_b2b_baud);
                    c_b2b_edge_cov:  cover (c_b2b_edge);
                    c_srcdep_defer:  cover (cu_done_o && pk_v_o && x_srcdep && G);
                    c_src_crc:       cover (x_prom && p_src == S_CRC && sp_op == OP_FIELD);
                    c_src_sr:        cover (x_prom && p_src == S_SR && sp_op == OP_FIELD);
                    c_two_after_park: cover (r_two);
                    // early in EDGE, receive release
                    c_early_edge:    cover (cdF && c_early && cfg_mode_i == EDGE && e_kind == DRIVE);
                    c_release:       cover (tx_rel_o);
                    // PIN, RAW
                    c_pin_wait0:     cover (e_isPIN && cu_done_o && pin_we_o && sc_ctl[11:0] == 12'd0);
                    c_pin_align:     cover (pin_we_o && sc_ctl[15] && cfg_mode_i == EDGE);
                    c_pin_ot_e:      cover (pin_we_o && ot_start_o && ot_done_i && cu_done_o);
                    c_pin_wait:      cover (e_isPIN && cu_done_o && !pin_we_o);
                    c_pin_baud:      cover (pin_we_o && cfg_mode_i == BAUD && bt_stop_i);
                    c_raw_drive:     cover (e_isRD && cu_done_o && !raw_go_o);
                    c_raws_b2b:      cover (e_isRS && cu_done_o && c_rs1);
                    // BAUD sample giving back a launch
                    c_gback_cov:     cover (cdF && c_gback);
                    // the spec-defect corner (e_chk): a transmitting field
                    // starting with an inherited pending BAUD sample
                    c_defect_inherit: cover (e_first && spend_i && e_isF && e_TX);
                    // results
                    c_res_tagged:    cover (q_ld && q_tagd);
                    c_res_on_ack:    cover (cu_done_o && e_needs && res_valid_o && res_ack_i);
                    c_crcchk:        cover (q_crcchk && m_o);
                    // worked timing tables
                    c_t101:          cover (t1_ok && cyc == 8'd35);
                    c_t102:          cover (t2_ok && t2_next);
                    // environment reachability (each assumption is exercised)
                    c_env_spend_up:  cover (spend_i && !$past(spend_i));
                    c_env_spend_dn:  cover (!spend_i && $past(spend_i && st_i));
                    c_env_stf_chg:   cover (stuff_next_i != $past(stuff_next_i) && $past(stf_valid_o));
                    c_env_cfg_chg:   cover ($past(rst_n) && cfg_mode_i != $past(cfg_mode_i));
                    c_env_halt:      cover (!run_i && $past(cu_v_o));
                    c_env_lt_baud:   cover (lt_i && cfg_mode_i == BAUD);
                end
            end
        end
    endgenerate

endmodule
