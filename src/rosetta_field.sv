`default_nettype none
// Operation unit and field execution (docs/blocks/field.md; lane.md 7 and
// 12.2; decisions.md rulings 6, 7, 18-20, 24, 26-29).
//
// Holds the parked and current operations, promotes, executes FIELD, RAW
// and PIN against the rosetta_tick strobes, owns SR, the bit counters, the
// result register and M, and drives rosetta_crc and rosetta_stuff.
// Edge-terminated RAW DRIVE (CODE[12], cfg_rawedge_i) per
// amendment-eth-can.md section 2 (ruling 59), with the timer cancel of 2.5
// (ruling 60, ot_cancel_o); with cfg_rawedge_i = 0 the block is unchanged
// and ot_cancel_o is 0.
//
// Flops (field.md 2): pk 63, cu 31, first/eff/hold/tail 4, SR 32, bc 6,
// nl 6, lnch/lstf/pstf/rel/mm 5, result 53, M 1, plus nlt/nleq 2 (registered
// "nl < n" and "nl == n", the 5.2 implementation note: launch_req and left
// then need no comparator). 203 in all.
//
// Readings of the spec where it is silent or ambiguous (see the report):
//  - seek is decoded only for SAMPLE/EXPECT (2.1 decodes it for every kind,
//    but 5.3 and isa 3.1 define it only for receiving kinds).
//  - an EXCHANGE capture tick in the cycle of an early launch (EL) leaves
//    lnch = 1 (the launch wins; the capture had nothing launched).
//  - with tail = 1 an sv at a data position takes no bit (the field
//    completes in that cycle by 5.6; the bit belongs to nobody).
//  - hold is cleared when the held op completes (only promotion, abort and
//    halt clear it in the text).
//  - a park in a promotion cycle (excluded by the sequencer) wins pk_v.
//  - CRCCHK wins over an EXPECT completion writing M (excluded too).
module rosetta_field (
    input  wire         clk, rst_n,
    input  wire         run_i, abort_i,
    input  wire         park_we_i,
    input  wire  [1:0]  park_op_i,
    input  wire  [27:0] park_ctl_i,
    input  wire  [31:0] park_data_i,
    input  wire         res_ack_i,
    input  wire  [1:0]  cmd_i,
    input  wire  [31:0] cmd_data_i,
    input  wire  [15:0] ts_i,
    input  wire  [1:0]  cfg_mode_i,
    input  wire         lt_i, ct_i, st_i, spend_i, bt_run_i, bt_stop_i, ot_done_i,
    input  wire         cfg_honour_i,
    input  wire         cfg_rawedge_i,      // CODE[12]: RAW DRIVE also ends on dedge_i
    input  wire  [15:0] d16_i,
    input  wire         dedge_i, rxb_i, rxv_i,
    input  wire         stuff_next_i, stuff_bit_i,
    input  wire  [31:0] crc_state_i, crc_value_i,
    output logic        park_full_o, res_valid_o,
    output logic [3:0]  res_dst_o,
    output logic [31:0] res_data_o,
    output logic [15:0] res_ts_o,
    output logic [3:0]  dst_busy_o,
    output logic        m_o,
    output logic [31:0] sr_o,
    output logic        cu_v_o, cu_fld_o, cu_tx_o, cu_smp_o, pk_v_o,
    output logic        launch_req_o, fld_next_o, fld_first_o,
    output logic        ot_start_o, ot_raw_o, ot_cancel_o,
    output logic [15:0] ot_n_o,
    output logic        tx_go_o, tx_bit_o, tx_rel_o, raw_go_o, raw_lvl_o,
    output logic        pin_we_o,
    output logic [3:0]  pin_mask_o, pin_val_o, pin_oe_o,
    output logic        crc_en_o, crc_bit_o,
    output logic        stf_valid_o, stf_bit_o,
    output logic        cu_done_o, hold_o
);
  localparam logic [1:0] OP_FIELD = 2'd0, OP_RAW = 2'd1, OP_PIN = 2'd2;
  localparam logic [1:0] K_DRIVE = 2'd0, K_EXCHANGE = 2'd2, K_EXPECT = 2'd3;
  localparam logic [1:0] MD_BAUD = 2'd0, MD_RECOVER = 2'd2, MD_PULSE = 2'd3;
  localparam logic [3:0] C_HOSTQ = 4'd4, C_CAP = 4'd5, C_CRC = 4'd6, C_MEM = 4'd7,
                         C_IMM = 4'd12, C_SR = 4'd13, C_NONE = 4'd15;

  // ---------------------------------------------------------------- 2. State
  logic        pk_v, cu_v, first, eff, hold, tail;
  logic [1:0]  pk_op, cu_op;
  logic [27:0] pk_ctl, cu_ctl;
  logic [31:0] pk_data, sr;
  logic [5:0]  bc, nl;
  logic        nlt, nleq;                 // nl < n, nl == n
  logic        lnch, lstf, pstf, rel, mm;
  logic        res_v, m;
  logic [3:0]  res_dst;
  logic [31:0] res_data;
  logic [15:0] res_ts;

  logic g;                                // 2.1: G = run_i && !abort_i
  assign g = run_i && !abort_i;

  // R(d) of lane.md 7.6: Rn for d in 0-3, R3 for MEM, none otherwise.
  function automatic logic [3:0] rbit(input logic [3:0] d);
    rbit = (d == C_MEM) ? 4'b1000 : (d[3:2] == 2'b00) ? (4'b0001 << d[1:0]) : 4'b0000;
  endfunction

  // ------------------------------------------- 2.1 Decoded fields (current)
  logic [1:0]  kind;
  logic [4:0]  nm1;                       // n - 1
  logic [3:0]  nwm1;                      // n' - 1, n' = min(n, 16)
  logic [5:0]  n;
  logic        lsb, crcf, stf, early, imm, seek, tx, rx, drv, bt, stx, srx;
  logic        is_fld, is_raw, is_pin, bauds;
  logic [3:0]  dst;
  logic [7:0]  tag;

  assign is_fld = cu_v && cu_op == OP_FIELD;
  assign is_raw = cu_v && cu_op == OP_RAW;
  assign is_pin = cu_v && cu_op == OP_PIN;
  assign kind   = cu_ctl[27:26];
  assign nm1    = cu_ctl[25:21];
  assign n      = {1'b0, nm1} + 6'd1;
  assign nwm1   = nm1[4] ? 4'hF : nm1[3:0];
  assign lsb    = cu_ctl[20];
  assign crcf   = cu_ctl[19];
  assign stf    = cu_ctl[18];
  assign imm    = cu_ctl[17:14] == C_IMM;
  assign early  = cu_ctl[9];
  assign tx     = !kind[0];               // DRIVE, EXCHANGE
  assign rx     = kind[0];                // SAMPLE, EXPECT
  assign seek   = rx && !imm && cu_ctl[8];
  assign tag    = imm ? 8'h00 : cu_ctl[7:0];
  assign drv    = cu_ctl[27];             // RAW direction
  // Destination: FIELD and RAW SAMPLE only (PIN and RAW DRIVE: NONE).
  assign dst    = (cu_op == OP_FIELD || (cu_op == OP_RAW && !drv)) ? cu_ctl[13:10] : C_NONE;
  assign bauds  = cfg_mode_i == MD_BAUD || cfg_mode_i == MD_PULSE;
  assign bt     = cfg_mode_i == MD_BAUD || (cfg_mode_i == MD_PULSE && tx);
  assign stx    = stf && tx;
  assign srx    = stf && rx && !bt;

  // Masks and one-hot positions for width n (32 bits) and n' (16 bits).
  logic [31:0] mask_n, mask_nm1, oh_n;
  logic [15:0] mask_w, mask_wm1, oh_w;
  assign mask_n   = ~(32'hFFFF_FFFE << nm1);
  assign mask_nm1 = ~(32'hFFFF_FFFF << nm1);
  assign oh_n     = 32'h1 << nm1;
  assign mask_w   = ~(16'hFFFE << nwm1);
  assign mask_wm1 = ~(16'hFFFF << nwm1);
  assign oh_w     = 16'h1 << nwm1;

  // opos0 / opos (2.1). spend_i enters only at the last select.
  logic sr_hi, sr_hi2, shp, sr_opos0, sr_opos;
  assign sr_hi    = sr[nm1];
  assign sr_hi2   = sr[nm1 - 5'd1];       // n-2 (unused when n = 1)
  assign shp      = spend_i && !pstf;
  assign sr_opos0 = lsb ? sr[0] : sr_hi;
  assign sr_opos  = lsb ? (shp ? sr[1] : sr[0]) : (shp ? sr_hi2 : sr_hi);

  // 5.4 Shift functions. INS(SR, b) = ins0 | b at its entry position;
  // ROT(SR) = INS(SR, SR[opos0]) for DRIVE.
  logic        ins_b;
  logic [31:0] ins0, sr_ins;
  assign ins0   = lsb ? ((sr >> 1) & mask_nm1) : ({sr[30:0], 1'b0} & mask_n);
  assign ins_b  = (kind == K_DRIVE) ? sr_opos0 : rxb_i;
  assign sr_ins = ins0 | ((lsb ? oh_n : 32'h1) & {32{ins_b}});

  // WIN(SR[15:0], rxb). The match WIN == SR[31:16] within mask(n') is split
  // in 6.1 into the bits without rxb_i (win_rest) and the rxb_i bit.
  logic [15:0] win0, win_pos, win_new;
  assign win0    = lsb ? ((sr[15:0] >> 1) & mask_wm1) : ({sr[14:0], 1'b0} & mask_w);
  assign win_pos = lsb ? oh_w : 16'h1;
  assign win_new = win0 | (win_pos & {16{rxb_i}});

  // Bit counter compares (flops only). hit: min(bc + 1, lim) == lim.
  logic [4:0] lim_m1;
  logic [5:0] lim;
  logic       bc_eq_n, bc_eq_nm1, hit, nl_lt_nm1, nl_eq_nm1, nl_eq_np1;
  assign bc_eq_n   = bc == n;
  assign bc_eq_nm1 = bc == {1'b0, nm1};
  assign lim_m1    = seek ? {1'b0, nwm1} : nm1;
  assign lim       = {1'b0, lim_m1} + 6'd1;
  assign hit       = bc >= {1'b0, lim_m1};
  assign nl_lt_nm1 = nl < {1'b0, nm1};                  // nl + 1 < n
  assign nl_eq_nm1 = nl == {1'b0, nm1};                 // nl + 1 == n
  assign nl_eq_np1 = {1'b0, nl} == ({2'b00, nm1} + 7'd2); // nl - 1 == n

  // ------------------------------------------------------- 5.1 Launch (TX)
  // Section 5 rules: G, current op a FIELD, hold = 0.
  logic fld_act, el, stuff_now, left, le, x;
  assign fld_act   = g && is_fld && !hold;
  assign el        = first && early && !bt;               // ruling 6
  assign stuff_now = stx && stuff_next_i;
  assign left      = nlt || (stuff_now && nleq);
  assign le        = fld_act && tx && (lt_i || el) && !lnch && left;
  assign x         = stuff_now ? stuff_bit_i : sr_opos;

  // 5.1 Release (RX) and BAUD-timed launch count with give-back (5.3, R4).
  logic rel_go, rx_lt, rx_gb;
  assign rel_go = fld_act && rx && !rel
               && (lt_i || (first && ((early && !bt) || cfg_mode_i == MD_RECOVER
                                      || cfg_mode_i == MD_PULSE)));
  assign rx_lt  = fld_act && rx && bt && lt_i;
  assign rx_gb  = fld_act && rx && bt && st_i && !rxv_i;

  // ------------------------------------------------ 5.2 Status to rosetta_tick
  logic launch_req, pk_bt, fld_next;
  assign launch_req = is_fld && bt && !hold && !tail
                   && (tx ? (!lnch && left) : (seek || nlt));
  assign pk_bt      = cfg_mode_i == MD_BAUD || (cfg_mode_i == MD_PULSE && !pk_ctl[26]);
  assign fld_next   = pk_v && pk_op == OP_FIELD && pk_bt
                   && (!cu_v || (cu_op == OP_FIELD && !launch_req));

  // ------------------------------------------------- 5.3 Capture and sample
  logic sv, pmode, cap_d, d_data, x_ct, x_samp, x_ss, x_data, lnch_n;
  logic r_act, stuff_pos, databit, mm_bit;
  assign sv     = st_i && (rxv_i || tx);
  assign pmode  = bt && !cfg_honour_i;    // EXCHANGE sample comes RTRIP later
  // DRIVE: ct_i with lnch; a stuff bit makes no shift.
  assign cap_d  = fld_act && kind == K_DRIVE && ct_i && lnch;
  assign d_data = cap_d && !lstf;
  // EXCHANGE: ct_i clears lnch (and sets pstf with pmode); the sample is the
  // matured pending one (pmode) or the same-cycle one with a launched bit.
  assign x_ct   = fld_act && kind == K_EXCHANGE && ct_i;
  assign x_samp = fld_act && kind == K_EXCHANGE && sv && (pmode || (ct_i && lnch));
  assign x_ss   = pmode ? pstf : lstf;
  assign x_data = x_samp && !x_ss;
  // lnch at the end of the cycle (a launch wins over a same-cycle capture).
  assign lnch_n = le || (lnch && !(fld_act && tx && ct_i));
  // SAMPLE / EXPECT.
  assign r_act     = fld_act && rx;
  assign stuff_pos = r_act && sv && srx && stuff_next_i;
  assign databit   = r_act && sv && !(srx && stuff_next_i) && !tail;
  assign mm_bit    = databit && kind == K_EXPECT && !seek && (rxb_i != sr_opos0);

  // ---------------------------------------------------- 6.1 Completion
  // Each finishing condition is a flop-only "armed" term (_a) ANDed last
  // with the strobes of the cycle, so the tick and coding inputs reach
  // cu_done and promote through few gates. Equivalent to the rules:
  //   DRIVE    fin_d = cap_d && (new bc == n) && !(stx && stuff_next_i)
  //   EXCHANGE fin_x = x_samp && (new bc == n) && !(stx && stuff_next_i)
  //                    && lnch is 0 at the end of the cycle
  //            (pmode: the sample comes without ct_i, so lnch stays 0 iff
  //             lnch = 0 and no launch, i.e. !(lt_i && left), el = 0 with bt;
  //             otherwise ct_i && lnch clears lnch and no launch can follow)
  //   SAMPLE/EXPECT, not seek: n-th data bit and !srx; or tail and
  //            (!stuff_next_i or sv at a stuff position) (5.3, 5.6)
  //   seek     data bit, new bc >= n', window matches (5.3)
  //   RAW SAMPLE dedge_i (7.3)
  logic fa, d_a, xp_a, xn_a, r1_a, r2_a, s_a, rs_a, win_rest, win_pbit;
  logic svr, fin_d, fin_x, fin_r, fin_s, fin_rs, fin_res, has_res, acc, ok, cu_done;
  assign fa       = is_fld && !hold;
  assign win_rest = ((win0 ^ sr[31:16]) & mask_w & ~win_pos) == 16'h0;
  assign win_pbit = |(sr[31:16] & win_pos);
  assign d_a  = fa && kind == K_DRIVE && lnch && (lstf ? bc_eq_n : bc_eq_nm1) && !stuff_now;
  assign xp_a = fa && kind == K_EXCHANGE && pmode && !lnch
             && (pstf ? bc_eq_n : bc_eq_nm1) && !stuff_now;
  assign xn_a = fa && kind == K_EXCHANGE && !pmode && lnch
             && (lstf ? bc_eq_n : bc_eq_nm1) && !stuff_now;
  assign r1_a = fa && rx && !seek && !tail && !srx && hit;
  assign r2_a = fa && rx && !seek && tail;
  assign s_a  = fa && rx && seek && !tail && !(srx && stuff_next_i) && hit && win_rest;
  assign rs_a = is_raw && !drv && !hold;
  assign svr  = st_i && rxv_i;                    // sv of a receiving kind
  assign fin_d   = g && ct_i && d_a;
  assign fin_x   = g && st_i && ((xp_a && !(lt_i && nlt)) || (xn_a && ct_i));
  assign fin_r   = g && ((svr && r1_a) || (r2_a && (!stuff_next_i || svr)));
  assign fin_s   = g && svr && s_a && (win_pbit == rxb_i);
  assign fin_rs  = g && dedge_i && rs_a;
  assign fin_res = fin_d || fin_x || fin_r || fin_s || fin_rs;
  assign has_res = dst != C_NONE;
  assign acc     = !res_v || res_ack_i;
  assign ok      = !has_res || acc;               // the result can be taken

  // 7.1 PIN and 7.2 RAW DRIVE effect cycles. A RAW DRIVE ends at ot_done_i,
  // or with cfg_rawedge_i at the first data edge in or after its effect
  // cycle (amendment 2.1); an edge before e (eff = 0, raw_go = 0) is ignored.
  logic bt_free, pin_wz, pin_rdy, pin_we, raw_go, pin_done, rawd_end, rawd_done;
  assign bt_free   = !bt_run_i || bt_stop_i;
  assign pin_wz    = cu_ctl[11:0] == 12'h000;
  assign pin_rdy   = bauds ? bt_free : (cu_ctl[15] ? lt_i : 1'b1);
  assign pin_we    = g && is_pin && !eff && pin_rdy;
  assign raw_go    = g && is_raw && drv && !eff && (bauds ? bt_free : 1'b1);
  assign pin_done  = (pin_we && (pin_wz || ot_done_i)) || (g && is_pin && eff && ot_done_i);
  assign rawd_end  = ot_done_i || (cfg_rawedge_i && dedge_i);
  assign rawd_done = (raw_go && rawd_end) || (g && is_raw && drv && eff && rawd_end);
  // 2.5: a RAW DRIVE that completes by the edge term alone cancels its timer.
  // rawd_done is the RAW DRIVE's cu_done (it never has a result or hold).
  // With cfg_rawedge_i = 0, rawd_done implies ot_done_i, so this is 0.
  logic ot_cancel;
  assign ot_cancel = rawd_done && !ot_done_i;

  // A completing op whose result cannot be accepted enters hold instead.
  assign cu_done = (g && hold && cu_v && acc) || (fin_res && ok) || pin_done || rawd_done;

  // ---------------------------------------------------- 6.2 Result, SR, M
  logic        res_load, hold_set;
  logic [31:0] sr_upd, v_new, v_res, res_val;
  assign hold_set = fin_res && has_res && !acc;
  assign res_load = g && cu_v && has_res && acc && (hold || fin_res);
  // SR after this cycle's 5.3 update.
  assign sr_upd  = (d_data || x_data || (databit && !seek)) ? sr_ins
                 : (databit && seek) ? {sr[31:16], win_new} : sr;
  assign v_new   = is_raw ? {16'h0, d16_i} : seek ? {16'h0, win_new} : (sr_upd & mask_n);
  assign v_res   = hold ? sr : v_new;
  assign res_val = ((dst == C_HOSTQ || dst == C_CAP) && tag != 8'h00)
                 ? {tag, v_res[23:0]} : v_res;

  // ----------------------------------------------- 3. Parking and promotion
  logic        park, promote, pk_srcdep, pk_load, pk_seek;
  logic [3:0]  pk_src, pk_dst, pk_nwm1;
  logic [4:0]  pk_nm1;
  logic [31:0] pk_val, pk_sr;
  assign park      = run_i && park_we_i && !abort_i;
  assign pk_src    = pk_ctl[17:14];
  assign pk_srcdep = (pk_op == OP_FIELD || (pk_op == OP_RAW && pk_ctl[27]))
                  && (pk_src == C_CRC || pk_src == C_SR);
  // PROMOTE(p) = run_i && pk_v && !abort_i && (!cu_v || cu_done)
  //              && !(pk_srcdep && cu_v); cu_done implies G and cu_v.
  assign promote   = (g && pk_v && !cu_v) || (pk_v && !pk_srcdep && cu_done);
  assign pk_dst    = (pk_op == OP_FIELD || (pk_op == OP_RAW && !pk_ctl[27])) ? pk_ctl[13:10] : C_NONE;

  // 4.1 SR load at promotion (FIELD and RAW DRIVE; RAW SAMPLE, PIN: none).
  assign pk_nm1  = pk_ctl[25:21];
  assign pk_nwm1 = pk_nm1[4] ? 4'hF : pk_nm1[3:0];
  assign pk_seek = pk_ctl[26] && pk_src != C_IMM && pk_ctl[8];
  assign pk_load = pk_op == OP_FIELD || (pk_op == OP_RAW && pk_ctl[27]);
  assign pk_val  = (pk_src == C_CRC) ? crc_value_i : (pk_src == C_SR) ? sr : pk_data;
  assign pk_sr   = (pk_op != OP_FIELD) ? pk_val
                 : pk_seek ? {pk_val[15:0] & ~(16'hFFFE << pk_nwm1), 16'h0}
                 : pk_val & ~(32'hFFFF_FFFE << pk_nm1);

  // ------------------------------------------------------ State update
  // Priority per flop: halt (run_i = 0), abort (8), promotion (3), then the
  // section 5-7 rules of the current op (all gated by G).
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pk_v <= 1'b0; pk_op <= 2'd0; pk_ctl <= 28'h0; pk_data <= 32'h0;
      cu_v <= 1'b0; cu_op <= 2'd0; cu_ctl <= 28'h0;
      first <= 1'b0; eff <= 1'b0; hold <= 1'b0; tail <= 1'b0;
      sr <= 32'h0; bc <= 6'd0; nl <= 6'd0; nlt <= 1'b0; nleq <= 1'b0;
      lnch <= 1'b0; lstf <= 1'b0; pstf <= 1'b0; rel <= 1'b0; mm <= 1'b0;
      res_v <= 1'b0; res_dst <= 4'd0; res_data <= 32'h0; res_ts <= 16'h0;
      m <= 1'b0;
    end else begin
      // 3. Park (the only reader of the shared buses).
      if (park) begin
        pk_op <= park_op_i; pk_ctl <= park_ctl_i; pk_data <= park_data_i;
      end
      if (!g)           pk_v <= 1'b0;     // 8: halt, abort (also over a park)
      else if (park)    pk_v <= 1'b1;
      else if (promote) pk_v <= 1'b0;

      // 3. Promote; cu_v falls at a completion without promotion.
      if (promote) begin
        cu_op <= pk_op; cu_ctl <= pk_ctl;
      end
      if (!g)           cu_v <= 1'b0;
      else if (promote) cu_v <= 1'b1;
      else if (cu_done) cu_v <= 1'b0;

      // 3 / 4.2: first is 1 only in the start cycle s.
      first <= promote;

      // 7.1 / 7.2: eff set in the effect cycle of a timed PIN or a RAW DRIVE.
      if (!g || promote)                        eff <= 1'b0;
      else if ((pin_we && !pin_wz) || raw_go)  eff <= 1'b1;

      // 6.2 hold (cleared again when the held op completes).
      if (!g || promote || cu_done) hold <= 1'b0;
      else if (hold_set)            hold <= 1'b1;

      // 5.6 tail: set by the n-th data bit of an srx field.
      if (!g || promote)                        tail <= 1'b0;
      else if (databit && !seek && hit && srx)  tail <= 1'b1;

      // 5.1 / 5.3 launch bookkeeping.
      if (!g || promote) lnch <= 1'b0;
      else               lnch <= lnch_n;
      if (!run_i || promote) lstf <= 1'b0;   // abort keeps lstf, pstf
      else if (le)           lstf <= stuff_now;
      if (!run_i || promote)   pstf <= 1'b0;
      else if (x_ct && pmode)  pstf <= lstf || !lnch;
      if (!g || promote) rel <= 1'b0;
      else if (rel_go)   rel <= 1'b1;

      // 5.3 EXPECT mismatch (kept through abort, halt and hold).
      if (promote)     mm <= 1'b0;
      else if (mm_bit) mm <= 1'b1;

      // Bit counters (kept through abort and halt; all updates need G).
      if (promote) begin
        bc <= 6'd0; nl <= 6'd0; nlt <= 1'b1; nleq <= 1'b0;
      end else begin
        if (d_data || x_data)  bc <= bc + 6'd1;
        else if (databit)      bc <= hit ? lim : bc + 6'd1;
        if ((le && !stuff_now) || (rx_lt && !rx_gb)) begin
          nl <= nl + 6'd1; nlt <= nl_lt_nm1; nleq <= nl_eq_nm1;
        end else if (rx_gb && !rx_lt && nl != 6'd0) begin
          nl <= nl - 6'd1; nlt <= nlt || nleq; nleq <= nl_eq_np1;
        end
      end

      // SR: promotion load (4.1) over the completion value (6.2, also when
      // entering hold) over the 5.3 shift.
      if (g) begin
        if (promote && pk_load)                sr <= pk_sr;
        else if (fin_res)                      sr <= v_new;
        else if (d_data || x_data || databit)  sr <= sr_upd;
      end

      // 6.2 Result register.
      if (!run_i)         res_v <= 1'b0;
      else if (abort_i)   res_v <= res_v;
      else if (res_load)  res_v <= 1'b1;
      else if (res_ack_i) res_v <= 1'b0;
      if (res_load) begin
        res_dst <= dst; res_data <= res_val; res_ts <= ts_i;
      end

      // 6.2 / 8: M.
      if (run_i && cmd_i == 2'd2)
        m <= crc_state_i == cmd_data_i;
      else if (cu_done && is_fld && kind == K_EXPECT)
        m <= seek ? 1'b1 : !(mm || mm_bit);
    end
  end

  // ------------------------------------------------------------- Outputs (8)
  assign park_full_o  = pk_v;
  assign pk_v_o       = pk_v;
  assign res_valid_o  = res_v;
  assign res_dst_o    = res_dst;
  assign res_data_o   = res_data;
  assign res_ts_o     = res_ts;
  assign m_o          = m;
  assign sr_o         = sr;
  assign dst_busy_o   = ({4{cu_v}} & rbit(dst)) | ({4{pk_v}} & rbit(pk_dst))
                      | ({4{res_v}} & rbit(res_dst));

  assign cu_v_o       = cu_v;
  assign cu_fld_o     = is_fld;
  assign cu_tx_o      = is_fld && tx;
  assign cu_smp_o     = is_fld && kind != K_DRIVE;
  assign launch_req_o = launch_req;
  assign fld_next_o   = fld_next;
  assign fld_first_o  = first && cu_op == OP_FIELD;
  assign ot_start_o   = (pin_we && !pin_wz) || raw_go;
  assign ot_raw_o     = cu_op == OP_RAW;
  assign ot_cancel_o  = ot_cancel;
  assign ot_n_o       = (cu_op == OP_RAW) ? sr[15:0] : {4'h0, cu_ctl[11:0]};

  assign tx_go_o      = le;
  assign tx_bit_o     = x;
  assign tx_rel_o     = rel_go;
  assign raw_go_o     = raw_go;
  assign raw_lvl_o    = cu_ctl[26];
  assign pin_we_o     = pin_we;
  assign pin_mask_o   = cu_ctl[27:24];
  assign pin_val_o    = cu_ctl[23:20];
  assign pin_oe_o     = cu_ctl[19:16];

  // 5.5 CRC: data bits of crcf fields only; stuffing: wire bits of stf fields.
  assign crc_en_o     = crcf && (d_data || x_data || databit);
  assign crc_bit_o    = rx ? rxb_i : sr_opos0;
  assign stf_valid_o  = (le && stx) || stuff_pos || (databit && srx);
  assign stf_bit_o    = le ? x : rxb_i;

  assign cu_done_o    = cu_done;
  assign hold_o       = hold;
endmodule
