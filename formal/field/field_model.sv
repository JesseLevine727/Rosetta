`default_nettype none
// ===========================================================================
//  field_spec: executable model of docs/blocks/field.md (sections 2-8), for
//  the checker in field_props.sv. Written from the spec only. Driven from the
//  block's input ports; every block output has an m_* twin here. Reset
//  asynchronously to the spec reset values (section 2).
//
//  Overseer rulings 32-38 (docs/decisions.md) applied:
//   32  `seek` applies to SAMPLE and EXPECT only (f_seek checks ctl[26]).
//   33  EXCHANGE with early: a ct in the start cycle takes no bit, the early
//       launch wins (lnch = 1).
//   34  A data-position sample while tail = 1 takes no bit; the field
//       completes (tail rule).
//   35  hold is cleared at completion, promotion, abort and halt.
//   36  A held RAW SAMPLE completes on acceptance.
//   37  Abort keeps lstf and pstf; promotion clears them.
//   38  Ack in an abort cycle is ignored; a park in a promotion cycle sets
//       pk_v; CRCCHK wins over an EXPECT completion and acts whenever run_i.
//  Remaining reading: nl wraps modulo 64 (only reachable for a BAUD seek,
//  where launch_req ignores nl; not observable).
//  Amendment H3 (amendment-eth-can.md 2.1, 2.3): input cfg_rawedge_i
//  (CODE[12]); a RAW DRIVE completes at raw_go or eff with
//  (ot_done_i || (cfg_rawedge_i && dedge_i)).
//  Ruling 60 / amendment 2.5: m_ot_cancel = a RAW DRIVE completes in this
//  cycle and ot_done_i = 0.
// ===========================================================================
module field_spec (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run_i,
    input  wire        abort_i,
    input  wire        park_we_i,
    input  wire [1:0]  park_op_i,
    input  wire [27:0] park_ctl_i,
    input  wire [31:0] park_data_i,
    input  wire        res_ack_i,
    input  wire [1:0]  cmd_i,
    input  wire [31:0] cmd_data_i,
    input  wire [15:0] ts_i,
    input  wire [1:0]  cfg_mode_i,
    input  wire        lt_i,
    input  wire        ct_i,
    input  wire        st_i,
    input  wire        spend_i,
    input  wire        bt_run_i,
    input  wire        bt_stop_i,
    input  wire        ot_done_i,
    input  wire        cfg_honour_i,
    input  wire        cfg_rawedge_i,
    input  wire [15:0] d16_i,
    input  wire        dedge_i,
    input  wire        rxb_i,
    input  wire        rxv_i,
    input  wire        stuff_next_i,
    input  wire        stuff_bit_i,
    input  wire [31:0] crc_state_i,
    input  wire [31:0] crc_value_i,
    // model outputs (twins of the block outputs)
    output logic        m_park_full,
    output logic        m_res_valid,
    output logic [3:0]  m_res_dst,
    output logic [31:0] m_res_data,
    output logic [15:0] m_res_ts,
    output logic [3:0]  m_dst_busy,
    output logic        m_m,
    output logic [31:0] m_sr,
    output logic        m_cu_v,
    output logic        m_cu_fld,
    output logic        m_cu_tx,
    output logic        m_cu_smp,
    output logic        m_pk_v,
    output logic        m_launch_req,
    output logic        m_fld_next,
    output logic        m_fld_first,
    output logic        m_ot_start,
    output logic        m_ot_raw,
    output logic [15:0] m_ot_n,
    output logic        m_tx_go,
    output logic        m_tx_bit,
    output logic        m_tx_rel,
    output logic        m_raw_go,
    output logic        m_raw_lvl,
    output logic        m_pin_we,
    output logic [3:0]  m_pin_mask,
    output logic [3:0]  m_pin_val,
    output logic [3:0]  m_pin_oe,
    output logic        m_crc_en,
    output logic        m_crc_bit,
    output logic        m_stf_valid,
    output logic        m_stf_bit,
    output logic        m_cu_done,
    output logic        m_hold,
    output logic        m_ot_cancel,
    // internals, for covers
    output logic        m_prom,
    output logic [5:0]  m_bc_o,
    output logic        m_tail_o,
    output logic        m_lstf_o,
    output logic        m_enter_hold,
    output logic [1:0]  m_pk_op_o,
    output logic [27:0] m_pk_ctl_o,
    output logic [31:0] m_pk_data_o,
    output logic [1:0]  m_cu_op_o,
    output logic [27:0] m_cu_ctl_o,
    output logic        m_first_o,
    output logic        m_eff_o,
    output logic        m_lnch_o,
    output logic        m_pstf_o,
    output logic        m_rel_o,
    output logic        m_mm_o,
    output logic [5:0]  m_nl_o
);
`include "field_spec.svh"

    localparam [1:0] BAUD = 2'd0, EDGE = 2'd1, RECOVER = 2'd2, PULSE = 2'd3;
    localparam [1:0] DRIVE = 2'd0, SAMPLE = 2'd1, EXCH = 2'd2, EXPECT = 2'd3;
    localparam [1:0] OP_FIELD = 2'd0, OP_RAW = 2'd1, OP_PIN = 2'd2;
    localparam [3:0] NONE = 4'd15, HOSTQ = 4'd4, CAP = 4'd5;

    // ---------------- state (section 2) ----------------
    logic        pk_v, cu_v, first, eff, hold, tail, lnch, lstf, pstf, rel, mm;
    logic [1:0]  pk_op, cu_op;
    logic [27:0] pk_ctl, cu_ctl;
    logic [31:0] pk_data, sr;
    logic [5:0]  bc, nl;
    logic        res_v, mflag;
    logic [3:0]  res_dst;
    logic [31:0] res_data;
    logic [15:0] res_ts;

    // ---------------- decodes (2.1) ----------------
    wire        G      = run_i && !abort_i;
    wire [1:0]  kind   = cu_ctl[27:26];
    wire [5:0]  n      = f_n(cu_ctl);
    wire        lsb    = cu_ctl[20];
    wire        crcf   = cu_ctl[19];
    wire        stf    = cu_ctl[18];
    wire [3:0]  dst    = f_dst(cu_op, cu_ctl);
    wire        early  = cu_ctl[9];
    wire        seek   = f_seek(cu_ctl);
    wire [7:0]  tag    = f_tag(cu_ctl);
    wire        TX     = f_tx(cu_ctl);
    wire        SMP    = (kind != DRIVE);
    wire        RX     = (kind == SAMPLE) || (kind == EXPECT);
    wire        bt     = f_bt(cfg_mode_i, cu_ctl);
    wire        stx    = stf && TX;
    wire        srx    = stf && RX && !bt;
    wire [4:0]  np     = f_np(cu_ctl);
    wire [5:0]  opos0  = lsb ? 6'd0 : (n - 6'd1);
    wire        shp    = spend_i && !pstf;
    wire [5:0]  opos   = lsb ? (shp ? 6'd1 : 6'd0) : (shp ? (n - 6'd2) : (n - 6'd1));
    wire        bnh    = bt && !cfg_honour_i;

    wire isF   = cu_v && (cu_op == OP_FIELD);
    wire isRAW = cu_v && (cu_op == OP_RAW);
    wire isPIN = cu_v && (cu_op == OP_PIN);
    wire raw_drv = cu_ctl[27];
    wire [11:0] pwait = cu_ctl[11:0];
    wire        palign = cu_ctl[15];

    wire act = G && isF && !hold;          // section 5 applies

    // ---------------- 5.1 launch ----------------
    wire EL     = first && early && !bt;
    wire left   = (nl < n) || (stx && stuff_next_i && (nl == n));
    wire LE     = act && TX && (lt_i || EL) && !lnch && left;
    wire stuffL = stx && stuff_next_i;
    wire x      = stuffL ? stuff_bit_i : sr[opos[4:0]];

    wire relE   = act && RX && !rel &&
                  (lt_i || (first && ((early && !bt) || cfg_mode_i == RECOVER ||
                                      cfg_mode_i == PULSE)));
    wire lt_cnt = act && RX && bt && lt_i;
    wire gback  = act && RX && bt && st_i && !rxv_i;

    // ---------------- 5.3 capture / sample ----------------
    wire sv     = st_i && (rxv_i || TX);
    // DRIVE
    wire dcap   = act && (kind == DRIVE) && ct_i && lnch;
    wire ddata  = dcap && !lstf;
    // EXCHANGE
    wire xct    = act && (kind == EXCH) && ct_i;
    wire xs_b   = act && (kind == EXCH) && bnh && sv;
    wire xs_o   = act && (kind == EXCH) && !bnh && sv && ct_i && lnch;
    wire xsmp   = xs_b || xs_o;
    wire xss    = xs_b ? pstf : lstf;
    wire xdata  = xsmp && !xss;
    // SAMPLE / EXPECT
    wire rstuff = act && RX && sv && srx && stuff_next_i;
    wire rdata  = act && RX && sv && !(srx && stuff_next_i) && !tail;
    wire [5:0]  lim    = seek ? {1'b0, np} : n;
    wire [5:0]  bc_inc = (bc + 6'd1 > lim) ? lim : (bc + 6'd1);
    wire [15:0] win    = f_win(sr[15:0], rxb_i, np, lsb);
    wire        match  = (bc_inc >= {1'b0, np}) &&
                         ((win & f_mask16(np)) == (sr[31:16] & f_mask16(np)));
    wire        bit_cmp = sr[opos0[4:0]];       // SR[opos0]

    // SR after this cycle's 5.3 update
    logic [31:0] sr_upd;
    always_comb begin
        sr_upd = sr;
        if (ddata)                 sr_upd = f_ins(sr, sr[opos0[4:0]], n, lsb);   // ROT
        else if (xdata)            sr_upd = f_ins(sr, rxb_i, n, lsb);
        else if (rdata && !seek)   sr_upd = f_ins(sr, rxb_i, n, lsb);
        else if (rdata && seek)    sr_upd = {sr[31:16], win};
    end

    wire [5:0] bc_after = (ddata || xdata) ? (bc + 6'd1) : rdata ? bc_inc : bc;
    wire lnch_next_raw = LE ? 1'b1 : ((dcap || xct) ? 1'b0 : lnch);

    // ---------------- 7 PIN / RAW ----------------
    wire bmode = (cfg_mode_i == BAUD) || (cfg_mode_i == PULSE);
    wire pe = G && isPIN && !eff && (bmode ? (!bt_run_i || bt_stop_i) : (palign ? lt_i : 1'b1));
    wire re = G && isRAW && raw_drv && !eff && (bmode ? (!bt_run_i || bt_stop_i) : 1'b1);

    // ---------------- 6.1 completion ----------------
    logic fin;
    always_comb begin
        fin = 1'b0;
        if (isF && !hold) begin
            case (kind)
                DRIVE:  fin = dcap && (bc_after == n) && !(stx && stuff_next_i);
                EXCH:   fin = xsmp && (bc_after == n) && !(stx && stuff_next_i) && !lnch_next_raw;
                default: begin
                    if (seek) fin = rdata && match;
                    else      fin = (rdata && (bc_inc == n) && !srx) ||
                                    (act && tail && (!stuff_next_i || rstuff));
                end
            endcase
        end else if (isPIN) begin
            fin = (pe && (pwait == 12'd0 || ot_done_i)) || (G && eff && ot_done_i);
        end else if (isRAW && raw_drv) begin
            // field.md 6.1 row RAW DRIVE as amended by H3 (amendment 2.1)
            fin = (re && (ot_done_i || (cfg_rawedge_i && dedge_i))) ||
                  (G && eff && (ot_done_i || (cfg_rawedge_i && dedge_i)));
        end else if (isRAW && !raw_drv && !hold) begin
            fin = G && dedge_i;
        end
    end

    wire needs_res = (isF || (isRAW && !raw_drv)) && (dst != NONE);
    wire acc       = !res_v || res_ack_i;
    wire done      = G && cu_v && (hold ? acc : (fin && (!needs_res || acc)));
    wire enter_hold = G && cu_v && !hold && fin && needs_res && !acc;

    // value V (6.2), first completion or entering hold
    wire [31:0] V = (isRAW) ? {16'h0, d16_i} :
                    seek    ? {16'h0, win} :
                              (sr_upd & f_mask(n));
    wire        setV = (done || enter_hold) && !hold && (isF || (isRAW && !raw_drv));
    wire [31:0] Vr   = hold ? sr : V;
    wire        load = done && needs_res;

    wire mm_next = mm || (rdata && !seek && (kind == EXPECT) && (rxb_i != bit_cmp));

    // ---------------- 3 park / promote ----------------
    wire park    = run_i && park_we_i && !abort_i;
    wire srcdep  = f_srcdep(pk_op, pk_ctl);
    wire prom    = run_i && pk_v && !abort_i && (!cu_v || done) && !(srcdep && cu_v);

    wire [31:0] sr_c = setV ? V : sr_upd;
    wire [31:0] sr_n = prom ? f_load(pk_op, pk_ctl, pk_data, crc_value_i, sr, sr_c) : sr_c;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pk_v <= 1'b0; pk_op <= 2'd0; pk_ctl <= 28'h0; pk_data <= 32'h0;
            cu_v <= 1'b0; cu_op <= 2'd0; cu_ctl <= 28'h0;
            first <= 1'b0; eff <= 1'b0; hold <= 1'b0; tail <= 1'b0;
            sr <= 32'h0; bc <= 6'd0; nl <= 6'd0;
            lnch <= 1'b0; lstf <= 1'b0; pstf <= 1'b0; rel <= 1'b0; mm <= 1'b0;
            res_v <= 1'b0; res_dst <= 4'd0; res_data <= 32'h0; res_ts <= 16'h0;
            mflag <= 1'b0;
        end else begin
            // SR (kept by abort and halt: no update happens without G)
            sr <= sr_n;
            // park register
            if (park) begin
                pk_op <= park_op_i; pk_ctl <= park_ctl_i; pk_data <= park_data_i;
            end
            if (!run_i || abort_i) pk_v <= 1'b0;
            else if (park)         pk_v <= 1'b1;
            else if (prom)         pk_v <= 1'b0;
            // current op
            if (prom) begin cu_op <= pk_op; cu_ctl <= pk_ctl; end
            if (!run_i || abort_i) cu_v <= 1'b0;
            else if (prom)         cu_v <= 1'b1;
            else if (done)         cu_v <= 1'b0;
            first <= run_i && !abort_i && prom;
            // eff
            if (!run_i || abort_i || prom) eff <= 1'b0;
            else if ((pe && pwait != 12'd0) || re) eff <= 1'b1;
            // hold
            if (!run_i || abort_i || prom) hold <= 1'b0;
            else if (enter_hold)           hold <= 1'b1;
            else if (done)                 hold <= 1'b0;
            // tail
            if (!run_i || abort_i || prom) tail <= 1'b0;
            else if (done)                 tail <= 1'b0;
            else if (rdata && !seek && (bc_inc == n) && srx) tail <= 1'b1;
            // lnch, lstf, pstf, rel
            if (!run_i || abort_i || prom) lnch <= 1'b0;
            else                           lnch <= lnch_next_raw;
            if (!run_i || prom)            lstf <= 1'b0;
            else if (LE)                   lstf <= stuffL;
            if (!run_i || prom)            pstf <= 1'b0;
            else if (xct && bnh)           pstf <= lstf || !lnch;
            if (!run_i || abort_i || prom) rel <= 1'b0;
            else if (relE)                 rel <= 1'b1;
            // counters (kept by abort and halt)
            if (prom)                  bc <= 6'd0;
            else                       bc <= bc_after;
            if (prom)                  nl <= 6'd0;
            else if (LE && !stuffL)    nl <= nl + 6'd1;
            else if (lt_cnt && !gback) nl <= nl + 6'd1;
            else if (gback && !lt_cnt) nl <= (nl == 6'd0) ? 6'd0 : nl - 6'd1;
            if (prom)                  mm <= 1'b0;
            else                       mm <= mm_next;
            // result register
            if (!run_i)          res_v <= 1'b0;
            else if (load)       res_v <= 1'b1;
            else if (res_ack_i && !abort_i) res_v <= 1'b0;   // ruling 38
            if (run_i && load) begin
                res_dst  <= dst;
                res_data <= (dst == HOSTQ || dst == CAP) ? f_tagged(Vr, tag) : Vr;
                res_ts   <= ts_i;
            end
            // M
            if (done && isF && kind == EXPECT) mflag <= seek ? 1'b1 : !mm_next;
            if (run_i && cmd_i == 2'd2)        mflag <= (crc_state_i == cmd_data_i);
        end
    end

    // ---------------- outputs ----------------
    always_comb begin
        m_park_full  = pk_v;
        m_pk_v       = pk_v;
        m_cu_v       = cu_v;
        m_res_valid  = res_v;
        m_res_dst    = res_dst;
        m_res_data   = res_data;
        m_res_ts     = res_ts;
        m_m          = mflag;
        m_sr         = sr;
        m_hold       = hold;
        m_ot_cancel  = done && isRAW && raw_drv && !ot_done_i;
        m_cu_fld     = isF;
        m_cu_tx      = isF && TX;
        m_cu_smp     = isF && SMP;
        m_launch_req = isF && bt && !hold && !tail &&
                       (TX ? (!lnch && left) : (seek || (nl < n)));
        m_fld_next   = pk_v && (pk_op == OP_FIELD) && f_bt(cfg_mode_i, pk_ctl) &&
                       (!cu_v || (cu_op == OP_FIELD && !m_launch_req));
        m_fld_first  = first && cu_v && (cu_op == OP_FIELD);
        m_ot_start   = (pe && pwait != 12'd0) || re;
        m_ot_raw     = re;
        m_ot_n       = re ? sr[15:0] : {4'h0, pwait};
        m_tx_go      = LE;
        m_tx_bit     = x;
        m_tx_rel     = relE;
        m_raw_go     = re;
        m_raw_lvl    = cu_ctl[26];
        m_pin_we     = pe;
        m_pin_mask   = cu_ctl[27:24];
        m_pin_val    = cu_ctl[23:20];
        m_pin_oe     = cu_ctl[19:16];
        m_crc_en     = crcf && (ddata || xdata || rdata);
        m_crc_bit    = (ddata || xdata) ? bit_cmp : rxb_i;
        m_stf_valid  = (LE && stx) || rstuff || (rdata && srx);
        m_stf_bit    = LE ? x : rxb_i;
        m_cu_done    = done;
        m_dst_busy   = (cu_v ? f_rbit(f_dst(cu_op, cu_ctl)) : 4'h0) |
                       (pk_v ? f_rbit(f_dst(pk_op, pk_ctl)) : 4'h0) |
                       (res_v ? f_rbit(res_dst) : 4'h0);
        m_prom       = prom;
        m_bc_o       = bc;
        m_tail_o     = tail;
        m_lstf_o     = lstf;
        m_enter_hold = enter_hold;
        m_pk_op_o    = pk_op;
        m_pk_ctl_o   = pk_ctl;
        m_pk_data_o  = pk_data;
        m_cu_op_o    = cu_op;
        m_cu_ctl_o   = cu_ctl;
        m_first_o    = first;
        m_eff_o      = eff;
        m_lnch_o     = lnch;
        m_pstf_o     = pstf;
        m_rel_o      = rel;
        m_mm_o       = mm;
        m_nl_o       = nl;
    end
endmodule
