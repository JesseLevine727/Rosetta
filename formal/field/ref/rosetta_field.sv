`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/field.md written by the formal
//  property writer, only to show that formal/field/*.sv can PASS and that
//  the properties are non-vacuous (each planted bug below makes a task
//  FAIL). The real block is src/rosetta_field.sv, written independently.
//  Do not integrate, synthesise or copy this file. It follows the same
//  readings R1-R5 as formal/field/field_model.sv.
//
//  Planted bugs (mutation testing), one at a time with a define:
//    MUT_BIT_ORDER      launch reads the wrong end of SR (bit order reversed)
//    MUT_LEN_OFF1       n decoded as ctl[25:21] (field one bit short)
//    MUT_STUFF_COUNTED  a stuff launch counts as a data launch (nl)
//    MUT_TRAIL_DROP     trailing stuff bit neither sent nor waited for
//    MUT_CRC_STUFF      CRC fed at the capture of a stuff bit too
//    MUT_CRC_NOFLAG     CRC fed for fields without the crc flag
//    MUT_EXPECT_AFTER   EXPECT with source CRC compares with the current CRC
//    MUT_M_MISMATCH     M set on mismatch (inverted)
//    MUT_PROMOTE_EARLY  idle lane promotes in the park cycle (1 cycle early)
//    MUT_RES_OVERWRITE  result register overwritten on overrun (no HOLD)
//    MUT_TAG_REG        tag applied to every destination
//    MUT_ABORT_SR       abort clears SR
//    MUT_ABORT_PROGRESS the current op progresses in the abort cycle
//    MUT_RES_NOPARK     a data edge on an idle lane produces a result
//    MUT_SEEK_LIMIT     seek window limited to 15 bits instead of 16
//    MUT_SHP_IGNORED    launch ignores a pending BAUD sample (opos = opos0)
//    MUT_EARLY_BAUD     early also launches in BAUD (ruling 6 ignored)
//    MUT_HOLD_ACTIVE    a held field still launches / releases on lt
//    MUT_TS_LATE        res_ts takes ts_i of the cycle after completion
//    MUT_NO_SRCDEP      source CRC/SR does not wait for the cycle after
//    MUT_GBACK_NONE     BAUD receive never gives a launch back
//    MUT_PIN_TWICE      a PIN with wait takes effect again (eff not set)
//    MUT_TAIL_DATA      a sample in the tail cycle is taken as data
// ===========================================================================
module rosetta_field (
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
    input  wire [15:0] d16_i,
    input  wire        dedge_i,
    input  wire        rxb_i,
    input  wire        rxv_i,
    input  wire        stuff_next_i,
    input  wire        stuff_bit_i,
    input  wire [31:0] crc_state_i,
    input  wire [31:0] crc_value_i,
    output logic        park_full_o,
    output logic        res_valid_o,
    output logic [3:0]  res_dst_o,
    output logic [31:0] res_data_o,
    output logic [15:0] res_ts_o,
    output logic [3:0]  dst_busy_o,
    output logic        m_o,
    output logic [31:0] sr_o,
    output logic        cu_v_o,
    output logic        cu_fld_o,
    output logic        cu_tx_o,
    output logic        cu_smp_o,
    output logic        pk_v_o,
    output logic        launch_req_o,
    output logic        fld_next_o,
    output logic        fld_first_o,
    output logic        ot_start_o,
    output logic        ot_raw_o,
    output logic [15:0] ot_n_o,
    output logic        tx_go_o,
    output logic        tx_bit_o,
    output logic        tx_rel_o,
    output logic        raw_go_o,
    output logic        raw_lvl_o,
    output logic        pin_we_o,
    output logic [3:0]  pin_mask_o,
    output logic [3:0]  pin_val_o,
    output logic [3:0]  pin_oe_o,
    output logic        crc_en_o,
    output logic        crc_bit_o,
    output logic        stf_valid_o,
    output logic        stf_bit_o,
    output logic        cu_done_o,
    output logic        hold_o
);
// ===========================================================================
//  Helper functions (docs/blocks/field.md 1, 2.1, 5.4).
// ===========================================================================

// mask(n): the low n bits set, n = 0..32
function automatic [31:0] f_mask(input [5:0] n);
    f_mask = n[5] ? 32'hFFFF_FFFF : ((32'h1 << n[4:0]) - 32'h1);
endfunction

// 16-bit mask for the seek window, n' = 0..16
function automatic [15:0] f_mask16(input [4:0] n);
    f_mask16 = n[4] ? 16'hFFFF : ((16'h1 << n[3:0]) - 16'h1);
endfunction

// n = ctl[25:21] + 1 (1..32)
function automatic [5:0] f_n(input [27:0] c);
`ifdef MUT_LEN_OFF1
    f_n = {1'b0, c[25:21]} + ((c[25:21] == 5'd0) ? 6'd1 : 6'd0);
`else
    f_n = {1'b0, c[25:21]} + 6'd1;
`endif
endfunction

// n' = min(n, 16)
function automatic [4:0] f_np(input [27:0] c);
`ifdef MUT_SEEK_LIMIT
    f_np = (c[25] || c[24:21] == 4'hF) ? 5'd15 : ({1'b0, c[24:21]} + 5'd1);
`else
    f_np = c[25] ? 5'd16 : ({1'b0, c[24:21]} + 5'd1);
`endif
endfunction

// INS(S, b) (5.4)
function automatic [31:0] f_ins(input [31:0] s, input b, input [5:0] n, input lsb);
    if (!lsb) f_ins = {s[30:0], b} & f_mask(n);
    else      f_ins = ((s >> 1) & f_mask(n - 6'd1)) | ({31'h0, b} << (n - 6'd1));
endfunction

// WIN(W, b) (5.4): INS on 16 bits with n'
function automatic [15:0] f_win(input [15:0] w, input b, input [4:0] np, input lsb);
    if (!lsb) f_win = {w[14:0], b} & f_mask16(np);
    else      f_win = ((w >> 1) & f_mask16(np - 5'd1)) | ({15'h0, b} << (np - 5'd1));
endfunction

// tagged(v, tag) (lane.md 1)
function automatic [31:0] f_tagged(input [31:0] v, input [7:0] tag);
    f_tagged = (tag != 8'h0) ? {tag, v[23:0]} : v;
endfunction

// R(d) of lane.md 7.6 as a one-hot nibble (0 = none)
function automatic [3:0] f_rbit(input [3:0] d);
    case (d)
        4'd0: f_rbit = 4'b0001;
        4'd1: f_rbit = 4'b0010;
        4'd2: f_rbit = 4'b0100;
        4'd3: f_rbit = 4'b1000;
        4'd7: f_rbit = 4'b1000;   // MEM marks R3
        default: f_rbit = 4'b0000;
    endcase
endfunction

// D(op): destination of a parked/current op (FIELD, or RAW with direction
// sample), NONE otherwise
function automatic [3:0] f_dst(input [1:0] op, input [27:0] c);
    if (op == 2'd0 || (op == 2'd1 && !c[27])) f_dst = c[13:10];
    else                                     f_dst = 4'd15;
endfunction

// tag of an op: (src != IMM) ? ctl[7:0] : 0
function automatic [7:0] f_tag(input [27:0] c);
    f_tag = (c[17:14] != 4'd12) ? c[7:0] : 8'h0;
endfunction

// seek = (src != IMM) && ctl[8], SAMPLE and EXPECT only (2.1, ruling 32)
function automatic f_seek(input [27:0] c);
    f_seek = (c[17:14] != 4'd12) && c[8] && c[26];
endfunction

// TX = kind in {DRIVE, EXCHANGE}
function automatic f_tx(input [27:0] c);
    f_tx = (c[27:26] == 2'd0) || (c[27:26] == 2'd2);
endfunction

// bt for a FIELD with ctl c under mode
function automatic f_bt(input [1:0] mode, input [27:0] c);
    f_bt = (mode == 2'd0) || (mode == 2'd3 && f_tx(c));
endfunction

// pk_srcdep: FIELD or RAW DRIVE with src in {CRC, SR}
function automatic f_srcdep(input [1:0] op, input [27:0] c);
    f_srcdep = (op == 2'd0 || (op == 2'd1 && c[27])) &&
               (c[17:14] == 4'd6 || c[17:14] == 4'd13);
endfunction

// SR at promotion (4.1); keep = SR value otherwise
function automatic [31:0] f_load(input [1:0] op, input [27:0] c, input [31:0] data,
                                 input [31:0] crcv, input [31:0] sr, input [31:0] keep);
    logic [31:0] v;
    v = (c[17:14] == 4'd6) ? crcv : (c[17:14] == 4'd13) ? sr : data;
    if (op == 2'd0) begin
        if (f_seek(c)) f_load = {v[15:0] & f_mask16(f_np(c)), 16'h0};
        else           f_load = v & f_mask(f_n(c));
    end else if (op == 2'd1 && c[27]) begin
        f_load = v;
    end else begin
        f_load = keep;
    end
endfunction


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
`ifdef MUT_ABORT_PROGRESS
    wire        G      = run_i;
`else
    wire        G      = run_i && !abort_i;
`endif
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
`ifdef MUT_BIT_ORDER
    wire [5:0]  opos   = !lsb ? (shp ? 6'd1 : 6'd0) : (shp ? (n - 6'd2) : (n - 6'd1));
`elsif MUT_SHP_IGNORED
    wire [5:0]  opos   = opos0;
`else
    wire [5:0]  opos   = lsb ? (shp ? 6'd1 : 6'd0) : (shp ? (n - 6'd2) : (n - 6'd1));
`endif
    wire        bnh    = bt && !cfg_honour_i;

    wire isF   = cu_v && (cu_op == OP_FIELD);
    wire isRAW = cu_v && (cu_op == OP_RAW);
    wire isPIN = cu_v && (cu_op == OP_PIN);
    wire raw_drv = cu_ctl[27];
    wire [11:0] pwait = cu_ctl[11:0];
    wire        palign = cu_ctl[15];

    wire act = G && isF && !hold;          // section 5 applies
`ifdef MUT_HOLD_ACTIVE
    wire actl = G && isF;
`else
    wire actl = act;
`endif

    // ---------------- 5.1 launch ----------------
`ifdef MUT_EARLY_BAUD
    wire EL     = first && early;
`else
    wire EL     = first && early && !bt;
`endif
`ifdef MUT_TRAIL_DROP
    wire left   = (nl < n);
`else
    wire left   = (nl < n) || (stx && stuff_next_i && (nl == n));
`endif
    wire LE     = actl && TX && (lt_i || EL) && !lnch && left;
    wire stuffL = stx && stuff_next_i;
    wire x      = stuffL ? stuff_bit_i : sr[opos[4:0]];

    wire relE   = actl && RX && !rel &&
                  (lt_i || (first && ((early && !bt) || cfg_mode_i == RECOVER ||
                                      cfg_mode_i == PULSE)));
    wire lt_cnt = act && RX && bt && lt_i;
`ifdef MUT_GBACK_NONE
    wire gback  = 1'b0;
`else
    wire gback  = act && RX && bt && st_i && !rxv_i;
`endif

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
`ifdef MUT_TAIL_DATA
    wire rdata  = act && RX && sv && !(srx && stuff_next_i) && !(tail && seek);
`else
    wire rdata  = act && RX && sv && !(srx && stuff_next_i) && !tail;
`endif
    wire [5:0]  lim    = seek ? {1'b0, np} : n;
    wire [5:0]  bc_inc = (bc + 6'd1 > lim) ? lim : (bc + 6'd1);
    wire [15:0] win    = f_win(sr[15:0], rxb_i, np, lsb);
    wire        match  = (bc_inc >= {1'b0, np}) &&
                         ((win & f_mask16(np)) == (sr[31:16] & f_mask16(np)));
`ifdef MUT_EXPECT_AFTER
    wire [5:0]  cpos    = lsb ? bc : (n - 6'd1 - bc);
    wire        bit_cmp = (cu_ctl[17:14] == 4'd6 && kind == EXPECT) ? crc_value_i[cpos[4:0]] : sr[opos0[4:0]];
`else
    wire        bit_cmp = sr[opos0[4:0]];       // SR[opos0]
`endif

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
`ifdef MUT_TRAIL_DROP
                DRIVE:  fin = dcap && (bc_after == n);
                EXCH:   fin = xsmp && (bc_after == n) && !lnch_next_raw;
`else
                DRIVE:  fin = dcap && (bc_after == n) && !(stx && stuff_next_i);
                EXCH:   fin = xsmp && (bc_after == n) && !(stx && stuff_next_i) && !lnch_next_raw;
`endif
                default: begin
                    if (seek) fin = rdata && match;
                    else      fin = (rdata && (bc_inc == n) && !srx) ||
                                    (act && tail && (!stuff_next_i || rstuff));
                end
            endcase
        end else if (isPIN) begin
            fin = (pe && (pwait == 12'd0 || ot_done_i)) || (G && eff && ot_done_i);
        end else if (isRAW && raw_drv) begin
            fin = (re && ot_done_i) || (G && eff && ot_done_i);
        end else if (isRAW && !raw_drv && !hold) begin
            fin = G && dedge_i;
        end
    end

    wire needs_res = (isF || (isRAW && !raw_drv)) && (dst != NONE);
`ifdef MUT_RES_OVERWRITE
    wire acc       = 1'b1;
`else
    wire acc       = !res_v || res_ack_i;
`endif
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
`ifdef MUT_NO_SRCDEP
    wire srcdep  = 1'b0;
`else
    wire srcdep  = f_srcdep(pk_op, pk_ctl);
`endif
`ifdef MUT_PROMOTE_EARLY
    // bypass: an idle lane takes the park buses straight into the current op
    wire byp     = park && !pk_v && !cu_v;
`else
    wire byp     = 1'b0;
`endif
    wire prom    = (run_i && pk_v && !abort_i && (!cu_v || done) && !(srcdep && cu_v)) || byp;
    wire [1:0]  q_op   = byp ? park_op_i   : pk_op;
    wire [27:0] q_ctl  = byp ? park_ctl_i  : pk_ctl;
    wire [31:0] q_data = byp ? park_data_i : pk_data;

    wire [31:0] sr_c = setV ? V : sr_upd;
    wire [31:0] sr_n = prom ? f_load(q_op, q_ctl, q_data, crc_value_i, sr, sr_c) : sr_c;

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
`ifdef MUT_ABORT_SR
            sr <= (run_i && abort_i) ? 32'h0 : sr_n;
`else
            sr <= sr_n;
`endif
            // park register
            if (park) begin
                pk_op <= park_op_i; pk_ctl <= park_ctl_i; pk_data <= park_data_i;
            end
            if (!run_i || abort_i) pk_v <= 1'b0;
            else if (byp)          pk_v <= 1'b0;
            else if (park)         pk_v <= 1'b1;
            else if (prom)         pk_v <= 1'b0;
            // current op
            if (prom) begin cu_op <= q_op; cu_ctl <= q_ctl; end
            if (!run_i || abort_i) cu_v <= 1'b0;
            else if (prom)         cu_v <= 1'b1;
            else if (done)         cu_v <= 1'b0;
            first <= run_i && !abort_i && prom;
            // eff
            if (!run_i || abort_i || prom) eff <= 1'b0;
`ifdef MUT_PIN_TWICE
            else if (re) eff <= 1'b1;
`else
            else if ((pe && pwait != 12'd0) || re) eff <= 1'b1;
`endif
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
`ifdef MUT_STUFF_COUNTED
            else if (LE)               nl <= nl + 6'd1;
`else
            else if (LE && !stuffL)    nl <= nl + 6'd1;
`endif
            else if (lt_cnt && !gback) nl <= nl + 6'd1;
            else if (gback && !lt_cnt) nl <= (nl == 6'd0) ? 6'd0 : nl - 6'd1;
            if (prom)                  mm <= 1'b0;
            else                       mm <= mm_next;
            // result register
            if (!run_i)          res_v <= 1'b0;
            else if (load)       res_v <= 1'b1;
            else if (res_ack_i && !abort_i) res_v <= 1'b0;   // ruling 38
`ifdef MUT_RES_NOPARK
            else if (!cu_v && !pk_v && dedge_i) res_v <= 1'b1;
`endif
            if (run_i && load) begin
                res_dst  <= dst;
`ifdef MUT_TAG_REG
                res_data <= f_tagged(Vr, tag);
`else
                res_data <= (dst == HOSTQ || dst == CAP) ? f_tagged(Vr, tag) : Vr;
`endif
`ifndef MUT_TS_LATE
                res_ts   <= ts_i;
`endif
            end
`ifdef MUT_TS_LATE
            if (res_v) res_ts <= ts_i;
`endif
            // M
`ifdef MUT_M_MISMATCH
            if (done && isF && kind == EXPECT) mflag <= seek ? 1'b1 : mm_next;
`else
            if (done && isF && kind == EXPECT) mflag <= seek ? 1'b1 : !mm_next;
`endif
            if (run_i && cmd_i == 2'd2)        mflag <= (crc_state_i == cmd_data_i);
        end
    end

    // ---------------- outputs ----------------
    always_comb begin
        park_full_o  = pk_v;
        pk_v_o       = pk_v;
        cu_v_o       = cu_v;
        res_valid_o  = res_v;
        res_dst_o    = res_dst;
        res_data_o   = res_data;
        res_ts_o     = res_ts;
        m_o          = mflag;
        sr_o         = sr;
        hold_o       = hold;
        cu_fld_o     = isF;
        cu_tx_o      = isF && TX;
        cu_smp_o     = isF && SMP;
        launch_req_o = isF && bt && !hold && !tail &&
                       (TX ? (!lnch && left) : (seek || (nl < n)));
        fld_next_o   = pk_v && (pk_op == OP_FIELD) && f_bt(cfg_mode_i, pk_ctl) &&
                       (!cu_v || (cu_op == OP_FIELD && !launch_req_o));
        fld_first_o  = first && cu_v && (cu_op == OP_FIELD);
        ot_start_o   = (pe && pwait != 12'd0) || re;
        ot_raw_o     = re;
        ot_n_o       = re ? sr[15:0] : {4'h0, pwait};
        tx_go_o      = LE;
        tx_bit_o     = x;
        tx_rel_o     = relE;
        raw_go_o     = re;
        raw_lvl_o    = cu_ctl[26];
        pin_we_o     = pe;
        pin_mask_o   = cu_ctl[27:24];
        pin_val_o    = cu_ctl[23:20];
        pin_oe_o     = cu_ctl[19:16];
`ifdef MUT_CRC_STUFF
        crc_en_o     = crcf && (dcap || xdata || rdata);
`elsif MUT_CRC_NOFLAG
        crc_en_o     = ddata || xdata || rdata;
`else
        crc_en_o     = crcf && (ddata || xdata || rdata);
`endif
        crc_bit_o    = (ddata || xdata) ? bit_cmp : rxb_i;
        stf_valid_o  = (LE && stx) || rstuff || (rdata && srx);
        stf_bit_o    = LE ? x : rxb_i;
        cu_done_o    = done;
        dst_busy_o   = (cu_v ? f_rbit(f_dst(cu_op, cu_ctl)) : 4'h0) |
                       (pk_v ? f_rbit(f_dst(pk_op, pk_ctl)) : 4'h0) |
                       (res_v ? f_rbit(res_dst) : 4'h0);
    end
endmodule
