`default_nettype none
`include "lane_wb.svh"
// ===========================================================================
//  Formal properties for rosetta_lane (the lane top: glue, wiring,
//  operation handshake, halt and reset, lane-level behaviour).
//
//  Written from docs/blocks/lane.md, docs/decisions.md (rulings 1-54), the
//  sub-block specs (tick, field, coding, events, pins, stuff, crc) and
//  sequencer.md 5.3 / 6 / 7 ONLY. The lane RTL was not read (port list
//  only). The proven sub-block RTL is a given component and was read for its
//  port lists and the names of a few internal flops used by lemmas.
//
//  Layers (all in this module; lane_fv.sv instantiates it):
//
//   P  PORT LEVEL, lane.md 9 (a_p<N>_*): only the lane ports and checker
//      shadows (mode registers from the SETM strobes, parked / current op
//      and result from the park strobe and port events). Holds for any
//      implementation; needs no internal name.
//   W  WIRING (a_w_*), WB = 1: every input port of the nine sub-block
//      instances of lane.md 3 (u_cfg .. u_pins, observed by name after
//      flatten, see lane_wb_<bus>.ys) equals the expression lane.md 3/5/6
//      gives for it, over lane inputs, other sub-blocks' outputs and the two
//      glue flops (shadowed here: in_q, crcw_q); every lane output equals
//      its lane.md 2.3/7.6 expression. Value buses are compared only under
//      their strobe (a design may gate them). Together with the sub-block
//      proofs this makes the design the spec composition.
//   E  INTEGRATION OBLIGATIONS (a_e_*), WB = 1: the environment rules each
//      sub-block proof ASSUMED (env_* in formal/<block>/*_props.sv),
//      asserted at the design's sub-block ports. Includes the two open
//      lane-level obligations of decisions.md: ruling 42 (a BAUD sample is
//      pending only for a current sampling field and its delayed reference
//      edge is never cancelled: a_e_r42_*) and ruling 53 (launch and capture
//      tick never coincide: a_e_r53_ltct; in the main instance outside EDGE
//      stretch_hold, in the R53 instance (MAIN = 0) everywhere, with the pin
//      model and the compiler rule).
//   L  LEMMAS (l_*), WB = 1: strengthening invariants (checker shadows equal
//      the sub-block state they mirror; BAUD launch/sample bookkeeping; the
//      invariants behind P10 and ruling 53). They are asserted, so they are
//      proven too; some name internal flops of the proven sub-blocks
//      (EXTRA in gen_wb.py).
//
//  The golden composition (layer G, lane_model.sv) and the shared-bus
//  two-copy check (P3) live in lane_fv.sv; the structural property P14 is a
//  static cone check (lane_struct.ys). lane.sby says which task proves what.
//
//  Input assumptions (u_*): see "Environment" and "R53 instance" below; each
//  has a cover (c_u_*, c_r53_*).
//
//  Spec readings (switchable defines; listed in the report):
//   LANE_CRCW_UNGATED     lane.md 5.4: the crcw_q term of u_crc.init_i is
//                         gated by run_i (default) / not gated.
//   LANE_ABORT_CRC_STRICT lane.md 9.11 "crc_o(t+1) = crc_o(t)" also when
//                         crcw_q (the second CRC init of a SETM 8/9 in t-1)
//                         falls in the abort cycle. Default: excepted (lane.md
//                         2.1 allows an abort the cycle after a SETM; 5.3
//                         then re-initialises the CRC in the abort cycle).
//   LANE_NO_SEEK_ASSUME   drop the ruling-49 assumption (no seek in BAUD).
//   LANE_R53_ANY_CFG      drop the R53-instance assumption that EDGE
//                         stretch_hold is configured with an open-drain
//                         clock output idling released and capture on the
//                         rising edge (TICK[9] = TICK[8] = 1, TICK[6] = 0).
//   LANE_R53_NO_XCYC      drop the R53-instance assumption that strobes come
//                         only in the lane's X cycles (sequencer.md 6).
//   LANE_R53_ALLOW_PIN3   drop the R53-instance assumption that no PIN
//                         writing out3 is parked while EDGE stretch_hold is
//                         configured.
//   (Each of the last three is necessary: without it, bmc finds a trace in
//   which launch and capture tick coincide although the master clock obeys
//   the 5-cycle rule; LANE_R53_NO_LEMMAS drops the R53 lemmas to see it.)
//   LANE_NO_DRIVE_VALUE   drop a_p7_drive (prove tasks; see lane.sby).
//   LANE_NO_WHITEBOX      (lane_fv.sv) checker without layers W, E, L.
// ===========================================================================
module lane_props #(
    parameter MAIN = 1,          // 1: main instance, in_i free; 0: R53 instance
    parameter WB   = 1           // white-box layers W, E, L
) (
    input wire         clk,
    input wire         rst_n,
    input wire         run_i,
    input wire         park_we_i,
    input wire [1:0]   park_op_i,
    input wire [27:0]  park_ctl_i,
    input wire [31:0]  park_data_i,
    input wire         res_ack_i,
    input wire         abort_i,
    input wire [3:0]   ev_clr_i,
    input wire         cfg_we_i,
    input wire [4:0]   cfg_reg_i,
    input wire         cfg_hi_i,
    input wire [15:0]  cfg_data_i,
    input wire [1:0]   cmd_i,
    input wire [31:0]  cmd_data_i,
    input wire         data_wait_i,
    input wire [3:0]   in_i,
    input wire [15:0]  ts_i,
    input wire         busy_o,
    input wire         park_full_o,
    input wire         res_valid_o,
    input wire [3:0]   res_dst_o,
    input wire [31:0]  res_data_o,
    input wire [15:0]  res_ts_o,
    input wire [3:0]   dst_busy_o,
    input wire         m_o,
    input wire [3:0]   ev_pend_o,
    input wire [31:0]  sr_o,
    input wire [31:0]  crc_o,
    input wire [3:0]   out_val_o,
    input wire [3:0]   out_oe_o,
    input wire [`LANE_WBW-1:0] wb,
    // R53 instance only: master clock of the clock-input pin, and the pin
    // as the lane sees it (IN_LAT = 3 cycles late), to be fed to in_i.
    input wire         mclk_i,
    output wire        r53_cin_o,
    output wire [1:0]  sh_clkin_o
);
`define LANE_WB_WIRES
`include "lane_wb.svh"
`undef LANE_WB_WIRES

    localparam [1:0] BAUD = 2'd0, EDGE = 2'd1, RECOVER = 2'd2, PULSE = 2'd3;
    localparam [1:0] OP_FIELD = 2'd0, OP_RAW = 2'd1, OP_PIN = 2'd2;
    localparam [1:0] K_DRIVE = 2'd0, K_SAMPLE = 2'd1, K_EXCH = 2'd2, K_EXPECT = 2'd3;
    localparam [3:0] D_HOSTQ = 4'd4, D_CAP = 4'd5, S_CRC = 4'd6, D_MEM = 4'd7,
                     S_IMM = 4'd12, S_SR = 4'd13, D_NONE = 4'd15;

    // ---------------------------------------------------------------------
    // Helpers (lane.md 1, 7.6; field.md 2.1)
    // ---------------------------------------------------------------------
    function automatic [31:0] maskn(input [5:0] n);    // low n bits, n in 0..32
        maskn = (n >= 6'd32) ? 32'hFFFF_FFFF : ((32'd1 << n) - 32'd1);
    endfunction
    // R(d): lane.md 7.6
    function automatic [3:0] rn(input [3:0] d);
        rn = (d < 4'd4) ? (4'd1 << d[1:0]) : (d == D_MEM) ? 4'b1000 : 4'b0000;
    endfunction
    // D(op): destination of a FIELD or a RAW with direction sample, else none
    function automatic [3:0] dof(input [1:0] op, input [27:0] ctl);
        dof = (op == OP_FIELD || (op == OP_RAW && !ctl[27])) ? ctl[13:10] : D_NONE;
    endfunction
    function automatic hasres(input [1:0] op, input [27:0] ctl);
        hasres = dof(op, ctl) != D_NONE;
    endfunction
    // pk_srcdep (lane.md 7.2): FIELD or RAW DRIVE with source CRC or SR
    function automatic srcdep(input [1:0] op, input [27:0] ctl);
        srcdep = (op == OP_FIELD || (op == OP_RAW && ctl[27])) &&
                 (ctl[17:14] == S_CRC || ctl[17:14] == S_SR);
    endfunction
    // clock mapping (tick.md 2): {v3, e3}
    function automatic [1:0] cmap(input c, input od);
        cmap = od ? {1'b0, !c} : {c, 1'b1};
    endfunction

    // ---------------------------------------------------------------------
    // Cycle bookkeeping
    // ---------------------------------------------------------------------
    logic f_pv;                           // previous cycle exists
    initial f_pv = 1'b0;
    always @(posedge clk) f_pv <= 1'b1;
    logic p_rstn;
    always @(posedge clk) p_rstn <= rst_n;
    wire ok1 = f_pv && rst_n && p_rstn;   // previous cycle exists, no reset since

    // ---------------------------------------------------------------------
    // Shadows of the mode registers (lane.md 5.1), from the SETM strobes
    // ---------------------------------------------------------------------
    wire wlo = run_i && cfg_we_i && !cfg_hi_i;
    logic [12:0] sh_tick;  logic [11:0] sh_code;  logic [7:0] sh_dly;  logic [5:0] sh_stuff;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            sh_tick <= '0; sh_code <= '0; sh_dly <= '0; sh_stuff <= '0;
        end else begin
            if (wlo && cfg_reg_i == 5'd0) sh_tick  <= cfg_data_i[12:0];
            if (wlo && cfg_reg_i == 5'd4) sh_dly   <= cfg_data_i[15:8];
            if (wlo && cfg_reg_i == 5'd5) sh_code  <= cfg_data_i[11:0];
            if (wlo && cfg_reg_i == 5'd7) sh_stuff <= cfg_data_i[5:0];
        end
    wire [1:0] sh_mode   = sh_tick[1:0];
    wire       sh_clkout = sh_tick[7];
    wire       sh_hold   = sh_tick[12];
    wire       edge_hold = sh_mode == EDGE && sh_clkout && sh_hold;
    wire       clk_dly   = sh_code[10] && sh_dly != 8'd0;          // tick.md 5 / 9.2
    wire [1:0] map_il    = cmap(sh_tick[8], sh_tick[9]);
    wire [1:0] map_stop  = cmap(sh_tick[10] ? sh_tick[6] : sh_tick[8], sh_tick[9]);
    assign sh_clkin_o = sh_tick[5:4];

    // glue flops (lane.md 3, 5.3), as the spec defines them
    wire wr_crc_now = run_i && cfg_we_i && (cfg_reg_i == 5'd8 || (cfg_reg_i == 5'd9 && !cfg_hi_i));
    logic [3:0] sh_in_q;  logic sh_crcw;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin sh_in_q <= 4'd0; sh_crcw <= 1'b0; end
        else        begin sh_in_q <= in_i; sh_crcw <= wr_crc_now; end

    // ---------------------------------------------------------------------
    // Environment (lane.md 2.1 "Sequencer guarantees" and the port table;
    // sequencer.md 5.3 / 7.1; decisions.md ruling 49). Each has a cover.
    // ---------------------------------------------------------------------
    wire wr_tick_now = wlo && cfg_reg_i == 5'd0;
    logic p_wrtick;
    always @(posedge clk) p_wrtick <= wr_tick_now;
    wire [2:0] nstrobe = {2'd0, park_we_i} + {2'd0, cfg_we_i} + {2'd0, cmd_i != 2'd0} +
                         {2'd0, res_ack_i} + {2'd0, abort_i};
    wire seek_park = park_we_i && park_op_i == OP_FIELD &&
                     (park_ctl_i[27:26] == K_SAMPLE || park_ctl_i[27:26] == K_EXPECT) &&
                     park_ctl_i[17:14] != S_IMM && park_ctl_i[8];

    always @* begin
        if (!f_pv) u_rst: assume (!rst_n);
        // all strobes 0 while halted
        u_halt:  assume (run_i || (!park_we_i && !res_ack_i && !abort_i && ev_clr_i == 4'd0 &&
                                   !cfg_we_i && cmd_i == 2'd0));
        // at most one of park_we, cfg_we, cmd, res_ack, abort (ev_clr may join abort)
        u_one:   assume (nstrobe <= 3'd1);
        u_park:  assume (!park_we_i || !park_full_o);
        u_op:    assume (!park_we_i || park_op_i != 2'd3);
        u_ack:   assume (!res_ack_i || res_valid_o);
        // SETM 0-9 and commands only to an idle lane
        u_idle:  assume (!((cfg_we_i && cfg_reg_i <= 5'd9) || cmd_i != 2'd0) ||
                         (!busy_o && !park_full_o));
        u_reg:   assume (!cfg_we_i || cfg_reg_i <= 5'd13);
`ifndef LANE_NO_SEEK_ASSUME
        // ruling 49: seek is rejected by the compiler under BAUD timing
        // (seek applies to SAMPLE/EXPECT only, ruling 32; never BAUD-timed in
        // PULSE). TICK cannot change between park and execution (u_idle).
        u_noseek: assume (!(seek_park && sh_mode == BAUD));
`endif
        // sequencer.md 5.3: a committing SETM (X cycle t) leaves qwait = 0
        // until the lane's next D cycle (>= t+4); tick.md env_after_wr
        if (f_pv && p_rstn) u_afterwr: assume (!p_wrtick || !data_wait_i);
    end

    // ---------------------------------------------------------------------
    // Registered past values
    // ---------------------------------------------------------------------
    logic        p_run, p_abort, p_park_we, p_ack, p_rv, p_pf, p_busy, p_m, p_shcrcw;
    logic [1:0]  p_park_op, p_cmd;
    logic [27:0] p_park_ctl;
    logic [31:0] p_park_data, p_sr, p_crc;
    logic [15:0] p_ts;
    logic [3:0]  p_ev_clr, p_ev_pend, p_out_val, p_out_oe;
    logic [3:0]  p_rdst;  logic [31:0] p_rdata;  logic [15:0] p_rts;
    logic        p_edge_hold;  logic [1:0] p_map_il;
    always @(posedge clk) begin
        p_run <= run_i; p_abort <= abort_i; p_park_we <= park_we_i; p_ack <= res_ack_i;
        p_rv <= res_valid_o; p_pf <= park_full_o; p_busy <= busy_o; p_m <= m_o;
        p_shcrcw <= sh_crcw;
        p_park_op <= park_op_i; p_park_ctl <= park_ctl_i; p_park_data <= park_data_i;
        p_cmd <= cmd_i; p_sr <= sr_o; p_crc <= crc_o; p_ts <= ts_i;
        p_ev_clr <= ev_clr_i; p_ev_pend <= ev_pend_o; p_out_val <= out_val_o;
        p_out_oe <= out_oe_o; p_rdst <= res_dst_o; p_rdata <= res_data_o; p_rts <= res_ts_o;
        p_edge_hold <= edge_hold; p_map_il <= map_il;
    end

    // ---------------------------------------------------------------------
    // Scoreboard: parked op, current op, from port events. q_* = state as of
    // the previous cycle; c_* = state in this cycle (q_* plus the events at
    // the end of the previous cycle, which are visible only now).
    // ---------------------------------------------------------------------
    wire ev_halt  = !p_run;
    wire ev_abort = p_run && p_abort;
    wire ev_park  = p_run && p_park_we && !p_abort;
    wire ev_prom  = p_run && !p_abort && p_pf && !park_full_o;          // promotion
    wire ev_load  = p_run && !p_abort && (!p_rv || p_ack) && res_valid_o; // result loaded

    logic        q_pk_v, q_cu_v, q_lb, q_exp, q_E, q_ever;
    logic [1:0]  q_pk_op, q_cu_op;
    logic [27:0] q_pk_ctl, q_cu_ctl;
    logic [31:0] q_pk_data, q_cu_data;
    logic        c_pk_v, c_cu_v, c_lb;
    logic [1:0]  c_pk_op, c_cu_op;
    logic [27:0] c_pk_ctl, c_cu_ctl;
    logic [31:0] c_pk_data, c_cu_data;

    // a FIELD that runs on the BAUD timer (field.md 2.1 `bt`)
    function automatic isbf(input [1:0] op, input [27:0] ctl, input [1:0] mode);
        isbf = op == OP_FIELD && (mode == BAUD ||
               (mode == PULSE && (ctl[27:26] == K_DRIVE || ctl[27:26] == K_EXCH)));
    endfunction

    always @* begin
        c_pk_v = q_pk_v; c_pk_op = q_pk_op; c_pk_ctl = q_pk_ctl; c_pk_data = q_pk_data;
        c_cu_v = q_cu_v; c_cu_op = q_cu_op; c_cu_ctl = q_cu_ctl; c_cu_data = q_cu_data;
        c_lb = q_lb;
        if (ok1) begin
            if (ev_halt || ev_abort) begin
                c_pk_v = 1'b0; c_cu_v = 1'b0; c_lb = 1'b0;
            end else begin
                if (ev_load) c_cu_v = 1'b0;
                if (ev_prom) begin
                    c_cu_v = q_pk_v; c_cu_op = q_pk_op; c_cu_ctl = q_pk_ctl;
                    c_cu_data = q_pk_data; c_pk_v = 1'b0;
                    c_lb = isbf(q_pk_op, q_pk_ctl, sh_mode);
                end
                if (ev_park) begin
                    c_pk_v = 1'b1; c_pk_op = p_park_op; c_pk_ctl = p_park_ctl;
                    c_pk_data = p_park_data;
                end
            end
        end
    end

    // listener events E (lane.md 9.12), EXPECT parks (9.9)
    wire park_ev  = run_i && park_we_i;
    wire e_event  = (park_ev && park_op_i == OP_PIN && (park_ctl_i[27:24] & park_ctl_i[19:16]) != 4'd0) ||
                    (park_ev && park_op_i == OP_FIELD &&
                     (park_ctl_i[27:26] == K_DRIVE || park_ctl_i[27:26] == K_EXCH)) ||
                    (park_ev && park_op_i == OP_RAW && park_ctl_i[27]) ||
                    (wlo && cfg_reg_i == 5'd0 && cfg_data_i[7]);
    wire exp_park = park_ev && park_op_i == OP_FIELD && park_ctl_i[27:26] == K_EXPECT;

    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            q_pk_v <= 1'b0; q_cu_v <= 1'b0; q_lb <= 1'b0; q_exp <= 1'b0; q_E <= 1'b0;
            q_ever <= 1'b0;
            q_pk_op <= '0; q_cu_op <= '0; q_pk_ctl <= '0; q_cu_ctl <= '0;
            q_pk_data <= '0; q_cu_data <= '0;
        end else begin
            q_pk_v <= c_pk_v; q_pk_op <= c_pk_op; q_pk_ctl <= c_pk_ctl; q_pk_data <= c_pk_data;
            q_cu_v <= c_cu_v; q_cu_op <= c_cu_op; q_cu_ctl <= c_cu_ctl; q_cu_data <= c_cu_data;
            q_lb   <= c_lb;
            q_exp  <= (run_i && cmd_i == 2'd2) ? 1'b0 : (q_exp || exp_park);
            q_E    <= q_E || e_event;
            q_ever <= q_ever || run_i;
        end

    // expected value structure of a result (field.md 6.2, isa 3.1 tags)
    wire [1:0] cu_kind = q_cu_ctl[27:26];
    wire [5:0] cu_n    = {1'b0, q_cu_ctl[25:21]} + 6'd1;
    wire [3:0] cu_src  = q_cu_ctl[17:14];
    wire [3:0] cu_dst  = q_cu_ctl[13:10];
    wire       cu_imm  = cu_src == S_IMM;
    wire       cu_seek = q_cu_op == OP_FIELD && !cu_imm && q_cu_ctl[8] &&
                         (cu_kind == K_SAMPLE || cu_kind == K_EXPECT);           // ruling 32
    wire [7:0] cu_tag  = cu_imm ? 8'd0 : q_cu_ctl[7:0];
    wire       cu_tagd = (cu_dst == D_HOSTQ || cu_dst == D_CAP) && cu_tag != 8'd0;
    wire [31:0] cu_vm  = (q_cu_op == OP_RAW) ? 32'h0000_FFFF :
                         cu_seek ? maskn((cu_n > 6'd16) ? 6'd16 : cu_n) : maskn(cu_n);
    wire       cu_drv_known = q_cu_op == OP_FIELD && cu_kind == K_DRIVE &&
                              cu_src != S_CRC && cu_src != S_SR;
    wire [31:0] cu_drv_v = q_cu_data & maskn(cu_n);
    wire [31:0] cu_drv_r = cu_tagd ? {cu_tag, cu_drv_v[23:0]} : cu_drv_v;

    wire [3:0] dbusy_exp = (c_cu_v ? rn(dof(c_cu_op, c_cu_ctl)) : 4'd0) |
                           (c_pk_v ? rn(dof(c_pk_op, c_pk_ctl)) : 4'd0) |
                           (res_valid_o ? rn(res_dst_o) : 4'd0);

    // =====================================================================
    // Layer P: lane.md 9, on the ports
    // =====================================================================
    always @* if (MAIN == 1) begin
        // ---- 1. Halted lane drives nothing
        a_p1_oe_halt: assert (run_i || out_oe_o == 4'd0);
        if (ok1 && !p_run) begin
            a_p1_busy:   assert (!busy_o);
            a_p1_pf:     assert (!park_full_o);
            a_p1_rv:     assert (!res_valid_o);
            a_p1_dbusy:  assert (dst_busy_o == 4'd0);
            a_p1_evpend: assert (ev_pend_o == 4'd0);
        end
        // ---- 2. Never started (also during reset)
        if (!q_ever) begin
            a_p2_oe:    assert (out_oe_o == 4'd0);
            a_p2_val:   assert (out_val_o == 4'd0);
            a_p2_stat:  assert (!busy_o && !park_full_o && !res_valid_o && dst_busy_o == 4'd0 &&
                                !m_o && ev_pend_o == 4'd0);
            a_p2_res:   assert (res_dst_o == 4'd0 && res_data_o == 32'd0 && res_ts_o == 16'd0);
            a_p2_sr:    assert (sr_o == 32'd0);
            a_p2_crc:   assert (crc_o == 32'd0);
        end
        if (ok1) begin
            // ---- 4. Idle stability (running lane: sequencer.md 7.2)
            if (!p_busy && !p_pf && !p_park_we && p_run) begin
                a_p4_busy: assert (!busy_o);
                a_p4_pf:   assert (!park_full_o);
                a_p4_rv:   assert (res_valid_o == (p_rv && !p_ack));
            end
            // ---- 5. Park
            if (!p_pf && park_full_o) a_p5_rise: assert (p_park_we && p_run);
            if (ev_park)              a_p5_park: assert (park_full_o);
            if (p_pf && !p_busy && p_run && !p_abort) begin
                a_p5_idle_prom: assert (!park_full_o);   // an idle lane promotes at once
                a_p5_idle_busy: assert (busy_o);         // ... and the op is current
            end
            // ---- 7. Results: none invented, none duplicated, order kept
            if (ev_load) begin
                a_p7_has_op:  assert (q_cu_v && hasres(q_cu_op, q_cu_ctl));
                a_p7_dst:     assert (res_dst_o == dof(q_cu_op, q_cu_ctl));
                a_p7_ts:      assert (res_ts_o == p_ts);
                // value structure: bits above n zero, tag rule (isa 3.1)
                a_p7_tag:     assert (cu_tagd ? (res_data_o[31:24] == cu_tag &&
                                                 (res_data_o[23:0] & ~cu_vm[23:0]) == 24'd0)
                                              : ((res_data_o & ~cu_vm) == 32'd0));
`ifndef LANE_NO_DRIVE_VALUE
                if (cu_drv_known) a_p7_drive: assert (res_data_o == cu_drv_r);
`endif
            end
            if (ev_prom) begin
                a_p7_prom_pk:  assert (q_pk_v);
                // a current op with a destination leaves only with its result
                a_p7_prom_res: assert (!(q_cu_v && hasres(q_cu_op, q_cu_ctl)) || ev_load);
            end
            // stability from the rise of res_valid_o to the end of the ack cycle
            if (p_run && p_rv && !p_ack) begin
                a_p7_keep_v:   assert (res_valid_o);
                a_p7_keep_dst: assert (res_dst_o == p_rdst);
                a_p7_keep_dat: assert (res_data_o == p_rdata);
                a_p7_keep_ts:  assert (res_ts_o == p_rts);
            end
            // ---- 9. M
            if (m_o != p_m) begin
                a_p9_when:  assert ((p_run && p_cmd == 2'd2) || p_busy);
                a_p9_exp:   assert ((p_run && p_cmd == 2'd2) || q_exp);
            end
            // ---- 10. busy_o to the end of the last BAUD bit
            if (p_busy && !busy_o && p_run && !p_abort && q_lb && sh_clkout && !clk_dly) begin
                a_p10_v: assert (out_val_o[3] == map_stop[1]);
                if (run_i) a_p10_e: assert (out_oe_o[3] == map_stop[0]);
            end
            // ---- 11. Abort
            if (ev_abort) begin
                a_p11_pf:   assert (!park_full_o);
                a_p11_sr:   assert (sr_o == p_sr);
`ifdef LANE_ABORT_CRC_STRICT
                a_p11_crc:  assert (crc_o == p_crc);
`else
                if (!p_shcrcw) a_p11_crc: assert (crc_o == p_crc);
`endif
                a_p11_rv:   assert (res_valid_o == p_rv);
                a_p11_val:  assert (out_val_o[2:0] == p_out_val[2:0]);
                a_p11_val3: assert (out_val_o[3] == p_out_val[3] ||
                                    (p_edge_hold && out_val_o[3] == p_map_il[1]));
                if (run_i) begin
                    a_p11_oe:  assert (out_oe_o[2:0] == p_out_oe[2:0]);
                    a_p11_oe3: assert (out_oe_o[3] == p_out_oe[3] ||
                                       (p_edge_hold && out_oe_o[3] == p_map_il[0]));
                end
            end
            // ---- 13. Events, port half
            a_p13_rise: assert ((~p_ev_pend & ev_pend_o & ~{4{p_run}}) == 4'd0);
            a_p13_fall: assert ((p_ev_pend & ~ev_pend_o & ~p_ev_clr & {4{p_run}}) == 4'd0);
        end
        // ---- 8. dst_busy_o formula (checker shadows)
        if (rst_n) a_p8_dbusy: assert (dst_busy_o == dbusy_exp);
        // ---- 12. Listener
        if (!q_E) a_p12_listen: assert (out_oe_o == 4'd0);
    end

    // =====================================================================
    // R53 instance: the clock-input pin (decisions.md ruling 53)
    // =====================================================================
    // Pin model: the clock line is driven low by the external master
    // (mclk_i = 0) or by the lane's out3 (wired AND; out3 released or high
    // leaves it to the master). The lane sees it IN_LAT = 3 cycles late
    // (ruling 2). Compiler rule (ruling 53): stretch_hold requires the
    // master's clock low phase to be at least 5 cycles.
    wire lane_lo = out_oe_o[3] && !out_val_o[3];     // the lane pulls the line low
    wire pin3    = mclk_i && !lane_lo;
    // The line is released (high) before the first cycle; the lane is in
    // reset then (u_rst), and the master's low phases are counted from it.
    // r<k> = pin k cycles ago (r3 is what the lane sees), m<k> = master,
    // lo<k> = lane pull-down, k cycles ago.
    logic r1, r2, r3, r4, m1, m2, m3, m4, m5, lo1, lo2, lo3, lo4;
    initial begin
        r1 = 1'b1; r2 = 1'b1; r3 = 1'b1; r4 = 1'b1;
        m1 = 1'b1; m2 = 1'b1; m3 = 1'b1; m4 = 1'b1; m5 = 1'b1;
        lo1 = 1'b0; lo2 = 1'b0; lo3 = 1'b0; lo4 = 1'b0;
    end
    always @(posedge clk) begin
        r1 <= pin3; r2 <= r1; r3 <= r2; r4 <= r3;
        m1 <= mclk_i; m2 <= m1; m3 <= m2; m4 <= m3; m5 <= m4;
        lo1 <= lane_lo; lo2 <= lo1; lo3 <= lo2; lo4 <= lo3;
    end
    assign r53_cin_o = r3;
    logic [2:0] mlow;          // length of the current master low phase, sat 5 (covers)
    logic       p_mclk;
    initial begin mlow = 3'd0; p_mclk = 1'b1; end
    always @(posedge clk) begin
        p_mclk <= mclk_i;
        mlow   <= mclk_i ? 3'd0 : (mlow == 3'd5 ? 3'd5 : mlow + 3'd1);
    end
    // cycles since the last SETM TICK (1 in the cycle after it, saturating)
    logic [2:0] tw_age;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n)            tw_age <= 3'd7;
        else if (wr_tick_now)  tw_age <= 3'd1;
        else if (tw_age != 3'd7) tw_age <= tw_age + 3'd1;
    // the wheel phase of this lane: xph == 0 in its X cycles (arbitrary
    // initial value, so any phase against reset)
    logic [2:0] xph, p_xph;
    logic       p_dwait;
    (* anyconst *) logic [2:0] hph;
    always @(posedge clk) begin
        xph <= (xph >= 3'd4) ? 3'd0 : xph + 3'd1;
        p_xph <= xph; p_dwait <= data_wait_i;
    end
    wire any_strobe = park_we_i || res_ack_i || abort_i || ev_clr_i != 4'd0 || cfg_we_i ||
                      cmd_i != 2'd0;
    generate if (MAIN == 0) begin : g_r53_env
        always @* begin
            // a master clock low phase lasts at least 5 cycles: a rise follows
            // 5 low cycles (the line is high before cycle 0)
            if (mclk_i && !m1) u_r53_low5: assume (!m2 && !m3 && !m4 && !m5);
            // sequencer.md 6 / 5.10: strobes to a lane only in its X cycle,
            // one every 5 cycles at a fixed phase (so the lane's own clock
            // changes made by SETM TICK are >= 5 cycles apart as well)
`ifndef LANE_R53_NO_XCYC
            u_r53_xcyc: assume (xph == 3'd0 || !any_strobe);
            // sequencer.md 5.3: qwait is written only at the end of the
            // lane's D cycles (xph = 4); a committed instruction, delivery or
            // vector (any strobe in X) leaves it 0; a halt clears it
            u_r53_dw_x: assume (!any_strobe || !data_wait_i);
            if (f_pv) u_r53_dw_d: assume (data_wait_i == p_dwait || p_xph == 3'd4 ||
                                          (!data_wait_i && (!run_i || !p_run)));
            // sequencer.md 7.1: run_i changes only at the end of the lane's
            // D cycle or of the host-write cycle (ph = 1, a fixed phase hph)
            u_r53_hph:  assume (hph <= 3'd4);
            if (f_pv) u_r53_run: assume (run_i == p_run || p_xph == 3'd4 || p_xph == hph);
`endif
`ifndef LANE_R53_ANY_CFG
            // stretch_hold shares the clock pin with the external master: in
            // EDGE with clock output and stretch_hold, the clock output is
            // open-drain (TICK[9]) with idle level released (TICK[8]) and the
            // capture edge rising (TICK[6] = 0: the clock's low phase is the
            // launch level), so the lane pulls the line low only by stretching
            u_r53_cfg: assume (!(wlo && cfg_reg_i == 5'd0 && cfg_data_i[1:0] == EDGE &&
                                 cfg_data_i[12] && cfg_data_i[7]) ||
                               (cfg_data_i[9] && cfg_data_i[8] && !cfg_data_i[6]));
`endif
`ifndef LANE_R53_ALLOW_PIN3
            // the clock pin belongs to the external master: no PIN writes
            // out3 while EDGE stretch_hold is configured
            u_r53_nopin3: assume (!(park_we_i && park_op_i == OP_PIN && park_ctl_i[27] && edge_hold));
`endif
        end
    end endgenerate

    // =====================================================================
    // Layers W, E, L: white box (sub-block ports by lane.md 3 instance names)
    // =====================================================================
    generate if (WB) begin : g_wb
        // spec values of the glue
`ifdef LANE_CRCW_UNGATED
        wire s_crc_init = (run_i && cmd_i == 2'd1) || w_cfg_wr_crc_o || sh_crcw;
`else
        wire s_crc_init = run_i && (cmd_i == 2'd1 || w_cfg_wr_crc_o || sh_crcw);
`endif
        wire s_stf_clr  = (run_i && cmd_i == 2'd3) || w_cfg_wr_stuff_o || !run_i;
        wire [12:0] T = w_cfg_tick_o;
        wire [11:0] C = w_cfg_code_o;
        wire [15:0] PU = w_cfg_pulse_o;
        wire [5:0]  ST = w_cfg_stuff_o;
        wire [6:0]  CC = w_cfg_crccfg_o;

        // ------------------------------------------------------------ W
        always @* if (rst_n && MAIN == 1) begin
            // u_cfg (lane.md 5)
            a_w_cfg_rst:  assert (w_cfg_rst_n == rst_n);
            a_w_cfg_run:  assert (w_cfg_run_i == run_i);
            a_w_cfg_we:   assert (w_cfg_cfg_we_i == cfg_we_i);
            if (cfg_we_i) begin
                a_w_cfg_reg:  assert (w_cfg_cfg_reg_i == cfg_reg_i);
                a_w_cfg_hi:   assert (w_cfg_cfg_hi_i == cfg_hi_i);
                a_w_cfg_data: assert (w_cfg_cfg_data_i == cfg_data_i);
            end
            // u_tick (tick.md 1, lane.md 5.1 / 6)
            a_w_tick_rst:   assert (w_tick_rst_n == rst_n);
            a_w_tick_run:   assert (w_tick_run_i == run_i);
            a_w_tick_abort: assert (w_tick_abort_i == abort_i);
            a_w_tick_wrt:   assert (w_tick_wr_tick_i == w_cfg_wr_tick_o);
            a_w_tick_cfgt:  assert ({w_tick_cfg_hold_i, w_tick_cfg_honour_i, w_tick_cfg_rest_i,
                                     w_tick_cfg_clkod_i, w_tick_cfg_idle_i, w_tick_cfg_clkout_i,
                                     w_tick_cfg_capfall_i, w_tick_cfg_clkin_i, w_tick_cfg_ps_i,
                                     w_tick_cfg_mode_i} == T);
            a_w_tick_per:   assert (w_tick_cfg_period_i == w_cfg_period_o &&
                                    w_tick_cfg_period2_i == w_cfg_period2_o);
            a_w_tick_thr0:  assert (w_tick_cfg_thr0_i == w_cfg_thr0_o);
            a_w_tick_pw:    assert (w_tick_cfg_pw0_i == PU[7:0] && w_tick_cfg_pw1_i == PU[15:8]);
            a_w_tick_dly:   assert (w_tick_cfg_dlyclk_i == C[10] && w_tick_cfg_dly_i == w_cfg_dly_o);
            a_w_tick_in:    assert (w_tick_in_i == in_i);
            a_w_tick_inq:   assert (w_tick_in_q_i == sh_in_q);
            a_w_tick_dline: assert (w_tick_dline_i == w_code_dline_o && w_tick_dedge_i == w_code_dedge_o);
            a_w_tick_pwbit: assert (w_tick_pw_bit_i == w_code_pw_bit_o);
            a_w_tick_cu:    assert (w_tick_cu_v_i == w_field_cu_v_o && w_tick_cu_fld_i == w_field_cu_fld_o &&
                                    w_tick_cu_tx_i == w_field_cu_tx_o && w_tick_cu_smp_i == w_field_cu_smp_o &&
                                    w_tick_pk_v_i == w_field_pk_v_o);
            a_w_tick_lreq:  assert (w_tick_launch_req_i == w_field_launch_req_o &&
                                    w_tick_fld_next_i == w_field_fld_next_o &&
                                    w_tick_fld_first_i == w_field_fld_first_o);
            a_w_tick_dwait: assert (w_tick_data_wait_i == data_wait_i);
            a_w_tick_ots:   assert (w_tick_ot_start_i == w_field_ot_start_o);
            if (w_field_ot_start_o)
                a_w_tick_otn: assert (w_tick_ot_raw_i == w_field_ot_raw_o && w_tick_ot_n_i == w_field_ot_n_o);
            a_w_tick_dcap:  assert (w_tick_dcap_i == w_dly_dcap_o);
            // u_dly (tick.md 9.1)
            a_w_dly_rst:   assert (w_dly_rst_n == rst_n);
            a_w_dly_run:   assert (w_dly_run_i == run_i && w_dly_abort_i == abort_i);
            a_w_dly_wrt:   assert (w_dly_wr_tick_i == w_cfg_wr_tick_o);
            a_w_dly_cfg:   assert (w_dly_cfg_dly_i == w_cfg_dly_o && w_dly_cfg_dlyclk_i == C[10] &&
                                   w_dly_cfg_dlyps_i == C[11]);
            a_w_dly_pt:    assert (w_dly_pt_i == w_tick_pt_o);
            a_w_dly_dreq:  assert (w_dly_dreq_i == w_code_dreq_o);
            if (w_code_dreq_o)
                a_w_dly_dpay: assert (w_dly_dpay_i == w_code_dpay_o && w_dly_dreq_dly_i == w_code_dreq_dly_o);
            a_w_dly_creq:  assert (w_dly_creq_i == w_tick_creq_o);
            if (w_tick_creq_o)
                a_w_dly_cpay: assert (w_dly_cpay_i == w_tick_cpay_o && w_dly_creq_dly_i == w_tick_creq_dly_o &&
                                      w_dly_creq_cap_i == w_tick_creq_cap_o);
            a_w_dly_pinwe: assert (w_dly_pin_we_i == w_field_pin_we_o);
            if (w_field_pin_we_o) a_w_dly_pinm: assert (w_dly_pin_mask_i == w_field_pin_mask_o);
            // u_code (coding.md 1)
            a_w_code_rst:  assert (w_code_rst_n == rst_n);
            a_w_code_run:  assert (w_code_run_i == run_i && w_code_abort_i == abort_i);
            a_w_code_wr:   assert (w_code_wr_code_i == w_cfg_wr_code_o && w_code_wr_tick_i == w_cfg_wr_tick_o);
            a_w_code_mode: assert (w_code_cfg_mode_i == T[1:0]);
            a_w_code_cfg:  assert ({w_code_cfg_dfi_i, w_code_cfg_dif_i, w_code_cfg_od_i, w_code_cfg_inv_i,
                                    w_code_cfg_din_i, w_code_cfg_tx_i, w_code_cfg_rx_i} == C[9:0]);
            a_w_code_pw:   assert (w_code_cfg_pw0_i == PU[7:0] && w_code_cfg_pw1_i == PU[15:8]);
            a_w_code_in:   assert (w_code_in_i == in_i);
            a_w_code_tick: assert (w_code_st_i == w_tick_st_o && w_code_cte_i == w_tick_cte_o &&
                                   w_code_hte_i == w_tick_hte_o && w_code_ge0_i == w_tick_ge0_o);
            a_w_code_go:   assert (w_code_tx_go_i == w_field_tx_go_o && w_code_tx_rel_i == w_field_tx_rel_o &&
                                   w_code_raw_go_i == w_field_raw_go_o);
            if (w_field_tx_go_o)  a_w_code_txb: assert (w_code_tx_bit_i == w_field_tx_bit_o);
            if (w_field_raw_go_o) a_w_code_rawl: assert (w_code_raw_lvl_i == w_field_raw_lvl_o);
            a_w_code_pinwe: assert (w_code_pin_we_i == w_field_pin_we_o);
            if (w_field_pin_we_o)
                a_w_code_pin: assert (w_code_pin_mask0_i == w_field_pin_mask_o[0] &&
                                      w_code_pin_val0_i == w_field_pin_val_o[0] &&
                                      w_code_pin_oe0_i == w_field_pin_oe_o[0]);
            a_w_code_oval: assert (w_code_out_val_i == w_pins_out_val_o[1:0]);
            // u_field (field.md 1)
            a_w_fld_rst:   assert (w_field_rst_n == rst_n);
            a_w_fld_run:   assert (w_field_run_i == run_i && w_field_abort_i == abort_i);
            a_w_fld_pwe:   assert (w_field_park_we_i == park_we_i);
            if (park_we_i)
                a_w_fld_pbus: assert (w_field_park_op_i == park_op_i && w_field_park_ctl_i == park_ctl_i &&
                                      w_field_park_data_i == park_data_i);
            a_w_fld_ack:   assert (w_field_res_ack_i == res_ack_i);
            a_w_fld_cmd:   assert ((w_field_cmd_i == 2'd2) == (cmd_i == 2'd2));   // only CRCCHK is used (field.md 1)
            if (cmd_i == 2'd2) a_w_fld_cmdd: assert (w_field_cmd_data_i == cmd_data_i);
            a_w_fld_ts:    assert (w_field_ts_i == ts_i);
            a_w_fld_cfg:   assert (w_field_cfg_mode_i == T[1:0] && w_field_cfg_honour_i == T[11]);
            a_w_fld_tick:  assert (w_field_lt_i == w_tick_lt_o && w_field_ct_i == w_tick_ct_o &&
                                   w_field_st_i == w_tick_st_o && w_field_spend_i == w_tick_spend_o &&
                                   w_field_bt_run_i == w_tick_bt_run_o && w_field_bt_stop_i == w_tick_bt_stop_o &&
                                   w_field_ot_done_i == w_tick_ot_done_o && w_field_d16_i == w_tick_d16_o);
            a_w_fld_dedge: assert (w_field_dedge_i == w_code_dedge_o);
            if (w_tick_st_o) a_w_fld_rx: assert (w_field_rxb_i == w_code_rxb_o && w_field_rxv_i == w_code_rxv_o);
            a_w_fld_stf:   assert (w_field_stuff_next_i == w_stuff_stuff_next_o &&
                                   w_field_stuff_bit_i == w_stuff_stuff_bit_o);
            a_w_fld_crc:   assert (w_field_crc_state_i == w_crc_state_o && w_field_crc_value_i == w_crc_value_o);
            // u_crc (lane.md 5.1, 5.3, 5.4)
            a_w_crc_rst:   assert (w_crc_rst_n == rst_n);
            a_w_crc_cfg:   assert (w_crc_cfg_poly_i == w_cfg_crcpoly_o && w_crc_cfg_init_i == {32{CC[5]}} &&
                                   w_crc_cfg_width_m1_i == CC[4:0] && w_crc_cfg_xorout_i == CC[6]);
            a_w_crc_init:  assert (w_crc_init_i == s_crc_init);
            a_w_crc_en:    assert (w_crc_en_i == w_field_crc_en_o);
            if (w_field_crc_en_o) a_w_crc_bit: assert (w_crc_bit_i == w_field_crc_bit_o);
            // u_stuff (stuff.md, lane.md 5.4)
            a_w_stf_rst:   assert (w_stuff_rst_n == rst_n);
            a_w_stf_cfg:   assert (w_stuff_cfg_en_i == ST[4] && w_stuff_cfg_n_i == ST[2:0] &&
                                   w_stuff_cfg_mode_i == ST[3]);
            a_w_stf_clr:   assert (w_stuff_clear_i == s_stf_clr);
            a_w_stf_v:     assert (w_stuff_bit_valid_i == w_field_stf_valid_o);
            if (w_field_stf_valid_o) a_w_stf_bit: assert (w_stuff_bit_i == w_field_stf_bit_o);
            // u_evt (events.md 1)
            a_w_evt_rst:   assert (w_evt_rst_n == rst_n);
            a_w_evt_run:   assert (w_evt_run_i == run_i);
            a_w_evt_in:    assert (w_evt_in_i == in_i && w_evt_in_q_i == sh_in_q);
            a_w_evt_pt:    assert (w_evt_pt_i == w_tick_pt_o);
            a_w_evt_cfg:   assert (w_evt_cfg_evc_i == w_cfg_evc_o && w_evt_evc_wr_i == w_cfg_wr_evc_o);
            a_w_evt_stuff: assert (w_evt_cfg_stuffev_i == ST[5] && w_evt_stuff_viol_i == w_stuff_violation_o);
            a_w_evt_clr:   assert (w_evt_ev_clr_i == ev_clr_i);
            // u_pins (pins.md 1)
            a_w_pin_rst:   assert (w_pins_rst_n == rst_n);
            a_w_pin_run:   assert (w_pins_run_i == run_i);
            a_w_pin_dwe:   assert (w_pins_d_we_i == w_dly_d_we_o);
            if (w_dly_d_we_o) a_w_pin_dpay: assert (w_pins_d_pay_i == w_dly_d_pay_o);
            a_w_pin_cwe:   assert (w_pins_c_we_i == w_dly_c_we_o);
            if (w_dly_c_we_o) a_w_pin_cpay: assert (w_pins_c_pay_i == w_dly_c_pay_o);
            a_w_pin_we:    assert (w_pins_pin_we_i == w_field_pin_we_o);
            if (w_field_pin_we_o)
                a_w_pin_pin: assert (w_pins_pin_mask_i == w_field_pin_mask_o &&
                                     w_pins_pin_val_i == w_field_pin_val_o &&
                                     w_pins_pin_oe_i == w_field_pin_oe_o);
            // lane outputs (lane.md 2.3, 7.6)
            a_w_o_busy:  assert (busy_o == (w_field_cu_v_o || w_tick_bt_run_o));
            a_w_o_pf:    assert (park_full_o == w_field_park_full_o);
            a_w_o_rv:    assert (res_valid_o == w_field_res_valid_o);
            if (res_valid_o)
                a_w_o_res: assert (res_dst_o == w_field_res_dst_o && res_data_o == w_field_res_data_o &&
                                   res_ts_o == w_field_res_ts_o);
            a_w_o_dbusy: assert (dst_busy_o == w_field_dst_busy_o);
            a_w_o_m:     assert (m_o == w_field_m_o);
            a_w_o_ev:    assert (ev_pend_o == w_evt_ev_pend_o);
            a_w_o_sr:    assert (sr_o == w_field_sr_o);
            a_w_o_crc:   assert (crc_o == w_crc_value_o);
            a_w_o_val:   assert (out_val_o == w_pins_out_val_o);
            a_w_o_oe:    assert (out_oe_o == w_pins_out_oe_o);
        end

        // ------------------------------------------------------------ E
        // Past values of sub-block nets for the multi-cycle obligations
        logic q_fcuv, q_fcudone, q_fpkv, q_spend_set, q_st, q_spend, q_tph0, q_stfv, q_idle;
        logic q_stn, q_stb, q_crcinit, q_stfclr, q_evcwr0, q_evcwr1, q_evcwr2, q_evcwr3;
        logic [3:0]  q_fire;  logic q_viol, q_stuffev;
        logic [72:0] q_cfgO;  logic [12:0] q_cfgT;  logic [9:0] q_dcfg;  logic [11:0] q_code10;
        logic [5:0]  q_stfcfg; logic [70:0] q_crccfg; logic [63:0] q_evc;
        logic q_wrtick, q_wrcode, q_dpv, q_tidle;
        wire  baudt  = (T[1:0] == BAUD) || (T[1:0] == PULSE && w_field_cu_tx_o);
        wire  G      = run_i && !abort_i;
        wire  idle_l = !busy_o && !park_full_o;
        wire [72:0] cfgO = {w_tick_cfg_period_i, w_tick_cfg_period2_i, w_tick_cfg_thr0_i,
                            w_tick_cfg_pw0_i, w_tick_cfg_pw1_i, w_tick_cfg_dlyclk_i, w_tick_cfg_dly_i};
        wire [12:0] cfgT = {w_tick_cfg_mode_i, w_tick_cfg_ps_i, w_tick_cfg_clkin_i, w_tick_cfg_capfall_i,
                            w_tick_cfg_clkout_i, w_tick_cfg_idle_i, w_tick_cfg_clkod_i, w_tick_cfg_rest_i,
                            w_tick_cfg_honour_i, w_tick_cfg_hold_i};
        wire [9:0]  dcfg = {w_dly_cfg_dly_i, w_dly_cfg_dlyclk_i, w_dly_cfg_dlyps_i};
        wire [11:0] codecfg = {w_code_cfg_rx_i, w_code_cfg_tx_i, w_code_cfg_din_i, w_code_cfg_inv_i,
                               w_code_cfg_od_i, w_code_cfg_dif_i, w_code_cfg_dfi_i, w_code_cfg_mode_i};
        wire [5:0]  stfcfg = {w_stuff_cfg_en_i, w_stuff_cfg_n_i, w_stuff_cfg_mode_i};
        wire [70:0] crccfg = {w_crc_cfg_poly_i, w_crc_cfg_init_i, w_crc_cfg_width_m1_i, w_crc_cfg_xorout_i};
        // tick idle_now (tick_props env_cfg_o): no op, no timer, no sample pending
        wire tick_idle = !run_i || (!w_field_cu_v_o && !w_field_pk_v_o && !w_tick_bt_run_o &&
                                    !w_tick_spend_o && !w_tick_ov);
        always @(posedge clk) begin
            q_fcuv <= w_field_cu_v_o; q_fcudone <= w_field_cu_done_o; q_fpkv <= w_field_pk_v_o;
            q_spend_set <= G && w_tick_ct_o && w_field_cu_smp_o && baudt && !T[11];
            q_st <= w_tick_st_o; q_spend <= w_tick_spend_o;
            q_stfv <= w_field_stf_valid_o; q_idle <= !w_field_cu_v_o && !w_field_pk_v_o && !w_tick_bt_run_o;
            q_stn <= w_stuff_stuff_next_o; q_stb <= w_stuff_stuff_bit_o;
            q_crcinit <= w_crc_init_i; q_stfclr <= w_stuff_clear_i;
            {q_evcwr3, q_evcwr2, q_evcwr1, q_evcwr0} <= w_evt_evc_wr_i;
            q_fire <= w_evt_fire_o; q_viol <= w_stuff_violation_o; q_stuffev <= w_evt_cfg_stuffev_i;
            q_cfgO <= cfgO; q_cfgT <= cfgT; q_dcfg <= dcfg; q_code10 <= codecfg;
            q_stfcfg <= stfcfg; q_crccfg <= crccfg; q_evc <= w_evt_cfg_evc_i;
            q_wrtick <= w_tick_wr_tick_i; q_wrcode <= w_code_wr_code_i; q_dpv <= w_dly_pv;
            q_tidle <= tick_idle;
        end
        // BAUD timer phase tracker of field_props env_t_alt (field.md 9)
        logic t_ph0;
        always_ff @(posedge clk or negedge rst_n)
            if (!rst_n) t_ph0 <= 1'b0;
            else        t_ph0 <= G && (w_tick_lt_o ? 1'b1 : ((w_tick_ct_o && baudt) ? 1'b0 : t_ph0));
        wire spend_exp = p_run && !p_abort && (q_spend_set || (q_spend && !q_st));
        // lane.md 1 / ruling 18: RTRIP = 4. A BAUD sampling capture (no honour,
        // clock not delayed: the capture tick is its own reference) is sampled
        // exactly 4 cycles later unless abort, halt or a TICK write intervene
        // (tick.md 5, 10.8 with the lane's RTRIP; catches a wrong u_tick RTRIP)
        wire dmode = w_tick_cfg_dlyclk_i && w_tick_cfg_dly_i != 8'd0 && w_tick_cfg_clkout_i;
        wire rt_go = G && w_tick_ct_o && w_field_cu_smp_o && baudt && !T[11] && !dmode;
        wire rt_ok = G && !w_tick_wr_tick_i;
        logic rt1, rt2, rt3, rt4;
        always_ff @(posedge clk or negedge rst_n)
            if (!rst_n) begin rt1 <= 1'b0; rt2 <= 1'b0; rt3 <= 1'b0; rt4 <= 1'b0; end
            else begin rt1 <= rt_go; rt2 <= rt1 && rt_ok; rt3 <= rt2 && rt_ok; rt4 <= rt3 && rt_ok; end

        always @* if (rst_n && MAIN == 1) begin
            // ---- ruling 42 / tick env_cfg_o: an idle lane has no BAUD sample
            // pending and no operation timer running (the sequencer writes
            // PERIOD..CRCCFG, and with them tick's cfg_ inputs, only then)
            if (idle_l) begin
                a_e_r42_spend: assert (!w_tick_spend_o);
                a_e_r42_ot:    assert (!w_tick_ov);
            end
            // ruling 42 itself: while a BAUD sample waits for its reference
            // (the delayed capture-level clock change, tick.md 5), that change
            // is pending in u_dly (never cancelled), so the sample will come
            if (w_tick_spend_o && !w_tick_sarm)
                a_e_r42_ref: assert (w_tick_cfg_dlyclk_i && w_tick_cfg_dly_i != 8'd0 && w_tick_cfg_clkout_i &&
                                     w_dly_pv && w_dly_pclk && w_dly_pcap);
            if (baudt && !T[11] && !dmode)
                a_e_rtrip: assert (w_tick_st_o == (rt4 && G));
            // ---- tick env_pulse (tick_props): PULSE, receiving field current
            if (T[1:0] == PULSE)
                a_e_t_pulse: assert ((!w_field_launch_req_o || w_field_cu_tx_o) &&
                                     (!w_tick_spend_o || w_field_cu_tx_o));
            // ---- tick env_wrtick, env_abort, env_fld, env_otrun
            a_e_t_wrtick: assert (!w_tick_wr_tick_i || (run_i && !abort_i && !w_field_cu_v_o &&
                                                        !w_field_pk_v_o && !w_tick_bt_run_o));
            a_e_t_fld:    assert ((!w_field_fld_first_o || w_field_cu_fld_o) &&
                                  (!w_field_launch_req_o || w_field_cu_fld_o) &&
                                  (!w_field_ot_start_o || w_field_cu_v_o) &&
                                  (!w_field_cu_fld_o || w_field_cu_v_o) &&
                                  (!w_field_cu_tx_o || w_field_cu_fld_o) &&
                                  (!w_field_cu_smp_o || w_field_cu_fld_o));
            a_e_t_otrun:  assert (!(w_field_ot_start_o && !w_field_ot_raw_o &&
                                    w_field_ot_n_o[11:0] == 12'd0 && w_tick_ov));
            // ---- field env_t_* (field_props): the tick-to-field contract
            //      (rulings 50, 53). ltct: see a_e_r53_ltct below.
            a_e_f_ltreq:  assert (!(baudt && w_tick_lt_o) || w_field_launch_req_o);
            a_e_f_sp0:    assert (!w_tick_spend_o || ((T[1:0] == BAUD || T[1:0] == PULSE) && !T[11]));
            a_e_f_st:     assert (!(baudt && !T[11] && w_tick_st_o) || w_tick_spend_o);
            a_e_f_ct:     assert (!(w_tick_spend_o && w_tick_ct_o));
            a_e_f_alt:    assert (!(baudt && w_tick_ct_o) || t_ph0);
            a_e_f_stct:   assert (baudt ? (T[11] ? (w_tick_st_o == (w_tick_ct_o && w_field_cu_smp_o)) : 1'b1)
                                        : (w_tick_st_o == w_tick_ct_o));
            if (ok1) begin
                a_e_f_spup: assert (!w_tick_spend_o || spend_exp);
                a_e_f_spdn: assert (w_tick_spend_o || !spend_exp || q_idle);
            end
            // ---- coding env (coding_props): one request source, gating
            a_e_c_one:  assert (!((w_field_tx_go_o && w_field_tx_rel_o) || (w_field_tx_go_o && w_field_raw_go_o) ||
                                  (w_field_tx_rel_o && w_field_raw_go_o)));
            a_e_c_gate: assert (!((abort_i || !run_i) &&
                                  (w_field_tx_go_o || w_field_tx_rel_o || w_field_raw_go_o ||
                                   w_tick_st_o || w_tick_cte_o || w_tick_hte_o)));
            if (ok1 && p_run) begin
                // ---- cfg_ change rules of the sub-blocks (conventions.md):
                // tick env_cfg_o / env_cfg_t
                a_e_t_cfgo:  assert (cfgO == q_cfgO || q_tidle);
                a_e_t_cfgt:  assert (cfgT == q_cfgT || q_wrtick);
                // coding env_cfg_code / env_cfg_mode
                a_e_c_cfg:   assert (codecfg[11:2] == q_code10[11:2] || q_wrcode);
                a_e_c_mode:  assert (codecfg[1:0] == q_code10[1:0] || q_wrtick);
                // stuff: cfg_ change only in a cycle with clear_i (stuff.md)
                a_e_s_cfg:   assert (stfcfg == q_stfcfg || q_stfclr);
                // crc: cfg_ change only in a cycle with init_i (crc.md)
                a_e_r_cfg:   assert (crccfg == q_crccfg || q_crcinit);
                // events: EVCk changes only with evc_wr[k] (events.md 4)
                a_e_v_cfg0:  assert (w_evt_cfg_evc_i[15:0]  == q_evc[15:0]  || q_evcwr0);
                a_e_v_cfg1:  assert (w_evt_cfg_evc_i[31:16] == q_evc[31:16] || q_evcwr1);
                a_e_v_cfg2:  assert (w_evt_cfg_evc_i[47:32] == q_evc[47:32] || q_evcwr2);
                a_e_v_cfg3:  assert (w_evt_cfg_evc_i[63:48] == q_evc[63:48] || q_evcwr3);
                // field env_stf: stuff outputs change only after stf_valid or a clear
                a_e_f_stf:   assert ({w_stuff_stuff_next_o, w_stuff_stuff_bit_o} == {q_stn, q_stb} ||
                                     q_stfv || q_stfclr);
                // ---- 13 (composition, events.md 3.4 with the lane wiring)
                a_p13_exact: assert (ev_pend_o == (q_fire | {q_viol && q_stuffev, 3'b000} |
                                                   (p_ev_pend & ~p_ev_clr)));
            end
        end
        always @* if (rst_n) begin
            // ---- ruling 53: launch and capture tick never in one cycle
            //      (field env_t_ltct). Main instance: in_i is free, so only
            //      outside EDGE stretch_hold (where ruling 53 shows it false
            //      for a short clock low phase); R53 instance: always, under
            //      the compiler rule (clock low phase >= 5 cycles).
            if (MAIN == 0 || !(T[1:0] == EDGE && T[7] && T[12]))
                a_e_r53_ltct: assert (!(w_tick_lt_o && w_tick_ct_o));
        end

        // ---- 6. Promotion is exactly PROMOTE(p) (composition: field.md 1)
        always @* if (MAIN == 1 && ok1 && p_run && !p_abort && q_pk_v) begin
            a_p6_prom_exact: assert (!park_full_o == ((!q_fcuv || q_fcudone) &&
                                                      !(srcdep(q_pk_op, q_pk_ctl) && q_fcuv)));
            a_p6_first: assert (w_field_fld_first_o == (!park_full_o && q_pk_op == OP_FIELD));
        end

        // ------------------------------------------------------------ L
        // lemmas: checker shadows equal the sub-block state they mirror
        wire [5:0] fcu_n    = {1'b0, w_field_cu_ctl[25:21]} + 6'd1;
        wire [5:0] fcu_nw   = (fcu_n > 6'd16) ? 6'd16 : fcu_n;
        wire       fcu_seek = w_field_cu_ctl[17:14] != S_IMM && w_field_cu_ctl[8] &&
                              (w_field_cu_ctl[27:26] == K_SAMPLE || w_field_cu_ctl[27:26] == K_EXPECT);
        wire [1:0] fcu_kind = w_field_cu_ctl[27:26];
        wire       fcu_bt   = T[1:0] == BAUD || (T[1:0] == PULSE && (fcu_kind == K_DRIVE || fcu_kind == K_EXCH));
        wire       outst    = w_tick_bt_run_o && !w_tick_bt_ph;   // timer phase 0 (incl. honour wait)
        always @* if (rst_n && MAIN == 1) begin
            // checker shadows = the state they mirror
            l_sh_tick: assert (sh_tick == w_cfg_tick_o);
            l_sh_code: assert (sh_code == w_cfg_code_o);
            l_sh_dly:  assert (sh_dly == w_cfg_dly_o);
            l_sh_stuff:assert (sh_stuff == w_cfg_stuff_o);
            l_pk_v:   assert (w_field_pk_v_o == c_pk_v);
            if (c_pk_v) l_pk: assert (w_field_pk_op == c_pk_op && w_field_pk_ctl == c_pk_ctl);
            if (w_field_cu_v_o) l_cu: assert (c_cu_v && w_field_cu_op == c_cu_op && w_field_cu_ctl == c_cu_ctl);
            if (c_cu_v && hasres(c_cu_op, c_cu_ctl)) l_cu_res: assert (w_field_cu_v_o);
            if (c_lb && w_field_cu_v_o) l_lb: assert (isbf(c_cu_op, c_cu_ctl, sh_mode));
            l_res:    assert (w_field_res_v == res_valid_o);
            // ruling 42: a BAUD sample is pending only for a current sampling
            // field; the operation timer runs only for a current op
            if (w_tick_spend_o) l_spend: assert (w_field_cu_v_o && w_field_cu_smp_o);
            if (w_tick_ov)      l_ov:    assert (w_field_cu_v_o);
            // SR holds no bits above the field width while a FIELD is current
            if (w_field_cu_v_o && w_field_cu_op == OP_FIELD) begin
                if (!fcu_seek) l_srmask: assert ((w_field_sr_o & ~maskn(fcu_n)) == 32'd0);
                else           l_srmask_seek: assert ((w_field_sr_o[15:0] & ~maskn(fcu_nw)) == 16'd0 &&
                                                      (w_field_sr_o[31:16] & ~maskn(fcu_nw)) == 16'd0);
            end
            // the stretch flop is set only under EDGE stretch_hold; the BAUD
            // hold flag only while the timer runs
            if (w_tick_stretch) l_nostr: assert (T[1:0] == EDGE && T[7] && T[12]);
            if (w_tick_lr)      l_lr:    assert (w_tick_bt_run_o);
            if (w_tick_srel)    l_srel:  assert (w_tick_stretch);
            // the TICK-write idle request (twq) never meets a current op
            if (w_tick_twq) l_twq: assert (!w_field_cu_v_o && !w_tick_bt_run_o);
            // a freshly promoted op starts with clean field state
            if (w_field_first) l_first: assert (w_field_cu_v_o && !w_field_lnch && w_field_nl == 6'd0 &&
                                                w_field_bc == 6'd0 && !w_field_hold && !w_field_tail &&
                                                !w_field_eff);
            // BAUD timer / field bookkeeping (ruling 42): a launched, not yet
            // captured bit (timer phase 0) belongs to the current BAUD-timed
            // field; launches = data bits + outstanding + pending sample
            if (outst) l_outst: assert (w_field_cu_v_o && w_field_cu_op == OP_FIELD && fcu_bt);
            if (w_field_cu_v_o && w_field_cu_op == OP_FIELD) begin
                l_nlflags: assert (w_field_nlt == (w_field_nl < fcu_n) && w_field_nleq == (w_field_nl == fcu_n));
                if (fcu_bt) begin
                    l_nl_le_n: assert (w_field_nl <= fcu_n && w_field_bc <= fcu_n);
                    if (fcu_kind == K_SAMPLE || fcu_kind == K_EXPECT)
                        l_cnt_rx: assert ({1'b0, w_field_nl} == {1'b0, w_field_bc} + {6'd0, outst} +
                                                                 {6'd0, w_tick_spend_o});
                    else begin
                        l_lnch:   assert (w_field_lnch == outst);
                        l_cnt_tx: assert ({1'b0, w_field_nl} == {1'b0, w_field_bc} +
                                          {6'd0, w_field_lnch && !w_field_lstf} +
                                          {6'd0, w_tick_spend_o && !w_field_pstf});
                    end
                end
            end
            // a held seek keeps its matched window {16'h0, WIN} in SR (field.md 6.2)
            if (w_field_cu_v_o && w_field_cu_op == OP_FIELD && fcu_seek && w_field_hold)
                l_seek_hold: assert (w_field_sr_o[31:16] == 16'd0);
            // a held RAW SAMPLE keeps its value {16'h0, d16} in SR (field.md 6.2)
            if (w_field_cu_v_o && w_field_cu_op == OP_RAW && !w_field_cu_ctl[27] && w_field_hold)
                l_raw_hold: assert (w_field_sr_o[31:16] == 16'd0);
            // without a clock delay, no clock change is pending once a BAUD
            // field has made its first (undelayed) clock request
            if (!clk_dly && sh_clkout && c_lb && w_field_cu_v_o && !w_field_fld_first_o)
                l_p10_dly: assert (!(w_dly_pv && w_dly_pclk));
            // 10: after its timer stopped, a current BAUD field leaves out3
            // at the stop level (clock output on, not delayed)
            if (c_lb && sh_clkout && !clk_dly && w_field_cu_v_o && !w_field_fld_first_o && !w_tick_bt_run_o)
                l_p10: assert (w_pins_val[3] == map_stop[1] && w_pins_oe[3] == map_stop[0]);
        end

        // ------------------------------------------------------------ R53 lemmas
        // R53 instance only: the invariant behind ruling 53. Under EDGE
        // stretch_hold the lane pulls the clock line low only while it
        // stretches, a stretch starts only inside a line low phase that the
        // master began at least 4 cycles earlier (so it is >= 5 long), and
        // while the lane stretches the line, and so its view of the clock,
        // stays low: no capture edge can meet the stretched launch tick.
        wire [1:0] r_c   = sh_tick[5:4];
        wire       lo_r  = w_pins_oe[3] && !w_pins_val[3];      // registered pull-down
`ifndef LANE_R53_NO_LEMMAS
        always @* if (MAIN == 0 && rst_n) begin
            l_r53_sh:    assert (sh_tick == w_cfg_tick_o);
            l_r53_pipe1: assert (r1 == (m1 && !lo1) && r2 == (m2 && !lo2));
            l_r53_pipe2: assert (r3 == (m3 && !lo3) && r4 == (m4 && !lo4));
            l_r53_lo:    assert (lane_lo == (lo_r && run_i));
            if (!edge_hold) l_r53_nostr: assert (!w_tick_stretch);
            if (tw_age <= 3'd4) l_r53_xph: assert (xph == tw_age && !data_wait_i);
            if (edge_hold && tw_age >= 3'd2) l_r53_inq: assert (w_tick_in_q_i[r_c] == r4);
            if (edge_hold && w_tick_stretch)
                l_r53_hold: assert (lo_r && !r1 && !r2 && !r3 && !w_tick_twq);
            if (edge_hold && tw_age >= 3'd2 && !w_tick_stretch)
                l_r53_rel:  assert (!lo_r);
            if (edge_hold && tw_age >= 3'd5 && !w_tick_stretch && !r3 && r4)
                l_r53_el:   assert (!lo1 && !lo2 && !lo3 && !lo4);
        end
`endif

        // ------------------------------------------------------------ covers
        // lane.md 9.15 and the white-box features
        logic q_tstr;
        always @(posedge clk) q_tstr <= w_tick_stretch;
        logic q_hold, q_prevfld;
        logic [1:0] q_gf;          // FIELD starts while the BAUD timer runs uninterrupted
        always_ff @(posedge clk or negedge rst_n)
            if (!rst_n) begin q_hold <= 1'b0; q_prevfld <= 1'b0; q_gf <= 2'd0; end
            else begin
                q_hold <= w_field_hold_o;
                if (w_field_cu_v_o) q_prevfld <= w_field_cu_fld_o;
                if (!w_tick_bt_run_o && !w_tick_lt_o) q_gf <= 2'd0;
                else if (w_field_fld_first_o && w_tick_lt_o && !w_tick_lr && q_gf != 2'd3) q_gf <= q_gf + 2'd1;
            end
        always @* if (rst_n) begin
            // gap-free back-to-back fields: BAUD (second field launches at the
            // first one's end of bit, the timer never stopped nor held)
            c_15_gapfree_baud:  cover (T[1:0] == BAUD && q_gf == 2'd1 && w_field_fld_first_o &&
                                       w_tick_lt_o && !w_tick_lr && w_tick_bt_run_o);
            // EDGE / RECOVER: the next FIELD is current in the cycle after the
            // previous FIELD's completion, then launches at its launch tick
            c_15_gapfree_edge:  cover (ok1 && T[1:0] == EDGE && q_fcudone && q_prevfld &&
                                       w_field_fld_first_o);
            c_15_gapfree_rec:   cover (ok1 && T[1:0] == RECOVER && q_fcudone && q_prevfld &&
                                       w_field_fld_first_o);
            c_15_timer_hold:    cover (w_tick_lr && w_tick_bt_run_o);
            c_15_overrun:       cover (w_field_hold_o);
            c_15_overrun_rel:   cover (ok1 && q_hold && !w_field_hold_o && ev_load);
            c_15_abort_both:    cover (abort_i && w_field_cu_v_o && w_field_pk_v_o);
            c_15_srcdep:        cover (ok1 && q_pk_v && p_run && !p_abort && q_fcudone &&
                                       srcdep(q_pk_op, q_pk_ctl) && park_full_o);
            c_15_pin_eob:       cover (w_field_pin_we_o && w_tick_bt_stop_o && q_prevfld);
            c_15_halt_field:    cover (!run_i && w_field_cu_fld_o);
            // ruling 42: a BAUD sample pending and taken
            c_r42_spend:        cover (w_tick_spend_o && w_tick_st_o);
            // hazard (reported): with the clock delayed, a PIN that writes out3
            // in the cycle the BAUD timer makes its delayed end-of-bit request
            if (MAIN == 1)
            c_haz_pin_dlyclk:   cover (w_field_pin_we_o && w_field_pin_mask_o[3] && w_tick_creq_o &&
                                       w_tick_creq_dly_o && dmode);
            // reading R-D2 of formal/tick (delay env_cfg): a SETM of CODE or
            // THR2L lands while a delayed change is pending. Reachable in a
            // legal program: the delay unit's cfg_ assumption is NOT a lane
            // guarantee (reported; not asserted).
            c_obl_dly_cfg:      cover (w_dly_pv && wlo && (cfg_reg_i == 5'd4 || cfg_reg_i == 5'd5));
            // ruling 53 without the compiler rule (main instance, free in_i):
            // launch and capture tick coincide under EDGE stretch_hold
            if (MAIN == 1) c_r53_coincide: cover (w_tick_lt_o && w_tick_ct_o);
            if (MAIN == 0) begin
                c_r53_stretch:  cover (w_tick_stretch && w_field_fld_first_o);
                c_r53_release:  cover (ok1 && !w_tick_stretch && q_tstr && w_tick_ct_o == 1'b0 && run_i);
                c_r53_low5:     cover (f_pv && mclk_i && !p_mclk && mlow == 3'd5 && w_tick_stretch);
                // assumptions of the R53 instance are satisfiable together with
                // the behaviour they restrict: a PIN under EDGE stretch_hold
                // (not on out3), a halt and restart, data_wait in a D cycle
                c_r53_pin:      cover (park_we_i && park_op_i == OP_PIN && edge_hold && park_ctl_i[24]);
                c_r53_restart:  cover (f_pv && run_i && !p_run && edge_hold);
                c_r53_dwait:    cover (f_pv && data_wait_i && !p_dwait && edge_hold);
            end
        end
    end endgenerate

    // =====================================================================
    // Covers
    // =====================================================================
    // assumptions
    always @* if (rst_n) begin
        c_u_park:     cover (park_we_i && run_i);
        c_u_ack:      cover (res_ack_i && res_valid_o);
        c_u_abort_clr:cover (abort_i && ev_clr_i != 4'd0);
        c_u_idle_cfg: cover (cfg_we_i && cfg_reg_i == 5'd9 && !busy_o && !park_full_o);
        c_u_idle_cmd: cover (cmd_i == 2'd2 && !busy_o && !park_full_o);
        c_u_evc_busy: cover (cfg_we_i && cfg_reg_i == 5'd13 && busy_o);
        c_u_op_pin:   cover (park_we_i && park_op_i == OP_PIN);
        c_u_seek:     cover (seek_park && sh_mode != BAUD);
        c_u_dwait:    cover (ok1 && data_wait_i && !p_wrtick && sh_mode == EDGE);
        c_u_halt:     cover (ok1 && p_run && !run_i);
    end
    // lane.md 9 features, on the ports
    always @* if (rst_n) begin
        c_p1_halt_oe:   cover (ok1 && !run_i && p_run && p_out_oe != 4'd0);
        c_p2_start:     cover (!q_ever && run_i);
        c_p4_idle_ack:  cover (ok1 && !p_busy && !p_pf && !p_park_we && p_run && p_rv && p_ack);
        c_p5_idle_prom: cover (ok1 && ev_prom && !p_busy);
        c_p5_busy_prom: cover (ok1 && ev_prom && p_busy);
        c_p7_load:      cover (ok1 && ev_load);
        c_p7_load_ack:  cover (ok1 && ev_load && p_rv && p_ack);     // 9.15: in the ack cycle
        c_p7_drive_tag: cover (ok1 && ev_load && cu_drv_known && cu_tagd);
        c_p7_raw:       cover (ok1 && ev_load && q_cu_op == OP_RAW);
        c_p8_r0:        cover (dst_busy_o[0] && !res_valid_o);
        c_p8_mem:       cover (dst_busy_o[3] && res_valid_o && res_dst_o == D_MEM);
        c_p8_all:       cover (dst_busy_o == 4'b0111);
        c_p9_crcchk:    cover (ok1 && m_o != p_m && p_cmd == 2'd2);
        c_p9_expect:    cover (ok1 && m_o != p_m && p_cmd != 2'd2);
        c_p10_stop:     cover (ok1 && p_busy && !busy_o && p_run && !p_abort && q_lb && sh_clkout &&
                               !clk_dly && out_val_o[3] != p_out_val[3]);
        c_p11_abort:    cover (ok1 && ev_abort && p_pf && p_busy);
        c_p11_release:  cover (ok1 && ev_abort && p_edge_hold && out_oe_o[3] != p_out_oe[3]);
        c_p12_enable:   cover (q_E && out_oe_o != 4'd0);
        c_p13_rise:     cover (ok1 && ev_pend_o != 4'd0 && p_ev_pend == 4'd0);
        c_p13_clear:    cover (ok1 && ev_pend_o == 4'd0 && p_ev_pend != 4'd0 && p_run);
        // a complete operation (FIELD with a result) in each tick mode
        c_op_baud:      cover (ok1 && ev_load && q_cu_op == OP_FIELD && sh_mode == BAUD);
        c_op_edge:      cover (ok1 && ev_load && q_cu_op == OP_FIELD && sh_mode == EDGE);
        c_op_recover:   cover (ok1 && ev_load && q_cu_op == OP_FIELD && sh_mode == RECOVER);
        c_op_pulse:     cover (ok1 && ev_load && q_cu_op == OP_FIELD && sh_mode == PULSE);
    end
endmodule
