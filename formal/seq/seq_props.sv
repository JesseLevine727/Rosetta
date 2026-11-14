`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_seq (shared sequencer).
//
//  Written from docs/blocks/sequencer.md as amended by docs/decisions.md
//  (rulings 1, 8-13, 16, 21, 26, 29, 30) and the lane side in
//  docs/blocks/lane.md 2 and 7. The RTL was not read. Every property is
//  about the block's ports (the observation ports of sequencer.md 2.7 carry
//  the architectural state).
//
//  Method (riscv-formal style, symbolic lane)
//   * L is a symbolic lane (anyconst, 0-3). Every lane-level property is
//     about L, so the proof covers each lane; the other lanes' inputs are
//     completely free (their environment rules are not assumed), which
//     shows that lane L's correctness does not depend on them.
//   * Exact transition model of lane L's architectural state, computed in
//     every cycle from the CURRENT observed state (obs_*, run_o, attn_o,
//     lane_data_wait_o), the instruction register model IR (= pm_rdata_i
//     delayed by one cycle, sequencer.md 4.2 / 9) and the inputs:
//       - in L's D cycle: the slot action (NONE, DELIVER, VECTOR, EXEC of
//         every instruction, waits, illegal-code halt) of sections 4.3, 5;
//       - in L's X cycle: the end-of-X writes (ALU, LOOP, POP, MEM load)
//         from a snapshot taken in the D cycle;
//       - host register writes (5.9);
//       - otherwise nothing changes.
//     The expected next state is registered and compared with the observed
//     state in the next cycle. The D-cycle memory/channel outputs and the
//     X-cycle lane strobes and shared buses are compared exactly as well.
//     Because the model starts from the observed state each cycle, it holds
//     no copy of the architectural state, and the comparison is 1-step
//     inductive on the checker side.
//   * The comparisons are split per slot-action class (one assertion group
//     per instruction class, `CLS below): aO_<cls> D-cycle outputs,
//     aD_<cls> state after the D cycle, aX_<cls> X-cycle strobes and buses,
//     aW_<cls> state after the X cycle. Field-wise assertions a_nx_* give
//     the same comparison per state field, for diagnosis.
//   * Port-level statements named after sequencer.md 9 (a_p1_* ... a_p20_*)
//     are written directly on the ports, mostly without the model.
//   * Environment (env_*): the lane guarantees of lane.md that the proof
//     needs, for lane L only, the channel frame of channels.md 4.4, the
//     host's ph = 1 rule (host.md 7.1) and reset in the first cycle.
//     Assumed with STANDALONE = 1, asserted with STANDALONE = 0.
//   * Memories are free inputs: pm_rdata_i, dm_rdata_i, qe_i, qf_i are
//     unconstrained, which includes every consistent memory (symbolic-memory
//     abstraction with no constraint at all); the properties are relative to
//     the values on these ports in the cycles the spec names.
//
//  Readings of the spec (see the report): RAW SAMPLE takes s = 15 before the
//  illegal-source check; a RAW (either direction) with destination 6, 12,
//  13, 14 halts; the WAIT timeout start is a commit for qwait (no data
//  condition) but keeps PC; qwait after a wait is "waited && the row's data
//  condition failed", whatever made it wait; lane_sr_i for MOVSR/MOVSRH is
//  the D-cycle value (equal to the X-cycle value under env_sr).
//
//  Assertions inside generate loops carry their name as a trailing comment.
// ===========================================================================

module seq_props #(
    // 1: stand-alone proof (environment assumed, covers active).
    // 0: monitor inside a larger proof (environment asserted, covers off).
    parameter STANDALONE = 1
) (
    input wire          clk,
    input wire          rst_n,
    input wire [4:0]    ph_o,
    input wire [8:0]    pm_addr_o,
    input wire [31:0]   pm_rdata_i,
    input wire [2:0]    dm_op_o,
    input wire [6:0]    dm_off_o,
    input wire [1:0]    dm_hmask_o,
    input wire [31:0]   dm_wdata_o,
    input wire [31:0]   dm_rdata_i,
    input wire [3:0]    qe_i,
    input wire [3:0]    qf_i,
    input wire          ch_put_o,
    input wire          ch_get_o,
    input wire [1:0]    ch_idx_o,
    input wire [31:0]   ch_wdata_o,
    input wire [3:0]    ch_valid_i,
    input wire [127:0]  ch_data_i,
    input wire [31:0]   ts_i,
    input wire          hreg_we_i,
    input wire [6:0]    hreg_addr_i,
    input wire [31:0]   hreg_wdata_i,
    input wire [31:0]   hreg_rdata_o,
    input wire [3:0]    run_o,
    input wire [3:0]    attn_o,
    input wire [3:0]    lane_run_o,
    input wire [3:0]    lane_park_we_o,
    input wire [1:0]    lane_park_op_o,
    input wire [27:0]   lane_park_ctl_o,
    input wire [31:0]   lane_park_data_o,
    input wire [3:0]    lane_res_ack_o,
    input wire [3:0]    lane_abort_o,
    input wire [15:0]   lane_ev_clr_o,
    input wire [3:0]    lane_cfg_we_o,
    input wire [4:0]    lane_cfg_reg_o,
    input wire          lane_cfg_hi_o,
    input wire [15:0]   lane_cfg_data_o,
    input wire [7:0]    lane_cmd_o,
    input wire [31:0]   lane_cmd_data_o,
    input wire [3:0]    lane_data_wait_o,
    input wire [3:0]    lane_busy_i,
    input wire [3:0]    lane_park_full_i,
    input wire [3:0]    lane_res_valid_i,
    input wire [15:0]   lane_res_dst_i,
    input wire [127:0]  lane_res_data_i,
    input wire [63:0]   lane_res_ts_i,
    input wire [15:0]   lane_dst_busy_i,
    input wire [3:0]    lane_m_i,
    input wire [15:0]   lane_ev_pend_i,
    input wire [127:0]  lane_sr_i,
    input wire [127:0]  lane_crc_i,
    input wire [35:0]   obs_pc_o,
    input wire [255:0]  obs_r_o,
    input wire [3:0]    obs_z_o,
    input wire [3:0]    obs_n_o,
    input wire [3:0]    obs_pause_o,
    input wire [3:0]    obs_step_o,
    input wire [15:0]   obs_armed_o,
    input wire [143:0]  obs_tgt_o,
    input wire [3:0]    obs_wsta_o,
    input wire [3:0]    obs_capw_o
);
    // ---------------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------------
    localparam [2:0] DM_NONE = 3'd0, DM_READ = 3'd1, DM_WRITE = 3'd2,
                     DM_POP  = 3'd3, DM_PUSH = 3'd4;

    // Slot-action classes
    localparam [5:0] C_NONE    = 6'd0,   // not eligible
                     C_DEL_R   = 6'd1,   // deliver to R0-R3
                     C_DEL_HQ  = 6'd2,   // deliver to HOSTQ
                     C_DEL_CAP0= 6'd3,   // CAP, timestamp word
                     C_DEL_CAP1= 6'd4,   // CAP, value word
                     C_DEL_MEM = 6'd5,   // deliver to MEM
                     C_DEL_CH  = 6'd6,   // deliver to C0-C3
                     C_DEL_OTH = 6'd7,   // deliver, discarded
                     C_DEL_WT  = 6'd8,   // delivery waits
                     C_VEC     = 6'd9,   // VECTOR
                     C_ILL     = 6'd10,  // undefined code: halt
                     C_EX_WT   = 6'd11,  // EXEC waits
                     C_FIELD   = 6'd12,
                     C_RAW     = 6'd13,
                     C_PIN     = 6'd14,
                     C_SETM    = 6'd15,
                     C_LDI     = 6'd16,
                     C_MEM_LD  = 6'd17,
                     C_MEM_ST  = 6'd18,
                     C_BR      = 6'd19,
                     C_LOOP    = 6'd20,
                     C_WT_HIT  = 6'd21,  // WAIT ends by event
                     C_WT_TO   = 6'd22,  // WAIT ends by timeout
                     C_WT_ST   = 6'd23,  // WAIT starts its timeout
                     C_EVT     = 6'd24,
                     C_QPOP    = 6'd25,
                     C_QPUSH   = 6'd26,  // PUSH, PUSHTS, PUSHCRC, PUSHI
                     C_QPUT    = 6'd27,  // PUT, PUTSR
                     C_QGET    = 6'd28,
                     C_NOP     = 6'd29,  // NOP, IRQ
                     C_HALT    = 6'd30,
                     C_CMD     = 6'd31,  // CRCINIT, CRCCHK, STUFFCLR
                     C_ALU     = 6'd32;  // + ALU op (0-13): 32..45

    // ---------------------------------------------------------------------
    // Symbolic lane and the checker's own wheel
    // ---------------------------------------------------------------------
    (* anyconst *) logic [1:0] L;

    logic [2:0] m_ph;                      // phase model, 0-4
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) m_ph <= 3'd0;
        else        m_ph <= (m_ph == 3'd4) ? 3'd0 : m_ph + 3'd1;
    wire [4:0] m_oh = 5'b00001 << m_ph;

    // Phases of lane L: F = L, I = L+1, D = (L+2) mod 5, X = (L+3) mod 5
    wire [2:0] phD = (L == 2'd3) ? 3'd0 : {1'b0, L} + 3'd2;
    wire [2:0] phX = (L == 2'd2) ? 3'd0 : (L == 2'd3) ? 3'd1 : {1'b0, L} + 3'd3;
    wire isD = rst_n && (m_ph == phD);
    wire isX = rst_n && (m_ph == phX);
    // D owner of the current cycle (sequencer.md 1); host at ph = 1
    wire [1:0] downer = (m_ph == 3'd0) ? 2'd3 : (m_ph == 3'd2) ? 2'd0 :
                        (m_ph == 3'd3) ? 2'd1 : 2'd2;

    // IR model: IR <= pm_rdata_i at the end of every cycle (4.2); reset 0
    logic [31:0] IR;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) IR <= 32'h0;
        else        IR <= pm_rdata_i;

    // ---------------------------------------------------------------------
    // Lane L: observed state and inputs
    // ---------------------------------------------------------------------
    wire [8:0]  pcL   = obs_pc_o[9*L +: 9];
    wire [63:0] rL    = obs_r_o[64*L +: 64];
    wire        zL    = obs_z_o[L];
    wire        nL    = obs_n_o[L];
    wire        pauL  = obs_pause_o[L];
    wire        stpL  = obs_step_o[L];
    wire [3:0]  armL  = obs_armed_o[4*L +: 4];
    wire [35:0] tgtL  = obs_tgt_o[36*L +: 36];
    wire        wsL   = obs_wsta_o[L];
    wire        cwL   = obs_capw_o[L];
    wire        runL  = run_o[L];
    wire        atL   = attn_o[L];
    wire        qwL   = lane_data_wait_o[L];

    wire        busyL = lane_busy_i[L];
    wire        pfL   = lane_park_full_i[L];
    wire        rvL   = lane_res_valid_i[L];
    wire [3:0]  rdstL = lane_res_dst_i[4*L +: 4];
    wire [31:0] rdatL = lane_res_data_i[32*L +: 32];
    wire [15:0] rtsL  = lane_res_ts_i[16*L +: 16];
    wire [3:0]  dbL   = lane_dst_busy_i[4*L +: 4];
    wire        mL    = lane_m_i[L];
    wire [3:0]  evL   = lane_ev_pend_i[4*L +: 4];
    wire [31:0] srL   = lane_sr_i[32*L +: 32];
    wire [31:0] crcL  = lane_crc_i[32*L +: 32];
    wire        qeL   = qe_i[L];
    wire        qfL   = qf_i[L];

    // lane-L strobes
    wire        s_pk   = lane_park_we_o[L];
    wire        s_ack  = lane_res_ack_o[L];
    wire        s_ab   = lane_abort_o[L];
    wire [3:0]  s_evc  = lane_ev_clr_o[4*L +: 4];
    wire        s_cfg  = lane_cfg_we_o[L];
    wire [1:0]  s_cmd  = lane_cmd_o[2*L +: 2];
    wire        s_any  = s_pk || s_ack || s_ab || (s_evc != 4'h0) || s_cfg || (s_cmd != 2'd0);

    // Host register addresses of lane L
    wire [6:0]  A_CTRL = 7'h10 + {2'b00, L, 3'b000};
    wire [6:0]  A_PC   = 7'h11 + {2'b00, L, 3'b000};
    localparam [6:0] A_IRQSTAT = 7'h04;
    wire h_ctrl = hreg_we_i && (hreg_addr_i == A_CTRL);
    wire h_pc   = hreg_we_i && (hreg_addr_i == A_PC);
    wire h_irq  = hreg_we_i && (hreg_addr_i == A_IRQSTAT);
    wire h_any  = h_ctrl || h_pc || h_irq;   // a host write that may touch L

    // ---------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------
    function automatic [15:0] rsel(input [63:0] r, input [1:0] i);
        rsel = r[16*i +: 16];
    endfunction
    function automatic [31:0] tagged(input [31:0] w, input [7:0] tag);
        tagged = (tag != 8'h0) ? {tag, w[23:0]} : w;
    endfunction
    function automatic [3:0] onehot2(input [1:0] i);
        onehot2 = 4'b0001 << i;
    endfunction

    // ---------------------------------------------------------------------
    // D-cycle model of lane L (meaningful when isD)
    // ---------------------------------------------------------------------
    wire        elig  = runL && (!pauL || stpL);
    wire        IDLE  = !busyL && !pfL;
    wire [3:0]  evm   = evL & armL;
    wire [1:0]  kvec  = evm[0] ? 2'd0 : evm[1] ? 2'd1 : evm[2] ? 2'd2 : 2'd3;
    wire [8:0]  PC1   = pcL + 9'd1;

    // instruction fields
    wire [3:0]  op    = IR[31:28];
    wire [3:0]  sub   = IR[27:24];         // ALU/Q/MISC op, WAIT mask
    wire [4:0]  cond  = IR[27:23];         // BR cond, SETM register
    wire        is_fr = (op == 4'd0) || (op == 4'd1);
    wire [3:0]  s     = ((op == 4'd1) && !IR[27]) ? 4'd15 : IR[17:14];
    wire [3:0]  fdst  = IR[13:10];
    wire        s_ch  = (s[3:2] == 2'b10);   // 8-11
    wire        cv_s  = ch_valid_i[s[1:0]];
    wire [1:0]  rd26  = IR[27:26];         // LDI, LOOP rd; EVT k
    wire [1:0]  ard   = IR[23:22];         // ALU rd, MEM ar, Q reg
    wire [1:0]  ars   = IR[21:20];         // ALU rs, Q ch
    wire [1:0]  mdr   = IR[26:25];         // MEM dr
    wire        m_st  = IR[27], m_hi = IR[24], m_pi = IR[21], m_ab = IR[20];
    wire [7:0]  qtag  = IR[7:0];
    wire        cv_q  = ch_valid_i[ars];

    wire illegal =
        (is_fr && ((s == 4'd5) || (s == 4'd14) || (fdst == 4'd6) ||
                   (fdst == 4'd12) || (fdst == 4'd13) || (fdst == 4'd14))) ||
        ((op == 4'd3)  && (cond >= 5'd14)) ||
        ((op == 4'd5)  && (sub  >= 4'd14)) ||
        ((op == 4'd7)  && (cond >= 5'd19)) ||
        ((op == 4'd11) && (sub  >= 4'd8))  ||
        ((op == 4'd12) && (sub  >= 4'd6))  ||
        (op >= 4'd13);

    // USES (table 5.4)
    logic [3:0] uses;
    always @* begin
        uses = 4'h0;
        case (op)
            4'd0, 4'd1: begin
                if (s[3:2] == 2'b00) uses = onehot2(s[1:0]);
                else if (s == 4'd7)  uses = 4'b1000;
            end
            4'd3:  if (IR[21]) uses = onehot2(IR[1:0]);
            4'd4, 4'd8: uses = onehot2(rd26);
            4'd5: begin
                uses = onehot2(ard);
                if (!IR[19] && ((sub <= 4'd8) || (sub == 4'd11))) uses = uses | onehot2(ars);
            end
            4'd6: begin
                uses = onehot2(mdr);
                if (!m_ab) uses = uses | onehot2(ard);
            end
            4'd11: if ((sub == 4'd0) || (sub == 4'd1) || (sub == 4'd4) || (sub == 4'd5)) uses = onehot2(ard);
            4'd12: if (sub == 4'd4) uses = 4'b0011;
            default: uses = 4'h0;
        endcase
    end
    wire HAZ = (uses & dbL) != 4'h0;

    // branch condition
    logic taken;
    always @* begin
        case (cond)
            5'd0:  taken = 1'b1;
            5'd1:  taken = zL;
            5'd2:  taken = !zL;
            5'd3:  taken = nL;
            5'd4:  taken = !nL;
            5'd5:  taken = mL;
            5'd6:  taken = !mL;
            5'd7:  taken = qeL;
            5'd8:  taken = !qeL;
            5'd9:  taken = qfL;
            5'd10: taken = !qfL;
            5'd11: taken = evL[0];
            5'd12: taken = evL[1];
            5'd13: taken = evL[2];
            5'd14: taken = evL[3];
            5'd15: taken = ch_valid_i[0];
            5'd16: taken = ch_valid_i[1];
            5'd17: taken = ch_valid_i[2];
            5'd18: taken = ch_valid_i[3];
            default: taken = 1'b0;
        endcase
    end

    // ALU (5.8)
    wire [15:0] aA = rsel(rL, ard);
    wire [15:0] aB = IR[19] ? IR[15:0] : rsel(rL, ars);
    wire [16:0] aS = {1'b0, aA} + {1'b0, aB};
    logic [15:0] ay;
    always @* begin
        case (sub)
            4'd0:  ay = aB;
            4'd1:  ay = aA + aB;
            4'd2, 4'd3: ay = aA - aB;
            4'd4:  ay = aA & aB;
            4'd5:  ay = aA | aB;
            4'd6:  ay = aA ^ aB;
            4'd7:  ay = (aB[4:0] >= 5'd16) ? 16'h0 : (aA << aB[3:0]);
            4'd8:  ay = (aB[4:0] >= 5'd16) ? 16'h0 : (aA >> aB[3:0]);
            4'd9:  ay = aA + 16'd1;
            4'd10: ay = aA - 16'd1;
            4'd11: ay = aS[15:0] + {15'h0, aS[16]};
            4'd12: ay = srL[15:0];
            4'd13: ay = srL[31:16];
            default: ay = 16'h0;
        endcase
    end
    wire an = ((sub == 4'd2) || (sub == 4'd3)) ? (aA < aB) : ay[15];

    // WAIT (5.5)
    wire        w_hit = (evL & sub) != 4'h0;
    wire [11:0] w_T   = IR[11:0];

    // CAP timestamp word (ruling 30)
    wire [31:0] tsword = {ts_i[31:16] - {15'h0, (ts_i[15:0] < rtsL)}, rtsL};

    // wait condition and data condition of the EXEC row
    logic row_wait, dc_fail;
    always @* begin
        row_wait = 1'b0; dc_fail = 1'b0;
        case (op)
            4'd0, 4'd1: begin
                dc_fail  = ((s == 4'd4) && qeL) || (s_ch && !cv_s);
                row_wait = HAZ || dc_fail;
            end
            4'd3:  row_wait = ((cond <= 5'd9) && !IDLE) || (IR[21] && HAZ);
            4'd4, 4'd6, 4'd8: row_wait = HAZ;
            4'd5:  row_wait = HAZ || (((sub == 4'd12) || (sub == 4'd13)) && !IDLE);
            4'd7:  row_wait = ((cond == 5'd5) || (cond == 5'd6)) && !IDLE;
            4'd9:  row_wait = !(w_hit || (wsL && IDLE) || (!wsL && (w_T != 12'h0) && IDLE));
            4'd11: case (sub)
                4'd0: begin dc_fail = qeL;   row_wait = qeL || HAZ;   end
                4'd1: begin dc_fail = qfL;   row_wait = qfL || HAZ;   end
                4'd2: begin dc_fail = qfL;   row_wait = qfL;          end
                4'd3: begin dc_fail = qfL;   row_wait = qfL || !IDLE; end
                4'd4: begin dc_fail = cv_q;  row_wait = cv_q || HAZ;  end
                4'd5: begin dc_fail = !cv_q; row_wait = !cv_q || HAZ; end
                4'd6: begin dc_fail = cv_q;  row_wait = cv_q || !IDLE; end
                4'd7: begin dc_fail = qfL;   row_wait = qfL;          end
                default: ;
            endcase
            4'd12: case (sub)
                4'd1, 4'd3, 4'd5: row_wait = !IDLE;
                4'd4: row_wait = !IDLE || HAZ;
                default: ;
            endcase
            default: ;
        endcase
    end
    // general run-ahead wait (5.2): every instruction waits while parked
    // full, except WAIT with wsta = 1
    wire gen_wait = pfL && !((op == 4'd9) && wsL);
    wire ex_wait  = gen_wait || row_wait;

    // deliveries (5.1)
    wire       d_isch  = (rdstL[3:2] == 2'b10);
    wire       del_wait = (((rdstL == 4'd4) || (rdstL == 4'd5)) && qfL) ||
                          (d_isch && ch_valid_i[rdstL[1:0]]);

    // ---- the D-cycle function
    logic [5:0]  e_cls;
    logic [2:0]  e_dmop;  logic [6:0] e_dmoff; logic [1:0] e_dmhm; logic [31:0] e_dmwd;
    logic        e_put, e_get; logic [1:0] e_idx; logic [31:0] e_cwd;
    logic [8:0]  e_pc;    logic [63:0] e_r;    logic [3:0] e_arm; logic [35:0] e_tgt;
    logic        e_ws, e_cw, e_run, e_at, e_qw, e_stp;
    logic        e_pk;    logic [1:0] e_pkop; logic [27:0] e_pkctl; logic e_pkmem; logic [31:0] e_pkdat;
    logic        e_ack, e_ab; logic [3:0] e_evc;
    logic        e_cfg;   logic [4:0] e_creg; logic e_chi; logic [15:0] e_cdat;
    logic [1:0]  e_cmd;   logic [31:0] e_cmdd;
    logic        e_xw;    logic [1:0] e_xreg; logic [1:0] e_xsrc; logic [15:0] e_xval;
    logic        e_fen, e_z, e_n;

    always @* begin
        e_cls = C_NONE;
        e_dmop = DM_NONE; e_dmoff = 7'h0; e_dmhm = 2'b00; e_dmwd = 32'h0;
        e_put = 1'b0; e_get = 1'b0; e_idx = 2'd0; e_cwd = 32'h0;
        e_pc = pcL; e_r = rL; e_arm = armL; e_tgt = tgtL;
        e_ws = wsL; e_cw = cwL; e_run = runL; e_at = atL; e_qw = qwL;
        e_stp = elig ? 1'b0 : stpL;            // 4.3
        e_pk = 1'b0; e_pkop = 2'd0; e_pkctl = 28'h0; e_pkmem = 1'b0; e_pkdat = 32'h0;
        e_ack = 1'b0; e_ab = 1'b0; e_evc = 4'h0;
        e_cfg = 1'b0; e_creg = 5'h0; e_chi = 1'b0; e_cdat = 16'h0;
        e_cmd = 2'd0; e_cmdd = 32'h0;
        e_xw = 1'b0; e_xreg = 2'd0; e_xsrc = 2'd0; e_xval = 16'h0;
        e_fen = 1'b0; e_z = 1'b0; e_n = 1'b0;

        if (!elig) begin
            e_cls = C_NONE;                                    // 4.3 row 1
        end else if (rvL) begin                                // DELIVER 5.1
            e_qw = del_wait;                                   // 5.3
            if (del_wait) begin
                e_cls = C_DEL_WT;
            end else if (rdstL[3:2] == 2'b00) begin
                e_cls = C_DEL_R;
                e_r[16*rdstL[1:0] +: 16] = rdatL[15:0];
                e_ack = 1'b1;
            end else if (rdstL == 4'd4) begin
                e_cls = C_DEL_HQ;
                e_dmop = DM_PUSH; e_dmwd = rdatL; e_ack = 1'b1;
            end else if (rdstL == 4'd5 && !cwL) begin
                e_cls = C_DEL_CAP0;
                e_dmop = DM_PUSH; e_dmwd = tsword; e_cw = 1'b1;
            end else if (rdstL == 4'd5) begin
                e_cls = C_DEL_CAP1;
                e_dmop = DM_PUSH; e_dmwd = rdatL; e_cw = 1'b0; e_ack = 1'b1;
            end else if (rdstL == 4'd7) begin
                e_cls = C_DEL_MEM;
                e_dmop = DM_WRITE; e_dmoff = rsel(rL, 2'd3) & 7'h7f; e_dmhm = 2'b11; e_dmwd = rdatL;
                e_r[48 +: 16] = rsel(rL, 2'd3) + 16'd1;
                e_ack = 1'b1;
            end else if (d_isch) begin
                e_cls = C_DEL_CH;
                e_put = 1'b1; e_idx = rdstL[1:0]; e_cwd = rdatL; e_ack = 1'b1;
            end else begin
                e_cls = C_DEL_OTH;
                e_ack = 1'b1;
            end
        end else if (evm != 4'h0) begin                        // VECTOR 5.6
            e_cls = C_VEC;
            e_pc = tgtL[9*kvec +: 9];
            e_ws = 1'b0; e_cw = 1'b0; e_qw = 1'b0;
            e_ab = 1'b1; e_evc = onehot2(kvec);
        end else if (illegal) begin                            // 5.7, ruling 13
            e_cls = C_ILL;
            e_run = 1'b0; e_at = 1'b1; e_arm = 4'h0; e_ws = 1'b0; e_cw = 1'b0; e_qw = 1'b0;
        end else if (ex_wait) begin                            // 5.3
            e_cls = C_EX_WT;
            e_qw  = dc_fail;
        end else begin                                         // EXEC commit 5.2
            e_qw = 1'b0;
            e_pc = PC1;
            case (op)
                4'd0, 4'd1: begin
                    e_cls = (op == 4'd0) ? C_FIELD : C_RAW;
                    if (s == 4'd4) e_dmop = DM_POP;
                    if (s == 4'd7) begin
                        e_dmop = DM_READ; e_dmoff = rsel(rL, 2'd3) & 7'h7f;
                        e_r[48 +: 16] = rsel(rL, 2'd3) + 16'd1;
                    end
                    if (s_ch) begin e_get = 1'b1; e_idx = s[1:0]; end
                    e_pk = 1'b1; e_pkop = {1'b0, op[0]}; e_pkctl = IR[27:0];
                    e_pkmem = (s == 4'd4) || (s == 4'd7);
                    if (s[3:2] == 2'b00)  e_pkdat = {16'h0, rsel(rL, s[1:0])};
                    else if (s_ch)        e_pkdat = ch_data_i[32*s[1:0] +: 32];
                    else if (s == 4'd12)  e_pkdat = {23'h0, IR[8:0]};
                    else                  e_pkdat = 32'h0;
                end
                4'd2: begin
                    e_cls = C_PIN;
                    e_pk = 1'b1; e_pkop = 2'd2; e_pkctl = IR[27:0]; e_pkdat = 32'h0;
                end
                4'd3: begin
                    e_cls = C_SETM;
                    e_cfg = 1'b1; e_creg = cond; e_chi = IR[22];
                    e_cdat = IR[21] ? rsel(rL, IR[1:0]) : IR[15:0];
                end
                4'd4: begin
                    e_cls = C_LDI;
                    e_r[16*rd26 +: 16] = IR[15:0];
                end
                4'd5: begin
                    e_cls = C_ALU + {2'b00, sub};
                    e_xw = (sub != 4'd3); e_xreg = ard; e_xsrc = 2'd0; e_xval = ay;
                    e_fen = 1'b1; e_z = (ay == 16'h0); e_n = an;
                end
                4'd6: begin
                    e_cls = m_st ? C_MEM_ST : C_MEM_LD;
                    e_dmoff = m_ab ? IR[6:0] : (rsel(rL, ard) & 7'h7f);
                    if (m_st) begin
                        e_dmop = DM_WRITE; e_dmhm = m_hi ? 2'b10 : 2'b01;
                        e_dmwd = {rsel(rL, mdr), rsel(rL, mdr)};
                    end else begin
                        e_dmop = DM_READ;
                        e_xw = 1'b1; e_xreg = mdr; e_xsrc = m_hi ? 2'd2 : 2'd1;
                    end
                    if (!m_ab && m_pi) e_r[16*ard +: 16] = rsel(rL, ard) + 16'd1;
                end
                4'd7: begin
                    e_cls = C_BR;
                    e_pc = taken ? IR[8:0] : PC1;
                end
                4'd8: begin
                    e_cls = C_LOOP;
                    e_pc = (rsel(rL, rd26) != 16'd1) ? IR[8:0] : PC1;
                    e_xw = 1'b1; e_xreg = rd26; e_xsrc = 2'd0; e_xval = rsel(rL, rd26) - 16'd1;
                end
                4'd9: begin
                    e_ws = 1'b0;
                    if (w_hit) begin
                        e_cls = C_WT_HIT;
                        e_ab  = wsL;
                    end else if (wsL) begin
                        e_cls = C_WT_TO;
                    end else begin
                        e_cls = C_WT_ST;
                        e_pc = pcL; e_ws = 1'b1;
                        e_pk = 1'b1; e_pkop = 2'd2; e_pkctl = {16'h0, w_T}; e_pkdat = 32'h0;
                    end
                end
                4'd10: begin
                    e_cls = C_EVT;
                    e_arm[rd26] = IR[25];
                    e_tgt[9*rd26 +: 9] = IR[8:0];
                    e_evc = IR[24] ? onehot2(rd26) : 4'h0;
                end
                4'd11: case (sub)
                    4'd0: begin
                        e_cls = C_QPOP; e_dmop = DM_POP;
                        e_xw = 1'b1; e_xreg = ard; e_xsrc = 2'd1;
                    end
                    4'd1: begin e_cls = C_QPUSH; e_dmop = DM_PUSH; e_dmwd = tagged({16'h0, rsel(rL, ard)}, qtag); end
                    4'd2: begin e_cls = C_QPUSH; e_dmop = DM_PUSH; e_dmwd = tagged(ts_i, qtag); end
                    4'd3: begin e_cls = C_QPUSH; e_dmop = DM_PUSH; e_dmwd = tagged(crcL, qtag); end
                    4'd4: begin e_cls = C_QPUT; e_put = 1'b1; e_idx = ars; e_cwd = {16'h0, rsel(rL, ard)}; end
                    4'd5: begin
                        e_cls = C_QGET; e_get = 1'b1; e_idx = ars;
                        e_r[16*ard +: 16] = ch_data_i[32*ars +: 16];
                    end
                    4'd6: begin e_cls = C_QPUT; e_put = 1'b1; e_idx = ars; e_cwd = srL; end
                    default: begin e_cls = C_QPUSH; e_dmop = DM_PUSH; e_dmwd = tagged({24'h0, IR[15:8]}, qtag); end
                endcase
                default: case (sub)    // 12 MISC (13-15 are illegal)
                    4'd0: e_cls = C_NOP;
                    4'd1: begin
                        e_cls = C_HALT; e_pc = pcL;
                        e_run = 1'b0; e_at = 1'b1; e_arm = 4'h0; e_ws = 1'b0; e_cw = 1'b0;
                    end
                    4'd2: begin e_cls = C_NOP; e_at = 1'b1; end
                    4'd3: begin e_cls = C_CMD; e_cmd = 2'd1; end
                    4'd4: begin e_cls = C_CMD; e_cmd = 2'd2; e_cmdd = {rsel(rL, 2'd1), rsel(rL, 2'd0)}; end
                    default: begin e_cls = C_CMD; e_cmd = 2'd3; end
                endcase
            endcase
        end
    end

    // ---------------------------------------------------------------------
    // D -> X snapshot (zero outside L's D cycle and in reset)
    // ---------------------------------------------------------------------
    logic [5:0]  x_cls;
    logic        x_pk;  logic [1:0] x_pkop; logic [27:0] x_pkctl; logic x_pkmem; logic [31:0] x_pkdat;
    logic        x_ack, x_ab; logic [3:0] x_evc;
    logic        x_cfg; logic [4:0] x_creg; logic x_chi; logic [15:0] x_cdat;
    logic [1:0]  x_cmd; logic [31:0] x_cmdd;
    logic        x_xw;  logic [1:0] x_xreg, x_xsrc; logic [15:0] x_xval;
    logic        x_fen, x_z, x_n;

    // isD is 0 in reset, so the snapshot is all-zero there and outside D.
    always_ff @(posedge clk) begin
        x_cls <= isD ? e_cls : C_NONE;
        x_pk <= isD && e_pk; x_pkop <= isD ? e_pkop : 2'd0; x_pkctl <= isD ? e_pkctl : 28'h0;
        x_pkmem <= isD && e_pkmem; x_pkdat <= isD ? e_pkdat : 32'h0;
        x_ack <= isD && e_ack; x_ab <= isD && e_ab; x_evc <= isD ? e_evc : 4'h0;
        x_cfg <= isD && e_cfg; x_creg <= isD ? e_creg : 5'h0; x_chi <= isD && e_chi;
        x_cdat <= isD ? e_cdat : 16'h0;
        x_cmd <= isD ? e_cmd : 2'd0; x_cmdd <= isD ? e_cmdd : 32'h0;
        x_xw <= isD && e_xw; x_xreg <= isD ? e_xreg : 2'd0; x_xsrc <= isD ? e_xsrc : 2'd0;
        x_xval <= isD ? e_xval : 16'h0;
        x_fen <= isD && e_fen; x_z <= isD && e_z; x_n <= isD && e_n;
    end

    // ---------------------------------------------------------------------
    // Next state of lane L in every cycle, registered and compared
    // ---------------------------------------------------------------------
    logic [8:0]  n_pc;  logic [63:0] n_r; logic n_z, n_n, n_pau, n_stp;
    logic [3:0]  n_arm; logic [35:0] n_tgt; logic n_ws, n_cw, n_run, n_at, n_qw;
    wire  [15:0] x_wv = (x_xsrc == 2'd0) ? x_xval :
                        (x_xsrc == 2'd1) ? dm_rdata_i[15:0] : dm_rdata_i[31:16];
    always @* begin
        n_pc = pcL; n_r = rL; n_z = zL; n_n = nL; n_pau = pauL; n_stp = stpL;
        n_arm = armL; n_tgt = tgtL; n_ws = wsL; n_cw = cwL; n_run = runL; n_at = atL; n_qw = qwL;
        if (isD) begin
            n_pc = e_pc; n_r = e_r; n_arm = e_arm; n_tgt = e_tgt; n_ws = e_ws; n_cw = e_cw;
            n_run = e_run; n_at = e_at; n_qw = e_qw; n_stp = e_stp;
        end
        if (isX) begin
            if (x_xw)  n_r[16*x_xreg +: 16] = x_wv;
            if (x_fen) begin n_z = x_z; n_n = x_n; end
        end
        // host writes (5.9); hreg_we_i only at ph = 1 (env), never a D cycle
        if (h_ctrl) begin
            n_run = hreg_wdata_i[0]; n_pau = hreg_wdata_i[1]; n_stp = hreg_wdata_i[2];
            if (!hreg_wdata_i[0]) begin n_arm = 4'h0; n_ws = 1'b0; n_cw = 1'b0; n_qw = 1'b0; end
        end
        if (h_pc && !runL) n_pc = hreg_wdata_i[8:0];
        if (h_irq && hreg_wdata_i[L]) n_at = 1'b0;
    end

    logic        chk_q;          // previous cycle was out of reset
    logic [1:0]  k_q;            // previous cycle: 0 other, 1 L's D, 2 L's X
    logic [5:0]  c_q;            // class of the D action concerned
    logic [8:0]  q_pc;  logic [63:0] q_r; logic q_z, q_n, q_pau, q_stp;
    logic [3:0]  q_arm; logic [35:0] q_tgt; logic q_ws, q_cw, q_run, q_at, q_qw;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            chk_q <= 1'b0; k_q <= 2'd0; c_q <= C_NONE;
            q_pc <= 9'h0; q_r <= 64'h0; q_z <= 1'b0; q_n <= 1'b0; q_pau <= 1'b0; q_stp <= 1'b0;
            q_arm <= 4'h0; q_tgt <= 36'h0; q_ws <= 1'b0; q_cw <= 1'b0; q_run <= 1'b0; q_at <= 1'b0; q_qw <= 1'b0;
        end else begin
            chk_q <= 1'b1;
            k_q   <= isD ? 2'd1 : isX ? 2'd2 : 2'd0;
            c_q   <= isD ? e_cls : x_cls;
            q_pc <= n_pc; q_r <= n_r; q_z <= n_z; q_n <= n_n; q_pau <= n_pau; q_stp <= n_stp;
            q_arm <= n_arm; q_tgt <= n_tgt; q_ws <= n_ws; q_cw <= n_cw; q_run <= n_run; q_at <= n_at; q_qw <= n_qw;
        end
    end
    wire cmp = rst_n && chk_q;

    wire st_ok = (pcL == q_pc) && (rL == q_r) && (zL == q_z) && (nL == q_n) &&
                 (pauL == q_pau) && (stpL == q_stp) && (armL == q_arm) && (tgtL == q_tgt) &&
                 (wsL == q_ws) && (cwL == q_cw) && (runL == q_run) && (atL == q_at) && (qwL == q_qw);

    // D-cycle outputs of lane L
    wire out_ok = (dm_op_o == e_dmop) && (dm_off_o == e_dmoff) && (dm_hmask_o == e_dmhm) &&
                  (dm_wdata_o == e_dmwd) && (ch_put_o == e_put) && (ch_get_o == e_get) &&
                  (!(e_put || e_get) || (ch_idx_o == e_idx)) && (!e_put || (ch_wdata_o == e_cwd));

    // X-cycle strobes and shared buses of lane L
    wire [31:0] x_pdat = x_pkmem ? dm_rdata_i : x_pkdat;
    wire strb_ok = (s_pk == x_pk) && (s_ack == x_ack) && (s_ab == x_ab) && (s_evc == x_evc) &&
                   (s_cfg == x_cfg) && (s_cmd == x_cmd) &&
                   (!x_pk  || ((lane_park_op_o == x_pkop) && (lane_park_ctl_o == x_pkctl) &&
                               (lane_park_data_o == x_pdat))) &&
                   (!x_cfg || ((lane_cfg_reg_o == x_creg) && (lane_cfg_hi_o == x_chi) &&
                               (lane_cfg_data_o == x_cdat))) &&
                   (!(x_cmd == 2'd2) || (lane_cmd_data_o == x_cmdd));

    // ---------------------------------------------------------------------
    // Per-class assertions (sequencer.md 9 property 10, and 11, 13, 14, 15,
    // 21, 22 through the model)
    // ---------------------------------------------------------------------
`define CLS(NM, C) \
    always @* begin \
        if (isD && e_cls == (C)) begin aO_``NM: assert (out_ok); end \
        if (isX && x_cls == (C)) begin aX_``NM: assert (strb_ok); end \
        if (cmp && k_q == 2'd1 && c_q == (C)) begin aD_``NM: assert (st_ok); end \
        if (cmp && k_q == 2'd2 && c_q == (C)) begin aW_``NM: assert (st_ok); end \
    end

    `CLS(none,     C_NONE)
    `CLS(del_r,    C_DEL_R)
    `CLS(del_hq,   C_DEL_HQ)
    `CLS(del_cap0, C_DEL_CAP0)
    `CLS(del_cap1, C_DEL_CAP1)
    `CLS(del_mem,  C_DEL_MEM)
    `CLS(del_ch,   C_DEL_CH)
    `CLS(del_oth,  C_DEL_OTH)
    `CLS(del_wait, C_DEL_WT)
    `CLS(vector,   C_VEC)
    `CLS(illegal,  C_ILL)
    `CLS(ex_wait,  C_EX_WT)
    `CLS(field,    C_FIELD)
    `CLS(raw,      C_RAW)
    `CLS(pin,      C_PIN)
    `CLS(setm,     C_SETM)
    `CLS(ldi,      C_LDI)
    `CLS(mem_ld,   C_MEM_LD)
    `CLS(mem_st,   C_MEM_ST)
    `CLS(br,       C_BR)
    `CLS(loop,     C_LOOP)
    `CLS(wait_hit, C_WT_HIT)
    `CLS(wait_to,  C_WT_TO)
    `CLS(wait_st,  C_WT_ST)
    `CLS(evt,      C_EVT)
    `CLS(q_pop,    C_QPOP)
    `CLS(q_push,   C_QPUSH)
    `CLS(q_put,    C_QPUT)
    `CLS(q_get,    C_QGET)
    `CLS(nop_irq,  C_NOP)
    `CLS(halt,     C_HALT)
    `CLS(crc_cmd,  C_CMD)
    `CLS(alu_mov,  C_ALU + 6'd0)
    `CLS(alu_add,  C_ALU + 6'd1)
    `CLS(alu_sub,  C_ALU + 6'd2)
    `CLS(alu_cmp,  C_ALU + 6'd3)
    `CLS(alu_and,  C_ALU + 6'd4)
    `CLS(alu_or,   C_ALU + 6'd5)
    `CLS(alu_xor,  C_ALU + 6'd6)
    `CLS(alu_shl,  C_ALU + 6'd7)
    `CLS(alu_shr,  C_ALU + 6'd8)
    `CLS(alu_inc,  C_ALU + 6'd9)
    `CLS(alu_dec,  C_ALU + 6'd10)
    `CLS(alu_addc, C_ALU + 6'd11)
    `CLS(alu_movsr,  C_ALU + 6'd12)
    `CLS(alu_movsrh, C_ALU + 6'd13)

    // Frame: a cycle that is neither L's D nor L's X changes lane L's state
    // only by host writes (property 6, and priority-1 (d)).
    always @* begin
        if (cmp && k_q == 2'd0) begin
            aF_frame: assert (st_ok);
        end
        // strobes to lane L are 0 outside L's X cycle (property 5, 6.)
        if (!isX) begin
            aF_no_strobe: assert (!s_any);
        end
    end

    // Field-wise comparison (diagnosis; the conjunction equals st_ok)
    always @* begin
        if (cmp) begin
            a_nx_pc:   assert (pcL  == q_pc);
            a_nx_r:    assert (rL   == q_r);
            a_nx_zn:   assert ((zL == q_z) && (nL == q_n));
            a_nx_ctl:  assert ((runL == q_run) && (pauL == q_pau) && (stpL == q_stp) && (atL == q_at));
            a_nx_ev:   assert ((armL == q_arm) && (tgtL == q_tgt));
            a_nx_misc: assert ((wsL == q_ws) && (cwL == q_cw) && (qwL == q_qw));
        end
    end

    // ---------------------------------------------------------------------
    // Port-level statements named after sequencer.md 9
    // ---------------------------------------------------------------------
    // Reset (priority 1 (g)): while rst_n is low every registered output is
    // its reset value; all lanes halted, no strobes, no requests.
    always @* begin
        if (!rst_n) begin
            a_rst_ph:     assert (ph_o == 5'b00001);
            a_rst_run:    assert ((run_o == 4'h0) && (lane_run_o == 4'h0) && (attn_o == 4'h0));
            a_rst_strobe: assert ((lane_park_we_o == 4'h0) && (lane_res_ack_o == 4'h0) &&
                                  (lane_abort_o == 4'h0) && (lane_ev_clr_o == 16'h0) &&
                                  (lane_cfg_we_o == 4'h0) && (lane_cmd_o == 8'h0));
            a_rst_req:    assert ((dm_op_o == DM_NONE) && !ch_put_o && !ch_get_o);
            a_rst_state:  assert ((obs_pc_o == 36'h0) && (obs_r_o == 256'h0) && (obs_z_o == 4'h0) &&
                                  (obs_n_o == 4'h0) && (obs_pause_o == 4'h0) && (obs_step_o == 4'h0) &&
                                  (obs_armed_o == 16'h0) && (obs_tgt_o == 144'h0) &&
                                  (obs_wsta_o == 4'h0) && (obs_capw_o == 4'h0) &&
                                  (lane_data_wait_o == 4'h0));
        end
    end

    // 1. Wheel: one-hot, 00001 in cycle 0 (and in reset), rotates; equal to
    //    the checker's own wheel.
    always @* begin
        a_p1_onehot: assert ((ph_o != 5'h0) && ((ph_o & (ph_o - 5'd1)) == 5'h0));
        a_p1_model:  assert (ph_o == m_oh);
        a_p1_range:  assert (m_ph <= 3'd4);
    end
    always @(posedge clk) begin
        if (!$initstate && rst_n) begin
            if (!$past(rst_n)) begin
                a_p1_cycle0: assert (ph_o == 5'b00001);
            end else begin
                a_p1_rotate: assert (ph_o == {$past(ph_o[3:0]), $past(ph_o[4])});
            end
        end
    end

    // 2. Fetch address.
    always @* begin
        if (ph_o[0]) begin a_p2_fetch0: assert (pm_addr_o == obs_pc_o[8:0]);   end
        if (ph_o[1]) begin a_p2_fetch1: assert (pm_addr_o == obs_pc_o[17:9]);  end
        if (ph_o[2]) begin a_p2_fetch2: assert (pm_addr_o == obs_pc_o[26:18]); end
        if (ph_o[3]) begin a_p2_fetch3: assert (pm_addr_o == obs_pc_o[35:27]); end
        if (ph_o[4]) begin a_p2_host:   assert (pm_addr_o == 9'h0);            end
    end

    // 3. Host position free; at most one channel op; never with a host
    //    register write (priority 1 (e), (f)). Data-memory outputs are
    //    0 / NONE in the host D cycle (2.3 "0 otherwise").
    always @* begin
        if (rst_n && m_ph == 3'd1) begin
            a_p3_host_dm: assert ((dm_op_o == DM_NONE) && (dm_off_o == 7'h0) &&
                                  (dm_hmask_o == 2'b00) && (dm_wdata_o == 32'h0));
            a_p3_host_ch: assert (!ch_put_o && !ch_get_o);
        end
        a_p3_one_ch:  assert (!(ch_put_o && ch_get_o));
        a_p3_ch_hreg: assert (!((ch_put_o || ch_get_o) && hreg_we_i));
        a_p3_dm_code: assert (dm_op_o <= DM_PUSH);
    end

    // 4. Never over- or under-run (owner = D owner of this cycle).
    always @* begin
        if (rst_n) begin
            if (dm_op_o == DM_POP)  begin a_p4_pop:  assert (!qe_i[downer]); end
            if (dm_op_o == DM_PUSH) begin a_p4_push: assert (!qf_i[downer]); end
            if (ch_put_o) begin a_p4_put: assert (!ch_valid_i[ch_idx_o]); end
            if (ch_get_o) begin a_p4_get: assert ( ch_valid_i[ch_idx_o]); end
        end
    end

    // 5. Strobe discipline for lane L (priority 1 (c)), plus the guarantees
    //    lane.md 2.1 needs.
    wire [3:0] s_cnt = {3'b000, s_pk} + {3'b000, s_cfg} + {3'b000, (s_cmd != 2'd0)} +
                       {3'b000, s_ack} + {3'b000, s_ab};
    always @* begin
        if (s_any) begin
            a_p5_xcycle: assert (isX);
            a_p5_run:    assert (runL && lane_run_o[L]);
        end
        if (s_pk)  begin a_p5_park_free: assert (!pfL); a_p5_park_op: assert (lane_park_op_o != 2'd3); end
        if (s_ack) begin a_p5_ack_valid: assert (rvL); end
        a_p5_one: assert (s_cnt <= 4'd1);
        if (s_evc != 4'h0) begin a_p5_evclr_alone: assert (!s_pk && !s_cfg && (s_cmd == 2'd0) && !s_ack); end
        if (s_cfg) begin
            a_p5_cfg_reg: assert (lane_cfg_reg_o <= 5'd13);
            if (lane_cfg_reg_o <= 5'd9) begin a_p5_cfg_idle: assert (!busyL && !pfL); end
        end
        if (s_cmd != 2'd0) begin a_p5_cmd_idle: assert (!busyL && !pfL); end
        a_p5_lane_run:  assert (lane_run_o == run_o);
    end

    // 6. Slot locality (priority 1 (a), (b)): PC changes only at the end of
    //    L's D cycle or by a host PC write while halted; registers only at
    //    the end of L's D or X cycle; Z/N only at the end of L's X cycle
    //    after a committed ALU instruction (20.); control state only at the
    //    end of L's D cycle or by a host write to CTRL[L] / IRQSTAT.
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n)) begin
            if (pcL != $past(pcL)) begin
                a_p6_pc: assert ($past(isD) || $past(h_pc && !runL));
            end
            if (rL != $past(rL)) begin
                a_p6_r: assert ($past(isD) || $past(isX));
            end
            if ((zL != $past(zL)) || (nL != $past(nL))) begin
                a_p20_zn: assert ($past(isX) && ($past(x_cls) >= C_ALU));
            end
            if ((runL != $past(runL)) || (pauL != $past(pauL)) || (stpL != $past(stpL)) ||
                (armL != $past(armL)) || (wsL != $past(wsL)) || (cwL != $past(cwL)) ||
                (qwL != $past(qwL))) begin
                a_p6_ctl: assert ($past(isD) || $past(h_ctrl));
            end
            if (tgtL != $past(tgtL)) begin
                a_p6_tgt: assert ($past(isD));
            end
            if (atL != $past(atL)) begin
                a_p6_attn: assert ($past(isD) || $past(h_irq));
            end
        end
    end

    // 7. Halted and paused lanes are inert; (d) a halted lane changes only
    //    by host writes.
    always @* begin
        if (isD && !elig) begin
            a_p7_no_dm: assert ((dm_op_o == DM_NONE) && !ch_put_o && !ch_get_o);
        end
    end
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n) && $past(isD && !elig)) begin
            a_p7_no_strobe: assert (!s_any);
            a_p7_no_change: assert ((pcL == $past(pcL)) && (rL == $past(rL)) && (armL == $past(armL)) &&
                                    (tgtL == $past(tgtL)) && (wsL == $past(wsL)) && (cwL == $past(cwL)) &&
                                    (runL == $past(runL)) && (atL == $past(atL)) && (qwL == $past(qwL)));
        end
        // a lane halted at the start of its D cycle gets nothing in X
        if (!$initstate && rst_n && $past(rst_n) && $past(isD && !runL)) begin
            a_p7_halted: assert (!s_any);
        end
    end

    // 8. No lane runs before the host starts it.
    // 15./16. run falls only at the end of L's D cycle (HALT or an
    //    undefined code, PC kept, attn set, clears) or by a host CTRL write
    //    with run = 0; a host PC write while running is ignored.
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n)) begin
            if (runL && !$past(runL)) begin
                a_p8_start: assert ($past(h_ctrl && hreg_wdata_i[0] && m_ph == 3'd1));
            end
            if (!runL && $past(runL)) begin
                a_p15_stop: assert ($past(isD) || $past(h_ctrl && !hreg_wdata_i[0]));
                if ($past(isD)) begin
                    a_p15_halt_state: assert ((pcL == $past(pcL)) && atL && (armL == 4'h0) &&
                                              !wsL && !cwL && !qwL);
                end else begin
                    a_p16_host_halt: assert ((armL == 4'h0) && !wsL && !cwL && !qwL);
                end
            end
            if ($past(h_pc && runL)) begin
                a_p16_pc_ignored: assert (pcL == $past(pcL));
            end
        end
    end

    // 9./14. Priority and vector (written on the inputs and ports, without
    //    the instruction model).
    logic [1:0] kvec_q; logic [8:0] tgtk_q;
    always_ff @(posedge clk) begin kvec_q <= kvec; tgtk_q <= tgtL[9*kvec +: 9]; end
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n) && $past(isD && elig)) begin
            if ($past(rvL)) begin
                a_p9_deliver: assert ((pcL == $past(pcL)) && !s_pk && !s_ab && !s_cfg &&
                                      (s_cmd == 2'd0) && (s_evc == 4'h0));
            end else if ($past(evm != 4'h0)) begin
                a_p14_vec_pc:  assert (pcL == tgtk_q);
                a_p14_vec_x:   assert (s_ab && (s_evc == onehot2(kvec_q)) && !s_pk && !s_ack &&
                                       !s_cfg && (s_cmd == 2'd0));
                a_p14_armed:   assert (armL == $past(armL));
            end
        end
    end

    // 12. Run-ahead: no EXEC commits while the parked register is full
    //     (except WAIT with wsta = 1) and no sync instruction commits unless
    //     IDLE (sequencer.md 5.11 list): PC stays and nothing is sent.
    wire is_sync = ((op == 4'd3) && (cond <= 5'd9)) ||
                   ((op == 4'd12) && ((sub == 4'd1) || (sub == 4'd3) || (sub == 4'd4) || (sub == 4'd5))) ||
                   ((op == 4'd5) && ((sub == 4'd12) || (sub == 4'd13))) ||
                   ((op == 4'd11) && ((sub == 4'd3) || (sub == 4'd6))) ||
                   ((op == 4'd7) && ((cond == 5'd5) || (cond == 5'd6))) ||
                   ((op == 4'd9) && !wsL && !w_hit);   // a WAIT that would start its timeout
    wire exec_sel = isD && elig && !rvL && (evm == 4'h0) && !illegal;
    wire blocked  = exec_sel && ((pfL && !((op == 4'd9) && wsL)) || (is_sync && !IDLE) || HAZ);
    always @* begin
        if (blocked) begin
            a_p12_no_dm: assert ((dm_op_o == DM_NONE) && !ch_put_o && !ch_get_o);
        end
    end
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n) && $past(blocked)) begin
            a_p12_pc:        assert (pcL == $past(pcL));
            a_p12_no_strobe: assert (!s_any);
            a_p12_regs:      assert (rL == $past(rL));
        end
    end

    // Deliveries (priority 3): an ack to lane L is given exactly in the X
    // cycle after a completing delivery D cycle; a CAP record's first word
    // gives none. Every delivery writes the result once (aO_del_*).
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n)) begin
            if (s_ack) begin
                a_p13_ack_after_deliver: assert ($past(isD && elig && rvL && !del_wait &&
                                                      !((rdstL == 4'd5) && !cwL)));
            end
        end
    end

    // Host read mux (2.5, host.md 4): combinational, exact.
    wire [6:0] ha = hreg_addr_i;
    wire       ha_lane = (ha[6:3] >= 4'd2) && (ha[6:3] <= 4'd5);
    wire [1:0] ha_l    = ha[4:3] - 2'd2;     // 0x10 -> 0, 0x18 -> 1, ...
    always @* begin
        if (ha == A_IRQSTAT) begin
            a_rd_irqstat: assert (hreg_rdata_o == {20'h0, run_o, 4'h0, attn_o});
        end else if (ha_lane && ha_l == L && ha[2:0] == 3'd0) begin
            a_rd_ctrl:  assert (hreg_rdata_o == {22'h0, qwL, cwL, wsL, rvL, pfL, busyL, atL, stpL, pauL, runL});
        end else if (ha_lane && ha_l == L && ha[2:0] == 3'd1) begin
            a_rd_pc:    assert (hreg_rdata_o == {23'h0, pcL});
        end else if (ha_lane && ha_l == L && ha[2:0] == 3'd2) begin
            a_rd_flags: assert (hreg_rdata_o == {19'h0, armL, qfL, qeL, evL, mL, nL, zL});
        end else if (ha_lane && ha_l == L && ha[2:0] == 3'd3) begin
            a_rd_r10:   assert (hreg_rdata_o == {rsel(rL, 2'd1), rsel(rL, 2'd0)});
        end else if (ha_lane && ha_l == L && ha[2:0] == 3'd4) begin
            a_rd_r32:   assert (hreg_rdata_o == {rsel(rL, 2'd3), rsel(rL, 2'd2)});
        end else if (!ha_lane || (ha[2:0] >= 3'd5)) begin
            a_rd_zero:  assert (hreg_rdata_o == 32'h0);
        end
    end

    // ---------------------------------------------------------------------
    // Environment (assumed with STANDALONE = 1). Sources:
    //  env_rst     reset in the first cycle (conventions.md, Formal)
    //  env_hreg_ph hreg_we_i only in ph = 1 cycles (host.md 7.1 / 3.3;
    //              sequencer.md 2.5)
    //  env_park    lane L: park_full rises only in the cycle after a park
    //              (lane.md 9.5, 7.1)
    //  env_idle    lane L: busy = park_full = 0 and no park in t, running
    //              -> busy = park_full = 0 in t+1 (lane.md 9.4, 7.6)
    //  env_res     lane L: a waiting result stays, with stable dst, data
    //              and res_ts, until the end of its ack cycle, while
    //              running (lane.md 7.5, 9.7)
    //  env_sr      lane L: sr stable while idle, running, with no cmd or
    //              cfg write in t or t-1 (lane.md 7.6, 5.2/5.3 crcw_q)
    //  env_chan    channel data changes only by a put to that channel
    //              (channels.md 2, 4.4)
    // ---------------------------------------------------------------------
    logic s_cfg_q;     // lane L cfg write in the previous cycle (crcw_q)
    always_ff @(posedge clk) s_cfg_q <= s_cfg;

    generate
        if (STANDALONE) begin : g_env_assume
            always @* begin
                if ($initstate) begin
                    env_rst: assume (!rst_n);
                end
                if (hreg_we_i) begin
                    env_hreg_ph: assume (ph_o[1] && rst_n);
                end
            end
            always @(posedge clk) begin
                if (!$initstate && $past(rst_n)) begin
                    if (!$past(pfL) && pfL) begin
                        env_park: assume ($past(s_pk));
                    end
                    if ($past(IDLE && !s_pk && runL)) begin
                        env_idle: assume (IDLE);
                    end
                    if ($past(rvL && !s_ack && runL)) begin
                        env_res: assume (rvL && (rdstL == $past(rdstL)) && (rdatL == $past(rdatL)) &&
                                         (rtsL == $past(rtsL)));
                    end
                    if ($past(IDLE && runL && !s_cfg && (s_cmd == 2'd0) && !s_cfg_q)) begin
                        env_sr: assume (srL == $past(srL));
                    end
                    if (!$past(ch_put_o && ch_idx_o == 2'd0)) begin env_chan0: assume (ch_data_i[31:0]   == $past(ch_data_i[31:0]));   end
                    if (!$past(ch_put_o && ch_idx_o == 2'd1)) begin env_chan1: assume (ch_data_i[63:32]  == $past(ch_data_i[63:32]));  end
                    if (!$past(ch_put_o && ch_idx_o == 2'd2)) begin env_chan2: assume (ch_data_i[95:64]  == $past(ch_data_i[95:64]));  end
                    if (!$past(ch_put_o && ch_idx_o == 2'd3)) begin env_chan3: assume (ch_data_i[127:96] == $past(ch_data_i[127:96])); end
                end
            end
        end else begin : g_env_assert
            always @* begin
                if (hreg_we_i) begin
                    env_hreg_ph: assert (ph_o[1] && rst_n);
                end
            end
            always @(posedge clk) begin
                if (!$initstate && $past(rst_n) && rst_n) begin
                    if (!$past(pfL) && pfL) begin
                        env_park: assert ($past(s_pk));
                    end
                    if ($past(IDLE && !s_pk && runL)) begin
                        env_idle: assert (IDLE);
                    end
                    if ($past(rvL && !s_ack && runL)) begin
                        env_res: assert (rvL && (rdstL == $past(rdstL)) && (rdatL == $past(rdatL)) &&
                                         (rtsL == $past(rtsL)));
                    end
                    if ($past(IDLE && runL && !s_cfg && (s_cmd == 2'd0) && !s_cfg_q)) begin
                        env_sr: assert (srL == $past(srL));
                    end
                    if (!$past(ch_put_o && ch_idx_o == 2'd0)) begin env_chan0: assert (ch_data_i[31:0]   == $past(ch_data_i[31:0]));   end
                    if (!$past(ch_put_o && ch_idx_o == 2'd1)) begin env_chan1: assert (ch_data_i[63:32]  == $past(ch_data_i[63:32]));  end
                    if (!$past(ch_put_o && ch_idx_o == 2'd2)) begin env_chan2: assert (ch_data_i[95:64]  == $past(ch_data_i[95:64]));  end
                    if (!$past(ch_put_o && ch_idx_o == 2'd3)) begin env_chan3: assert (ch_data_i[127:96] == $past(ch_data_i[127:96])); end
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Covers (sequencer.md 9 property 24; every assumption reachable)
    // ---------------------------------------------------------------------
    generate
        if (STANDALONE) begin : g_cover
            // retry: a wait, then the same instruction commits 5 cycles later
            logic       w_seen;  logic [8:0] w_pc;
            logic       cap0_seen, park_seen, wst_seen;
            logic [2:0] park_age;
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    w_seen <= 1'b0; w_pc <= 9'h0; cap0_seen <= 1'b0; park_seen <= 1'b0;
                    park_age <= 3'd0; wst_seen <= 1'b0;
                end else begin
                    if (isD && e_cls == C_EX_WT) begin w_seen <= 1'b1; w_pc <= pcL; end
                    else if (isD) w_seen <= 1'b0;
                    if (isD) cap0_seen <= (e_cls == C_DEL_CAP0);
                    if (s_pk) begin park_seen <= 1'b1; park_age <= 3'd0; end
                    else if (park_age != 3'd7) park_age <= park_age + 3'd1;
                    if (isD && e_cls == C_WT_ST) wst_seen <= 1'b1;
                end
            end
            always @* begin
                if (rst_n) begin
                    c_retry:      cover (isD && w_seen && (pcL == w_pc) && (e_cls >= C_FIELD) && (e_cls != C_WT_ST));
                    c_cap_pair:   cover (isD && cap0_seen && e_cls == C_DEL_CAP1);
                    c_park_b2b:   cover (s_pk && park_seen && park_age == 3'd4);
                    c_wait_hit_after_start: cover (isD && wst_seen && e_cls == C_WT_HIT && wsL);
                    c_wait_to_after_start:  cover (isD && wst_seen && e_cls == C_WT_TO);
                    c_ill_halt:   cover (isD && e_cls == C_ILL);
                    c_ill_op15:   cover (isD && e_cls == C_ILL && op == 4'd15);
                    c_ill_fr:     cover (isD && e_cls == C_ILL && is_fr);
                    c_vec0:       cover (isD && e_cls == C_VEC && kvec == 2'd0);
                    c_vec3:       cover (isD && e_cls == C_VEC && kvec == 2'd3 && evL[0]);
                    c_vec_parked: cover (isD && e_cls == C_VEC && pfL);
                    c_memld_pi_same: cover (isD && e_cls == C_MEM_LD && m_pi && !m_ab && (mdr == ard));
                    c_step:       cover (isD && elig && pauL && stpL && e_cls != C_NONE);
                    c_host_halt:  cover (h_ctrl && !hreg_wdata_i[0] && runL);
                    c_host_pc:    cover (h_pc && !runL);
                    c_qwait:      cover (isD && e_cls == C_EX_WT && dc_fail);
                    c_haz_wait:   cover (isD && e_cls == C_EX_WT && HAZ && !pfL);
                    c_sync_wait:  cover (isD && e_cls == C_EX_WT && is_sync && !pfL && busyL);
                    c_park_wait:  cover (isD && e_cls == C_EX_WT && pfL);
                    c_tsword_borrow: cover (isD && e_cls == C_DEL_CAP0 && (ts_i[15:0] < rtsL));
                    // environment reachable
                    c_env_park:   cover (s_pk);
                    c_env_idle:   cover (isX && s_cmd != 2'd0);
                    c_env_chan:   cover (ch_put_o);
                    c_env_hreg:   cover (hreg_we_i);
                end
            end
            // a result held over a slot boundary (env_res)
            logic rv_slot;
            always_ff @(posedge clk or negedge rst_n)
                if (!rst_n) rv_slot <= 1'b0;
                else if (isD) rv_slot <= rvL && elig && del_wait;
            always @* if (rst_n) begin c_env_res: cover (isD && rv_slot && rvL && e_cls != C_DEL_WT); end
            // every class reached
`define CC(NM, C) always @* if (rst_n) begin cc_``NM: cover (isD && e_cls == (C)); end
            `CC(none, C_NONE)        `CC(del_r, C_DEL_R)      `CC(del_hq, C_DEL_HQ)
            `CC(del_cap0, C_DEL_CAP0) `CC(del_cap1, C_DEL_CAP1) `CC(del_mem, C_DEL_MEM)
            `CC(del_ch, C_DEL_CH)    `CC(del_oth, C_DEL_OTH)  `CC(del_wait, C_DEL_WT)
            `CC(vector, C_VEC)       `CC(illegal, C_ILL)      `CC(ex_wait, C_EX_WT)
            `CC(field, C_FIELD)      `CC(raw, C_RAW)          `CC(pin, C_PIN)
            `CC(setm, C_SETM)        `CC(ldi, C_LDI)          `CC(mem_ld, C_MEM_LD)
            `CC(mem_st, C_MEM_ST)    `CC(br, C_BR)            `CC(loop, C_LOOP)
            `CC(wait_hit, C_WT_HIT)  `CC(wait_to, C_WT_TO)    `CC(wait_st, C_WT_ST)
            `CC(evt, C_EVT)          `CC(q_pop, C_QPOP)       `CC(q_push, C_QPUSH)
            `CC(q_put, C_QPUT)       `CC(q_get, C_QGET)       `CC(nop_irq, C_NOP)
            `CC(halt, C_HALT)        `CC(crc_cmd, C_CMD)
            `CC(alu_mov, C_ALU+6'd0)  `CC(alu_add, C_ALU+6'd1)  `CC(alu_sub, C_ALU+6'd2)
            `CC(alu_cmp, C_ALU+6'd3)  `CC(alu_and, C_ALU+6'd4)  `CC(alu_or, C_ALU+6'd5)
            `CC(alu_xor, C_ALU+6'd6)  `CC(alu_shl, C_ALU+6'd7)  `CC(alu_shr, C_ALU+6'd8)
            `CC(alu_inc, C_ALU+6'd9)  `CC(alu_dec, C_ALU+6'd10) `CC(alu_addc, C_ALU+6'd11)
            `CC(alu_movsr, C_ALU+6'd12) `CC(alu_movsrh, C_ALU+6'd13)
            // every BR condition, taken
            genvar gb;
            for (gb = 0; gb < 19; gb = gb + 1) begin : g_brc
                always @* if (rst_n) cover (isD && e_cls == C_BR && cond == gb && taken);  // c_br_cond
            end
        end
    endgenerate
endmodule
