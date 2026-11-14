`default_nettype none
// rosetta_seq: shared sequencer (docs/blocks/sequencer.md, amended by
// docs/decisions.md rulings 1, 8-13, 16, 21, 26, 29, 30; lane side per
// docs/blocks/lane.md 2 and 7; encodings per docs/isa.md v0.2 section 3).
//
// Slot wheel, four-stage pipeline (F address, I instruction register, D
// decide/data memory/channels, X execute), per-lane program state, deliveries,
// event vectoring, ALU, halt and single step.
//
// Flops (all reset to 0, except ph to 00001):
//   per-lane state 122 x 4 = 488 (pc 9, r 64, Z N 2, run pause step attn 4,
//     armed 4, tgt 36, wsta capw qwait 3), ph 5 + replica of ph[0] 1, IR 32,
//   D/X register 59: strobes park/ack/abort/cfg 4, cmd 2, ev_clr 4, X write
//     kind 2, flag enable 1, payload xa 16, xb 16, xc 14.   Total 585.
//
// D/X payload use (written at the end of the D cycle, read in the X cycle):
//   park : xc = {op, ctl[27:16]}, xb = ctl[15:0], xa = R[s] (source 0-3)
//   ALU  : xa = A, xb = B, xc[11:8] = op, xc[13:12] = rd   (LOOP: op = DEC)
//   SETM : xc[11:6] = IR[27:22] (reg, hi), xa = value
//   CRCCHK: {xb, xa} = {R1, R0};  MEM load / POP: xc[13:12] = register
// Source data from dm_rdata_i and ch_data_i are muxed in the X cycle
// (sequencer.md 5.2 stability note, 10).
//
// Timing structure (16.67 ns, SRAM data 6.45 ns after the clock, slow):
//   - IR is a plain register on pm_rdata_i; nothing decodes the SRAM output.
//   - Every per-lane select is a one-hot AND-OR straight off the ph ring
//     flops (dl/xl): two gate levels, no binary lane index anywhere. ph[0]
//     has a reset-0 replica for these selects (see 4.1).
//   - Two register read ports RA/RB in D, 4:1 by an index taken from IR
//     only. Post-increment uses one incrementer per register index (on the
//     selected lane's R0-R3), so no index mux sits in front of the carry;
//     the MEM-delivery address is R3 directly.
//   - BR and LOOP decide from registered Z/N/flags; LOOP compares each
//     register with 1 and then selects by rd.
//   - ALU: SUB/CMP's ~B and the INC/DEC constants are formed in D, so the X
//     stage starts at the adder. One Kogge-Stone adder serves ADD, SUB, CMP,
//     INC, DEC, ADDC; ADDC's end-around carry is the group generate G[15:0]
//     fed in as carry-in (one AO level, not a second adder). Z uses a
//     carry-free test for the adder ops; results and loads are one flat
//     one-hot AND-OR into the register write port.
//   - Commit is formed per opcode class (go_*), and each output or state
//     enable uses only its own class terms. Write data are selected by
//     decode and phase; the commit enters only as the final enable.
//   - The lane strobes are D/X flops ANDed with the one-hot X owner.
//
// Readings of the spec where it is silent or ambiguous (also in the report):
//   - RAW SAMPLE (IR[27] = 0): its source is taken as 15 everywhere,
//     including the illegal-source check (5.2); its destination field is
//     checked for 6, 12-14 like every FIELD/RAW (5.2 literal), also RAW DRIVE.
//   - WAIT with wsta = 0 and park_full = 1 waits even on a hit (5.2 general
//     condition applies before 5.5).
//   - hreg writes act only in ph = 1 cycles (5.9 "end of a ph = 1 cycle").
//   - Shared lane buses carry the D/X payload in every cycle; they are
//     meaningful only with the strobe (sequencer.md 7.3).
module rosetta_seq #(
    parameter int N_LANES = 4
) (
    input  wire                     clk, rst_n,
    // 2.1 wheel
    output logic [N_LANES:0]        ph_o,
    // 2.2 program memory
    output logic [8:0]              pm_addr_o,
    input  wire  [31:0]             pm_rdata_i,
    // 2.3 data memory (combinational in the D cycle)
    output logic [2:0]              dm_op_o,
    output logic [6:0]              dm_off_o,
    output logic [1:0]              dm_hmask_o,
    output logic [31:0]             dm_wdata_o,
    input  wire  [31:0]             dm_rdata_i,
    input  wire  [N_LANES-1:0]      qe_i, qf_i,
    // 2.4 channels and timestamp (combinational in the D cycle)
    output logic                    ch_put_o, ch_get_o,
    output logic [1:0]              ch_idx_o,
    output logic [31:0]             ch_wdata_o,
    input  wire  [3:0]              ch_valid_i,
    input  wire  [127:0]            ch_data_i,
    input  wire  [31:0]             ts_i,
    // 2.5 host register bus
    input  wire                     hreg_we_i,
    input  wire  [6:0]              hreg_addr_i,
    input  wire  [31:0]             hreg_wdata_i,
    output logic [31:0]             hreg_rdata_o,
    output logic [N_LANES-1:0]      run_o, attn_o,
    // 2.6 lane interface (lane.md 2.1 / 2.3 port names, prefixed lane_)
    output logic [N_LANES-1:0]      lane_run_o,
    output logic [N_LANES-1:0]      lane_park_we_o,
    output logic [1:0]              lane_park_op_o,
    output logic [27:0]             lane_park_ctl_o,
    output logic [31:0]             lane_park_data_o,
    output logic [N_LANES-1:0]      lane_res_ack_o,
    output logic [N_LANES-1:0]      lane_abort_o,
    output logic [4*N_LANES-1:0]    lane_ev_clr_o,
    output logic [N_LANES-1:0]      lane_cfg_we_o,
    output logic [4:0]              lane_cfg_reg_o,
    output logic                    lane_cfg_hi_o,
    output logic [15:0]             lane_cfg_data_o,
    output logic [2*N_LANES-1:0]    lane_cmd_o,
    output logic [31:0]             lane_cmd_data_o,
    output logic [N_LANES-1:0]      lane_data_wait_o,
    input  wire  [N_LANES-1:0]      lane_busy_i,
    input  wire  [N_LANES-1:0]      lane_park_full_i,
    input  wire  [N_LANES-1:0]      lane_res_valid_i,
    input  wire  [4*N_LANES-1:0]    lane_res_dst_i,
    input  wire  [32*N_LANES-1:0]   lane_res_data_i,
    input  wire  [16*N_LANES-1:0]   lane_res_ts_i,
    input  wire  [4*N_LANES-1:0]    lane_dst_busy_i,
    input  wire  [N_LANES-1:0]      lane_m_i,
    input  wire  [4*N_LANES-1:0]    lane_ev_pend_i,
    input  wire  [32*N_LANES-1:0]   lane_sr_i,
    input  wire  [32*N_LANES-1:0]   lane_crc_i,
    // 2.7 observation
    output logic [9*N_LANES-1:0]    obs_pc_o,
    output logic [64*N_LANES-1:0]   obs_r_o,
    output logic [N_LANES-1:0]      obs_z_o, obs_n_o,
    output logic [N_LANES-1:0]      obs_pause_o, obs_step_o,
    output logic [4*N_LANES-1:0]    obs_armed_o,
    output logic [36*N_LANES-1:0]   obs_tgt_o,
    output logic [N_LANES-1:0]      obs_wsta_o, obs_capw_o
);
  localparam int NL = N_LANES;
  localparam int NP = N_LANES + 1;          // wheel positions; host is NL

  // Data-memory operation codes (2.3).
  localparam logic [2:0] DM_NONE = 3'd0, DM_READ = 3'd1, DM_WRITE = 3'd2,
                         DM_POP = 3'd3, DM_PUSH = 3'd4;

  // ================================================================ 3. State
  logic [9*NL-1:0]  pc;
  logic [64*NL-1:0] r;                       // r[L][n] at [64L + 16n +: 16]
  logic [NL-1:0]    zf, nf, run, pause, step, attn, wsta, capw, qwait;
  logic [4*NL-1:0]  armed;
  logic [36*NL-1:0] tgt;                     // tgt[L][k] at [36L + 9k +: 9]

  // ============================================================== 4.1 Wheel
  logic [NP-1:0] ph;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) ph <= {{(NP-1){1'b0}}, 1'b1};
    else        ph <= {ph[NP-2:0], ph[NP-1]};
  assign ph_o = ph;

  // Stage owners, one-hot from the ring (section 1): lane L has F at ph = L,
  // D at (L+2) mod 5, X at (L+3) mod 5. The host's D cycle is ph = 1.
  // phs: the ring as seen by the internal selects. phs[0] is a replica of
  // ph[0] that resets to 0 instead of 1 (the cell library has no set flop,
  // so ph[0] is a reset flop with inverters, and that inverter would drive
  // every lane-3 D select and lane-2 X select, about 330 loads). The two
  // differ only in cycle 0 after reset, where every lane is halted, all
  // registers and the D/X register are 0 and pc[0] = 0, so no select there
  // has any effect (pm_addr_o = pc[0] = 0 either way).
  logic          ph0r;
  logic [NP-1:0] phs;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) ph0r <= 1'b0;
    else        ph0r <= ph[NP-1];
  assign phs = {ph[NP-1:1], ph0r};

  logic [NL-1:0] dl, xl;
  for (genvar L = 0; L < NL; L++) begin : g_own
    assign dl[L] = phs[(L+2) % NP];
    assign xl[L] = phs[(L+3) % NP];
  end

  // ========================================================= 4.2 Fetch (F)
  // pm_addr_o = pc[k] with ph = k (k < NL), 0 with ph = NL. Combinational,
  // so a host PC write at the end of ph = 1 is fetched at once (property 2).
  always_comb begin
    pm_addr_o = '0;
    for (int L = 0; L < NL; L++) pm_addr_o |= {9{phs[L]}} & pc[9*L +: 9];
  end

  // =================================== 4.2 Instruction register (I stage)
  // IR <= pm_rdata_i at the end of every cycle; never decoded from the SRAM
  // output directly.
  logic [31:0] ir;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) ir <= '0;
    else        ir <= pm_rdata_i;

  // ==================================== D stage: per-lane state select mux
  // Values of the D owner (section 1: "in the D cycle of lane L"). All zero
  // in the host's D cycle (ph = 1), so nothing is eligible there.
  logic        d_run, d_pause, d_step, d_wsta, d_capw, d_z, d_n;
  logic        d_busy, d_pkf, d_rv, d_m, d_qe, d_qf;
  logic [3:0]  d_armed, d_rdst, d_dbusy, d_ev;
  logic [8:0]  d_pc;
  logic [35:0] d_tgt;
  logic [63:0] d_r;
  logic [31:0] d_rdata, d_sr, d_crc;
  logic [15:0] d_rts;
  always_comb begin
    {d_run, d_pause, d_step, d_wsta, d_capw, d_z, d_n} = '0;
    {d_busy, d_pkf, d_rv, d_m, d_qe, d_qf}             = '0;
    {d_armed, d_rdst, d_dbusy, d_ev}                   = '0;
    d_pc = '0; d_tgt = '0; d_r = '0;
    d_rdata = '0; d_sr = '0; d_crc = '0; d_rts = '0;
    for (int L = 0; L < NL; L++) begin
      d_run   |= dl[L] & run[L];
      d_pause |= dl[L] & pause[L];
      d_step  |= dl[L] & step[L];
      d_wsta  |= dl[L] & wsta[L];
      d_capw  |= dl[L] & capw[L];
      d_z     |= dl[L] & zf[L];
      d_n     |= dl[L] & nf[L];
      d_busy  |= dl[L] & lane_busy_i[L];
      d_pkf   |= dl[L] & lane_park_full_i[L];
      d_rv    |= dl[L] & lane_res_valid_i[L];
      d_m     |= dl[L] & lane_m_i[L];
      d_qe    |= dl[L] & qe_i[L];
      d_qf    |= dl[L] & qf_i[L];
      d_armed |= {4{dl[L]}}  & armed[4*L +: 4];
      d_rdst  |= {4{dl[L]}}  & lane_res_dst_i[4*L +: 4];
      d_dbusy |= {4{dl[L]}}  & lane_dst_busy_i[4*L +: 4];
      d_ev    |= {4{dl[L]}}  & lane_ev_pend_i[4*L +: 4];
      d_pc    |= {9{dl[L]}}  & pc[9*L +: 9];
      d_tgt   |= {36{dl[L]}} & tgt[36*L +: 36];
      d_r     |= {64{dl[L]}} & r[64*L +: 64];
      d_rdata |= {32{dl[L]}} & lane_res_data_i[32*L +: 32];
      d_sr    |= {32{dl[L]}} & lane_sr_i[32*L +: 32];
      d_crc   |= {32{dl[L]}} & lane_crc_i[32*L +: 32];
      d_rts   |= {16{dl[L]}} & lane_res_ts_i[16*L +: 16];
    end
  end

  // ============================================================ 5.2 Decode
  logic [3:0]  op, sub, s, dst;
  logic [4:0]  cond, mreg;
  logic [12:0] opd;                         // one-hot opcode 0-12
  logic i_fld, i_raw, i_pin, i_setm, i_ldi, i_alu, i_mem, i_br, i_loop;
  logic i_wait, i_evt, i_q, i_misc, fr;
  logic q_pop, q_push, q_pushts, q_pushcrc, q_put, q_get, q_putsr, q_pushi;
  logic m_halt, m_irq, m_crcinit, m_crcchk, m_stuffclr;
  logic s_reg, s_hq, s_mem, s_ch, s_bad, dst_bad, ill;

  assign op   = ir[31:28];
  assign sub  = ir[27:24];                   // ALU, Q, MISC operation
  assign cond = ir[27:23];                   // BR condition
  assign mreg = ir[27:23];                   // SETM register
  assign opd  = 13'(16'h0001 << op);
  assign {i_misc, i_q, i_evt, i_wait, i_loop, i_br, i_mem, i_alu, i_ldi,
          i_setm, i_pin, i_raw, i_fld} = opd;
  assign fr   = i_fld | i_raw;

  assign q_pop     = i_q & sub == 4'd0;
  assign q_push    = i_q & sub == 4'd1;
  assign q_pushts  = i_q & sub == 4'd2;
  assign q_pushcrc = i_q & sub == 4'd3;
  assign q_put     = i_q & sub == 4'd4;
  assign q_get     = i_q & sub == 4'd5;
  assign q_putsr   = i_q & sub == 4'd6;
  assign q_pushi   = i_q & sub == 4'd7;
  assign m_halt     = i_misc & sub == 4'd1;
  assign m_irq      = i_misc & sub == 4'd2;
  assign m_crcinit  = i_misc & sub == 4'd3;
  assign m_crcchk   = i_misc & sub == 4'd4;
  assign m_stuffclr = i_misc & sub == 4'd5;

  // Source s = IR[17:14]; RAW SAMPLE ignores it (s = 15, NONE).
  assign s       = (i_raw & ~ir[27]) ? 4'd15 : ir[17:14];
  assign dst     = ir[13:10];
  assign s_reg   = s[3:2] == 2'b00;          // R0-R3
  assign s_hq    = s == 4'd4;                // HOSTQ: pop
  assign s_mem   = s == 4'd7;                // MEM at R3, R3 += 1
  assign s_ch    = s[3:2] == 2'b10;          // C0-C3: take
  assign s_bad   = s == 4'd5 | s == 4'd14;
  assign dst_bad = dst == 4'd6 | dst == 4'd12 | dst == 4'd13 | dst == 4'd14;

  // 5.7 / ruling 13: every undefined or reserved code halts.
  assign ill = (op >= 4'd13) | (fr & (s_bad | dst_bad)) |
               (i_setm & mreg >= 5'd14) | (i_alu & sub >= 4'd14) |
               (i_br & cond >= 5'd19) | (i_q & sub[3]) |
               (i_misc & sub >= 4'd6);

  // ------------------------------------------------- 5.4 Hazard set USES
  logic [3:0] uses;
  logic       haz;
  always_comb begin
    uses = '0;
    if (fr & (s_reg | s_mem))      uses |= 4'b0001 << s[1:0];   // s = 7: R3
    if (i_setm & ir[21])           uses |= 4'b0001 << ir[1:0];
    if (i_ldi | i_loop)            uses |= 4'b0001 << ir[27:26];
    if (i_alu) begin
      uses |= 4'b0001 << ir[23:22];
      if (!ir[19] && (sub <= 4'd8 || sub == 4'd11)) uses |= 4'b0001 << ir[21:20];
    end
    if (i_mem) begin
      uses |= 4'b0001 << ir[26:25];
      if (!ir[20]) uses |= 4'b0001 << ir[23:22];
    end
    if (q_pop | q_push | q_put | q_get) uses |= 4'b0001 << ir[23:22];
    if (m_crcchk)                  uses |= 4'b0011;
  end
  assign haz = |(uses & d_dbusy);

  // --------------------------------------- 5.2 / 5.11 Wait conditions
  // Sync instructions (5.11) wait for IDLE; with the general park_full wait
  // that is a wait on lane_busy_i (which covers the end of the last BAUD
  // bit, ruling 30).
  logic sync, cv_x, need_pop, need_push, need_take, need_put, dfail;
  logic pkw, idle, hit, t_nz, w_commit, w_start;
  logic [1:0] cidx;
  assign sync = (i_setm & mreg <= 5'd9) | m_halt | m_crcinit | m_crcchk |
                m_stuffclr | (i_alu & (sub == 4'd12 | sub == 4'd13)) |
                q_pushcrc | q_putsr | (i_br & (cond == 5'd5 | cond == 5'd6));

  assign cidx      = i_q ? ir[21:20] : ir[15:14];      // Q ch, or source c
  assign cv_x      = ch_valid_i[cidx];
  assign need_pop  = (fr & s_hq) | q_pop;
  assign need_push = q_push | q_pushts | q_pushcrc | q_pushi;
  assign need_take = (fr & s_ch) | q_get;
  assign need_put  = q_put | q_putsr;
  // Data conditions (5.3): QE, QF, !CV for takes, CV for puts.
  assign dfail = (need_pop & d_qe) | (need_push & d_qf) |
                 (need_take & ~cv_x) | (need_put & cv_x);

  // General condition: every instruction waits on park_full, except WAIT
  // with wsta = 1 (its own timeout may be the parked op).
  assign pkw       = d_pkf & ~(i_wait & d_wsta);

  // 5.5 WAIT
  assign idle     = ~d_busy & ~d_pkf;
  assign hit      = |(d_ev & ir[27:24]);
  assign t_nz     = |ir[11:0];
  assign w_commit = ~pkw & (hit | (d_wsta & idle));
  assign w_start  = ~pkw & ~hit & ~d_wsta & t_nz & idle;

  // =================================================== 4.3 Slot action
  logic elig, act_del, act_vec, act_exec, ex_halt, xb0;
  logic go_fr, go_pin, go_setm, go_ldi, go_alu, go_mem, go_br, go_loop;
  logic go_wait, go_evt, go_q, go_misc;
  logic [3:0] evm, evk;
  assign elig     = d_run & (~d_pause | d_step);
  assign evm      = d_ev & d_armed;
  assign evk      = evm & ~(evm - 4'd1);     // lowest set bit (E0 first)
  assign act_del  = elig & d_rv;
  assign act_vec  = elig & ~d_rv & (|evm);
  assign act_exec = elig & ~d_rv & ~(|evm);
  // EXEC that takes effect (commit, or a WAIT timeout start), per opcode
  // class: act_exec & ~ill & ~(general wait | HAZ | sync & busy | data
  // condition), WAIT per 5.5. Written per class so that each consumer sees
  // only its own class's terms (shallower than one generic commit).
  assign xb0     = act_exec & ~d_pkf;          // general wait (not WAIT)
  assign go_fr   = xb0 & fr & ~haz & ~s_bad & ~dst_bad & ~(s_hq & d_qe) &
                   ~(s_ch & ~cv_x);
  assign go_pin  = xb0 & i_pin;
  assign go_setm = xb0 & i_setm & ~haz & (mreg < 5'd14) & ~((mreg <= 5'd9) & d_busy);
  assign go_ldi  = xb0 & i_ldi & ~haz;
  assign go_alu  = xb0 & i_alu & ~haz & (sub < 4'd14) &
                   ~((sub == 4'd12 | sub == 4'd13) & d_busy);
  assign go_mem  = xb0 & i_mem & ~haz;
  assign go_br   = xb0 & i_br & (cond < 5'd19) & ~((cond == 5'd5 | cond == 5'd6) & d_busy);
  assign go_loop = xb0 & i_loop & ~haz;
  assign go_wait = act_exec & i_wait & (w_commit | w_start);
  assign go_evt  = xb0 & i_evt;
  assign go_q    = xb0 & i_q & ~sub[3] & ~haz & ~dfail & ~((q_pushcrc | q_putsr) & d_busy);
  assign go_misc = xb0 & i_misc & (sub < 4'd6) & ~haz & ~(sync & d_busy);
  // 5.7: HALT once IDLE, or an illegal instruction regardless of waits.
  assign ex_halt = (act_exec & ill) | (go_misc & m_halt);

  // ====================================================== 5.1 DELIVER
  logic del_reg, del_hq, del_cap, del_mem, del_ch, cv_d, del_wait, del_ok;
  assign del_reg  = d_rdst[3:2] == 2'b00;
  assign del_hq   = d_rdst == 4'd4;
  assign del_cap  = d_rdst == 4'd5;
  assign del_mem  = d_rdst == 4'd7;
  assign del_ch   = d_rdst[3:2] == 2'b10;
  assign cv_d     = ch_valid_i[d_rdst[1:0]];
  assign del_wait = ((del_hq | del_cap) & d_qf) | (del_ch & cv_d);
  assign del_ok   = act_del & ~del_wait;

  // ============================================ D stage: register read ports
  // RA: ALU rd, MEM ar, Q reg (IR[23:22]); LOOP/LDI rd (IR[27:26]);
  //     FIELD/RAW source (IR[15:14]; s = 7 gives 3 = R3); SETM IR[1:0];
  //     CRCCHK R0.  RB: ALU rs (IR[21:20]); MEM dr (IR[26:25]); CRCCHK R1.
  // Indices come from IR only (no lane input), so RA/RB settle 4 levels
  // after the clock. A MEM delivery uses R3 directly (d_r[63:48]).
  logic [1:0]  ia, ib;
  logic [15:0] ra, rb;
  always_comb begin
    if (i_alu | i_mem | i_q)           ia = ir[23:22];
    else if (i_ldi | i_loop)           ia = ir[27:26];
    else if (fr)                       ia = ir[15:14];
    else if (i_setm)                   ia = ir[1:0];
    else                               ia = 2'd0;
    if (i_alu)                         ib = ir[21:20];
    else if (i_mem)                    ib = ir[26:25];
    else                               ib = 2'd1;
  end
  assign ra = d_r[16*ia +: 16];
  assign rb = d_r[16*ib +: 16];

  // Post-increment: one incrementer per register index of the D owner, so
  // the index select is not in front of the carry logic. Each is written as
  // a flat AND per bit (c[i] = &a[i-1:0]) for a shallow carry.
  function automatic logic [15:0] inc16(input logic [15:0] a);
    logic [15:0] c;
    for (int i = 0; i < 16; i++) c[i] = &(a | ~((16'h1 << i) - 16'h1));
    inc16 = a ^ c;
  endfunction
  logic [63:0] r_inc;
  for (genvar n = 0; n < 4; n++) begin : g_inc
    assign r_inc[16*n +: 16] = inc16(d_r[16*n +: 16]);
  end

  // ====================================================== 5.6 VECTOR target
  logic [8:0] tgt_k;
  always_comb begin
    tgt_k = '0;
    for (int k = 0; k < 4; k++) tgt_k |= {9{evk[k]}} & d_tgt[9*k +: 9];
  end

  // ================================================== 5.2 Branch decision
  // Z, N are flops written only in the lane's own X cycles (registered).
  logic       taken, jump, pc_we;
  logic [1:0] c11;
  logic [8:0] pc1, npc;
  assign c11 = cond[1:0] + 2'd1;             // cond-11 (11-14), cond-15 (15-18)
  always_comb begin
    case (cond)
      5'd0:    taken = 1'b1;
      5'd1:    taken = d_z;
      5'd2:    taken = ~d_z;
      5'd3:    taken = d_n;
      5'd4:    taken = ~d_n;
      5'd5:    taken = d_m;
      5'd6:    taken = ~d_m;
      5'd7:    taken = d_qe;
      5'd8:    taken = ~d_qe;
      5'd9:    taken = d_qf;
      5'd10:   taken = ~d_qf;
      5'd11, 5'd12, 5'd13, 5'd14: taken = d_ev[c11];
      5'd15, 5'd16, 5'd17, 5'd18: taken = ch_valid_i[c11];
      default: taken = 1'b0;
    endcase
  end
  // LOOP: R[rd] != 1, compared per register and then selected by rd.
  logic [3:0] ne1;
  for (genvar n = 0; n < 4; n++) begin : g_ne1
    assign ne1[n] = d_r[16*n +: 16] != 16'd1;
  end
  assign jump  = (i_br & taken) | (i_loop & ne1[ir[27:26]]);
  assign pc1   = d_pc + 9'd1;
  // PC moves on VECTOR and on every commit except HALT and a WAIT timeout
  // start (PC unchanged there).
  assign pc_we = act_vec | go_fr | go_pin | go_setm | go_ldi | go_alu | go_mem |
                 go_br | go_loop | (go_wait & ~w_start) | go_evt | go_q |
                 (go_misc & ~m_halt);
  assign npc   = act_vec ? tgt_k : jump ? ir[8:0] : pc1;

  // ============================ 5.1 / 5.2 Data-memory and channel requests
  logic do_pop, do_push, do_read, do_write, tag_en, cap_ts;
  logic [31:0] tsw, w0, wword;
  assign do_pop   = (go_fr & s_hq) | (go_q & q_pop);
  assign do_push  = (go_q & need_push) | (del_ok & (del_hq | del_cap));
  assign do_read  = (go_fr & s_mem) | (go_mem & ~ir[27]);
  assign do_write = (go_mem & ir[27]) | (del_ok & del_mem);

  always_comb begin
    dm_op_o = DM_NONE;
    if (do_read)  dm_op_o = DM_READ;
    if (do_write) dm_op_o = DM_WRITE;
    if (do_pop)   dm_op_o = DM_POP;
    if (do_push)  dm_op_o = DM_PUSH;
  end
  assign dm_off_o   = ~(do_read | do_write) ? 7'd0 :
                      d_rv ? d_r[54:48] :              // MEM delivery: R3
                      (i_mem & ir[20]) ? ir[6:0] : ra[6:0];
  assign dm_hmask_o = ~do_write ? 2'b00 : d_rv ? 2'b11 :
                      ir[24] ? 2'b10 : 2'b01;

  // CAP timestamp word (ruling 30, lane.md 7.5).
  // ts_i[31:16] - 1 is formed from ts_i alone, in parallel with the
  // compare; the compare is a 4-level (lt, eq) tree.
  function automatic logic lt16(input logic [15:0] a, input logic [15:0] b);
    logic [15:0] l, e;
    l = ~a & b;
    e = ~(a ^ b);
    for (int w = 1; w < 16; w = w * 2)
      for (int i = 0; i < 16; i += 2 * w) begin
        l[i] = l[i+w] | (e[i+w] & l[i]);
        e[i] = e[i+w] & e[i];
      end
    lt16 = l[0];
  endfunction
  function automatic logic [15:0] dec16(input logic [15:0] a);
    logic [15:0] b;
    for (int i = 0; i < 16; i++) b[i] = ~|(a & ((16'h1 << i) - 16'h1));
    dec16 = a ^ b;
  endfunction
  logic        ts_borrow;
  logic [15:0] ts_hm1;
  assign ts_hm1    = dec16(ts_i[31:16]);
  assign ts_borrow = lt16(ts_i[15:0], d_rts);
  assign tsw       = {ts_borrow ? ts_hm1 : ts_i[31:16], d_rts};
  assign cap_ts = del_cap & ~d_capw;
  // Shared write word for dm_wdata_o and ch_wdata_o (one-hot AND-OR).
  logic [31:0] we0;
  always_comb begin
    we0 = '0;
    if (i_mem)          we0 |= {rb, rb};
    if (q_push | q_put) we0 |= {16'h0, ra};
    if (q_pushts)       we0 |= ts_i;
    if (q_pushcrc)      we0 |= d_crc;
    if (q_putsr)        we0 |= d_sr;
    if (q_pushi)        we0 |= {24'h0, ir[15:8]};
  end
  // Delivery word (d_rv) or tagged instruction word; no lane input other
  // than res_valid steers the select, the commit only gates the outputs.
  assign tag_en = (q_push | q_pushts | q_pushcrc | q_pushi) & (|ir[7:0]);
  assign w0     = tag_en ? {ir[7:0], we0[23:0]} : we0;
  assign wword  = d_rv ? (cap_ts ? tsw : d_rdata) : w0;
  assign dm_wdata_o = (do_write | do_push) ? wword : 32'h0;

  assign ch_put_o   = (go_q & need_put) | (del_ok & del_ch);
  assign ch_get_o   = (go_fr & s_ch) | (go_q & q_get);
  assign ch_idx_o   = (ch_put_o | ch_get_o) ? (d_rv ? d_rdst[1:0] : cidx) : 2'd0;
  assign ch_wdata_o = ch_put_o ? wword : 32'h0;

  // ============================ End-of-D register write (D write bus)
  // One D write per slot: LDI, GET, delivery to Rd, or post-increment.
  // Data are selected by decode only (dbus_n); the commit decision enters
  // only through the write enable dwe.
  logic        isinc, d_wgo;
  logic [1:0]  widx;
  logic [3:0]  dwe;
  logic [15:0] dbus_c;
  logic [63:0] dbus;
  logic [15:0] ch_sel;
  logic        postinc_x;
  assign ch_sel    = ch_data_i[32*ir[21:20] +: 16];        // GET: bits 15:0
  assign postinc_x = (i_mem & ~ir[20] & ir[21]) | (fr & s_mem);
  assign isinc     = d_rv ? del_mem : postinc_x;
  assign dbus_c    = d_rv ? d_rdata[15:0] : i_ldi ? ir[15:0] : ch_sel;
  for (genvar n = 0; n < 4; n++) begin : g_dbus
    assign dbus[16*n +: 16] = isinc ? r_inc[16*n +: 16] : dbus_c;
  end
  assign widx  = d_rv ? (del_mem ? 2'd3 : d_rdst[1:0]) :
                 i_ldi ? ir[27:26] : q_get ? ir[23:22] : ia;
  // = (del_ok & (del_reg | del_mem)) | go_ldi | (go_q & q_get) |
  //   (go_mem & post-inc) | (go_fr & s_mem), written flat.
  assign d_wgo = elig & (d_rv ? ((del_reg | del_mem) & ~del_wait) :
                 (~(|evm) & ~d_pkf & ~haz &
                  (i_ldi | (q_get & cv_x) | (i_mem & ~ir[20] & ir[21]) |
                   (fr & s_mem & ~dst_bad))));
  assign dwe   = {4{d_wgo}} & (4'b0001 << widx);

  // ===================================== D/X pipeline register (section 6)
  logic        x_park, x_ack, x_abort, x_cfg, x_flag;
  logic [1:0]  x_cmd, x_wr;                  // x_wr: 0 none, 1 ALU, 2 lo, 3 hi
  logic [3:0]  x_evclr;
  logic [15:0] xa, xb;
  logic [13:0] xc;

  logic        n_park, n_ack, n_abort, n_cfg, n_flag;
  logic [1:0]  n_cmd, n_wr;
  logic [3:0]  n_evclr;
  logic [15:0] n_xa, n_xb;
  logic [13:0] n_xc;
  assign n_park  = go_fr | go_pin | (go_wait & w_start);
  assign n_ack   = del_ok & ~cap_ts;
  assign n_abort = act_vec | (go_wait & hit & d_wsta);
  assign n_evclr = act_vec ? evk :
                   (go_evt & ir[24]) ? (4'b0001 << ir[27:26]) : 4'b0000;
  assign n_cfg   = go_setm;
  assign n_cmd   = ~go_misc ? 2'd0 : m_crcinit ? 2'd1 : m_crcchk ? 2'd2 :
                   m_stuffclr ? 2'd3 : 2'd0;
  assign n_wr    = ((go_alu & sub != 4'd3) | go_loop) ? 2'd1 :
                   (go_mem & ~ir[27]) ? {1'b1, ir[24]} :
                   (go_q & q_pop) ? 2'd2 : 2'd0;
  assign n_flag  = go_alu;
  // Payload.
  assign n_xa = (i_setm & ~ir[21]) ? ir[15:0] : ra;
  always_comb begin
    // ALU B; for the shared adder SUB/CMP carry ~B, INC 0, DEC (and LOOP)
    // all ones, so the X stage needs no operand mux (5.8).
    if (i_alu)
      case (sub)
        4'd2, 4'd3: n_xb = ~(ir[19] ? ir[15:0] : rb);
        4'd9:       n_xb = 16'h0000;
        4'd10:      n_xb = 16'hFFFF;
        4'd12:      n_xb = d_sr[15:0];
        4'd13:      n_xb = d_sr[31:16];
        default:    n_xb = ir[19] ? ir[15:0] : rb;
      endcase
    else if (i_loop) n_xb = 16'hFFFF;
    else if (i_wait) n_xb = {4'h0, ir[11:0]};     // timeout PIN {16'h0, T}
    else if (i_misc) n_xb = rb;                   // CRCCHK: R1
    else             n_xb = ir[15:0];             // park ctl[15:0]
  end
  always_comb begin
    if (i_wait)      n_xc = {2'd2, 12'h000};      // PIN, ctl[27:16] = 0
    else if (fr | i_pin) n_xc = {ir[29:28], ir[27:16]};
    else if (i_loop) n_xc = {ir[27:26], 4'd10, ir[23:16]};  // DEC into rd
    else if (i_mem)  n_xc = {ir[26:25], ir[27:16]};
    else             n_xc = {ir[23:22], ir[27:16]};   // ALU rd, POP reg, SETM
  end

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      {x_park, x_ack, x_abort, x_cfg, x_flag} <= '0;
      x_cmd <= '0; x_wr <= '0; x_evclr <= '0;
      xa <= '0; xb <= '0; xc <= '0;
    end else begin
      {x_park, x_ack, x_abort, x_cfg, x_flag} <= {n_park, n_ack, n_abort, n_cfg, n_flag};
      x_cmd <= n_cmd; x_wr <= n_wr; x_evclr <= n_evclr;
      xa <= n_xa; xb <= n_xb; xc <= n_xc;
    end

  // ======================================================= 5.8 ALU (X)
  // xb already holds the adder operand (~B for SUB/CMP, 0 for INC, FFFF for
  // DEC/LOOP). One Kogge-Stone adder serves ADD, SUB, CMP, INC, DEC, ADDC.
  logic [3:0]  aop;
  logic        a_arith, a_sub, a_addc, a_and, a_or, a_xor, a_shl, a_shr, a_mov;
  logic        cin_c, cin, cout, z_ar, z_lg, z_y, n_y;
  logic [15:0] g, p, gg, pp, carry, sum, sh_l, sh_r, y_lg, y;
  assign aop     = xc[11:8];
  assign a_sub   = aop == 4'd2 | aop == 4'd3;
  assign a_addc  = aop == 4'd11;
  assign a_arith = aop == 4'd1 | a_sub | aop == 4'd9 | aop == 4'd10;   // not ADDC
  assign a_and   = aop == 4'd4;
  assign a_or    = aop == 4'd5;
  assign a_xor   = aop == 4'd6;
  assign a_shl   = aop == 4'd7;
  assign a_shr   = aop == 4'd8;
  assign a_mov   = aop == 4'd0 | aop == 4'd12 | aop == 4'd13;
  assign cin_c   = a_sub | aop == 4'd9;       // SUB, CMP, INC
  assign g = xa & xb;
  assign p = xa ^ xb;
  // gg[i], pp[i]: group generate/propagate of bits [i:0].
  always_comb begin
    logic [15:0] g1, p1;
    gg = g; pp = p;
    for (int d = 1; d < 16; d = d * 2) begin
      g1 = gg; p1 = pp;
      for (int i = d; i < 16; i++) begin
        g1[i] = gg[i] | (pp[i] & gg[i-d]);
        p1[i] = pp[i] & pp[i-d];
      end
      gg = g1; pp = p1;
    end
  end
  // ADDC: the end-around carry is the carry out of A + B, gg[15]; feeding it
  // in as carry-in gives s[15:0] + s[16] in the same adder.
  assign cin   = cin_c | (a_addc & gg[15]);
  assign carry = {gg[14:0] | (pp[14:0] & {15{cin}}), cin};
  assign sum   = p ^ carry;
  assign cout  = gg[15] | (pp[15] & cin_c);   // SUB/CMP only
  assign sh_l  = xb[4] ? 16'h0 : (xa << xb[3:0]);
  assign sh_r  = xb[4] ? 16'h0 : (xa >> xb[3:0]);
  // Non-adder results, one-hot AND-OR.
  assign y_lg  = ({16{a_and}} & g) | ({16{a_or}} & (xa | xb)) |
                 ({16{a_xor}} & p) | ({16{a_shl}} & sh_l) |
                 ({16{a_shr}} & sh_r) | ({16{a_mov}} & xb);
  assign y     = (a_arith | a_addc) ? sum : y_lg;
  // Z without the carry chain. For y = A + O + cin (ADD, SUB, CMP, INC, DEC):
  // y == 0 iff p[0] == cin and p[i] == (A | O)[i-1] for i >= 1.
  // ADDC: y == 0 iff A == B == 0 (s[16] = 1 gives y >= 1).
  assign z_ar  = ~|(p ^ {xa[14:0] | xb[14:0], cin_c});
  assign z_lg  = ~|y_lg;
  assign z_y   = a_arith ? z_ar : a_addc ? ~|(xa | xb) : z_lg;
  assign n_y   = a_sub ? ~cout : y[15];       // SUB/CMP: borrow (A < B)

  // X write bus: ALU result or the loaded half of dm_rdata_i (one-hot).
  logic [15:0] xbus;
  logic [3:0]  xwe;
  assign xwe  = {4{x_wr != 2'd0}} & (4'b0001 << xc[13:12]);
  assign xbus = ({16{x_wr == 2'd1}} & y) | ({16{x_wr == 2'd2}} & dm_rdata_i[15:0]) |
                ({16{x_wr == 2'd3}} & dm_rdata_i[31:16]);

  // ===================================== 6. X-cycle outputs to the lanes
  for (genvar L = 0; L < NL; L++) begin : g_xo
    assign lane_park_we_o[L]      = x_park  & xl[L];
    assign lane_res_ack_o[L]      = x_ack   & xl[L];
    assign lane_abort_o[L]        = x_abort & xl[L];
    assign lane_cfg_we_o[L]       = x_cfg   & xl[L];
    assign lane_ev_clr_o[4*L +: 4] = x_evclr & {4{xl[L]}};
    assign lane_cmd_o[2*L +: 2]   = x_cmd   & {2{xl[L]}};
  end
  assign lane_park_op_o  = xc[13:12];
  assign lane_park_ctl_o = {xc[11:0], xb};
  assign lane_cfg_reg_o  = xc[11:7];
  assign lane_cfg_hi_o   = xc[6];
  assign lane_cfg_data_o = xa;
  assign lane_cmd_data_o = {xb, xa};

  // Park data SRC(s) (5.2), muxed in X from the payload, dm_rdata_i, ch_data_i.
  logic [3:0] p_s;
  logic       p_none;
  always_comb begin
    p_s    = {xc[1:0], xb[15:14]};           // ctl[17:14]
    p_none = (xc[13:12] == 2'd2) | (xc[13:12] == 2'd1 & ~xc[11]);  // PIN, RAW SAMPLE
    lane_park_data_o = '0;
    if (!p_none) begin
      if (p_s[3:2] == 2'b00)            lane_park_data_o = {16'h0, xa};
      if (p_s == 4'd4 || p_s == 4'd7)   lane_park_data_o = dm_rdata_i;
      if (p_s[3:2] == 2'b10)            lane_park_data_o = ch_data_i[32*p_s[1:0] +: 32];
      if (p_s == 4'd12)                 lane_park_data_o = {23'h0, xb[8:0]};
    end
  end

  // ======================================== 5.9 Host control (ph = 1 only)
  logic                 hw, irq_w;
  logic [NL-1:0]        ctrl_w, pc_w;
  assign hw    = hreg_we_i & ph[1];
  assign irq_w = hw & hreg_addr_i == 7'h04;
  for (genvar L = 0; L < NL; L++) begin : g_hw
    assign ctrl_w[L] = hw & hreg_addr_i == 7'(8'h10 + 8*L);
    assign pc_w[L]   = hw & hreg_addr_i == 7'(8'h11 + 8*L);
  end

  // ===================================== 3. Per-lane state, all updates
  // End-of-D next values of the D owner, each one enable and one value
  // (the D-cycle cases are mutually exclusive):
  //   qwait: 5.3, EXEC -> data condition failed (0 on halt), DELIVER -> its
  //          wait, VECTOR -> 0, NONE keeps (enable = elig).
  //   wsta : 5.5 WAIT commit/start, cleared by VECTOR and halt (5.6, 5.7).
  //   capw : 5.1 CAP pairing, cleared by VECTOR and halt.
  //   armed/tgt: EVT (5.2); armed cleared by halt.
  logic d_qw, wsta_en, wsta_d, capw_en, capw_d, attn_set;
  logic [3:0] arm_en, evt_k;
  assign d_qw     = d_rv ? del_wait : (~(|evm) & dfail & ~ill);
  assign wsta_en  = go_wait | act_vec | ex_halt;
  assign wsta_d   = go_wait & w_start;
  assign capw_en  = (del_ok & del_cap) | act_vec | ex_halt;
  assign capw_d   = del_ok & del_cap & ~d_capw;
  assign evt_k    = {4{go_evt}} & (4'b0001 << ir[27:26]);
  assign arm_en   = evt_k | {4{ex_halt}};
  assign attn_set = ex_halt | (go_misc & m_irq);

  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      pc <= '0; r <= '0; zf <= '0; nf <= '0;
      run <= '0; pause <= '0; step <= '0; attn <= '0;
      armed <= '0; tgt <= '0; wsta <= '0; capw <= '0; qwait <= '0;
    end else begin
      for (int L = 0; L < NL; L++) begin
        // ---- End of L's D cycle (4.3, 5.1-5.7). Host writes (5.9) are only
        // at ph = 1, where no lane has its D cycle.
        if (dl[L]) begin
          if (elig)     step[L]  <= 1'b0;
          if (pc_we)    pc[9*L +: 9] <= npc;
          if (elig)     qwait[L] <= d_qw;
          if (wsta_en)  wsta[L]  <= wsta_d;
          if (capw_en)  capw[L]  <= capw_d;
          if (ex_halt)  run[L]   <= 1'b0;
          if (attn_set) attn[L]  <= 1'b1;
          for (int k = 0; k < 4; k++) begin
            if (arm_en[k]) armed[4*L + k]       <= evt_k[k] & ir[25];
            if (evt_k[k])  tgt[36*L + 9*k +: 9] <= ir[8:0];
          end
        end else begin
          if (ctrl_w[L]) begin
            run[L] <= hreg_wdata_i[0]; pause[L] <= hreg_wdata_i[1];
            step[L] <= hreg_wdata_i[2];
            if (!hreg_wdata_i[0]) begin
              armed[4*L +: 4] <= 4'h0; wsta[L] <= 1'b0; capw[L] <= 1'b0;
              qwait[L] <= 1'b0;
            end
          end
          if (pc_w[L] && !run[L]) pc[9*L +: 9] <= hreg_wdata_i[8:0];
          if (irq_w && hreg_wdata_i[L]) attn[L] <= 1'b0;
        end
        // ---- Register file: one write per lane per cycle (sequencer.md 10),
        // end of D (D write bus) or end of X (ALU, LOOP, MEM load, POP). The
        // data are chosen early by phase, the late enable picks hold.
        for (int n = 0; n < 4; n++)
          if ((dl[L] && dwe[n]) || (xl[L] && xwe[n]))
            r[64*L + 16*n +: 16] <= dl[L] ? dbus[16*n +: 16] : xbus;
        // ---- End of L's X cycle: flags (5.8).
        if (xl[L] && x_flag) begin zf[L] <= z_y; nf[L] <= n_y; end
      end
    end

  // ===================================== 2.5 Host read mux (combinational)
  always_comb begin
    hreg_rdata_o = '0;
    if (hreg_addr_i == 7'h04) hreg_rdata_o[NL+7:0] = {run, {(8-NL){1'b0}}, attn};
    for (int L = 0; L < NL; L++)
      if (hreg_addr_i[6:3] == 4'(2 + L))
        case (hreg_addr_i[2:0])
          3'd0: hreg_rdata_o = {22'h0, qwait[L], capw[L], wsta[L],
                                lane_res_valid_i[L], lane_park_full_i[L],
                                lane_busy_i[L], attn[L], step[L], pause[L], run[L]};
          3'd1: hreg_rdata_o = {23'h0, pc[9*L +: 9]};
          3'd2: hreg_rdata_o = {19'h0, armed[4*L +: 4], qf_i[L], qe_i[L],
                                lane_ev_pend_i[4*L +: 4], lane_m_i[L], nf[L], zf[L]};
          3'd3: hreg_rdata_o = r[64*L +: 32];
          3'd4: hreg_rdata_o = r[64*L + 32 +: 32];
          default: hreg_rdata_o = '0;
        endcase
  end

  // ===================================== Registered outputs and observation
  assign run_o            = run;
  assign attn_o           = attn;
  assign lane_run_o       = run;
  assign lane_data_wait_o = qwait;
  assign obs_pc_o    = pc;
  assign obs_r_o     = r;
  assign obs_z_o     = zf;
  assign obs_n_o     = nf;
  assign obs_pause_o = pause;
  assign obs_step_o  = step;
  assign obs_armed_o = armed;
  assign obs_tgt_o   = tgt;
  assign obs_wsta_o  = wsta;
  assign obs_capw_o  = capw;

  // hreg_wdata_i[31:9] are not used by any sequencer register.
  logic unused_ok;
  assign unused_ok = &{1'b0, hreg_wdata_i[31:9]};
endmodule
