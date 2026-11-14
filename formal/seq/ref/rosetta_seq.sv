`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/sequencer.md (as amended by
//  docs/decisions.md rulings 13, 26, 30) written by the formal property
//  writer, only to show that formal/seq/*.sv can PASS and that the
//  properties are non-vacuous (the planted bugs below make tasks FAIL).
//  The real block is src/rosetta_seq.sv, written independently.
//  Do not integrate, synthesise or copy this file.
//
//  Structure: one-hot wheel, IR, per-lane state, a D-stage decision for the
//  D owner, a D/X register rewritten every cycle (strobes come from it),
//  X-stage register/flag writes, host writes at ph = 1.
//
//  Planted bugs (mutation testing), enabled one at a time with a define:
//    MUT_STROBE_SLOT    the event-clear strobe goes to lane x+1 (wrong slot)
//    MUT_HALTED_STROBE  deliveries (and their ack strobe) are made to a
//                       halted lane: eligibility ignores run for DELIVER
//    MUT_REGION         a MEM store request is issued one cycle late (in the
//                       next lane's D cycle, so it lands in that region)
//    MUT_ALU_SUB        SUB computes A + B
//    MUT_FLAG_SHL       SHL does not update Z/N
//    MUT_BR_INV         BR NZ is inverted (behaves as BR Z)
//    MUT_LOOP_OFF1      LOOP branches while R != 0 (off by one)
//    MUT_UNDEF_NOP      opcode 15 executes as a NOP instead of halting
//    MUT_WAIT_PC        a waiting Q POP (queue empty) advances PC
//    MUT_DELIV_LOST     a delivery to HOSTQ does not wait for space (the
//                       push into a full queue is dropped: result lost)
//    MUT_CAP_SWAP       CAP pushes the value word first, then the timestamp
//    MUT_CAP_NOBORROW   CAP timestamp word without the borrow of ruling 30
//    MUT_PARK_FULL      FIELD does not wait while the parked register is full
//    MUT_SYNC_SETM      SETM of registers 0-9 does not wait for IDLE
//    MUT_HOST_PC_RUN    host PC write taken while the lane runs
//    MUT_VEC_PRIO       VECTOR picks the highest armed pending event
//    MUT_ADDC           ADDC without the end-around carry
//    MUT_STEP_KEEP      step is not cleared by an eligible D cycle
//    MUT_RESET_RUN      lane 1 comes out of reset running
//    MUT_QWAIT          qwait is not set when a POP waits for data
// ===========================================================================

module rosetta_seq (
    input  wire          clk,
    input  wire          rst_n,
    output logic [4:0]   ph_o,
    output logic [8:0]   pm_addr_o,
    input  wire  [31:0]  pm_rdata_i,
    output logic [2:0]   dm_op_o,
    output logic [6:0]   dm_off_o,
    output logic [1:0]   dm_hmask_o,
    output logic [31:0]  dm_wdata_o,
    input  wire  [31:0]  dm_rdata_i,
    input  wire  [3:0]   qe_i,
    input  wire  [3:0]   qf_i,
    output logic         ch_put_o,
    output logic         ch_get_o,
    output logic [1:0]   ch_idx_o,
    output logic [31:0]  ch_wdata_o,
    input  wire  [3:0]   ch_valid_i,
    input  wire  [127:0] ch_data_i,
    input  wire  [31:0]  ts_i,
    input  wire          hreg_we_i,
    input  wire  [6:0]   hreg_addr_i,
    input  wire  [31:0]  hreg_wdata_i,
    output logic [31:0]  hreg_rdata_o,
    output logic [3:0]   run_o,
    output logic [3:0]   attn_o,
    output logic [3:0]   lane_run_o,
    output logic [3:0]   lane_park_we_o,
    output logic [1:0]   lane_park_op_o,
    output logic [27:0]  lane_park_ctl_o,
    output logic [31:0]  lane_park_data_o,
    output logic [3:0]   lane_res_ack_o,
    output logic [3:0]   lane_abort_o,
    output logic [15:0]  lane_ev_clr_o,
    output logic [3:0]   lane_cfg_we_o,
    output logic [4:0]   lane_cfg_reg_o,
    output logic         lane_cfg_hi_o,
    output logic [15:0]  lane_cfg_data_o,
    output logic [7:0]   lane_cmd_o,
    output logic [31:0]  lane_cmd_data_o,
    output logic [3:0]   lane_data_wait_o,
    input  wire  [3:0]   lane_busy_i,
    input  wire  [3:0]   lane_park_full_i,
    input  wire  [3:0]   lane_res_valid_i,
    input  wire  [15:0]  lane_res_dst_i,
    input  wire  [127:0] lane_res_data_i,
    input  wire  [63:0]  lane_res_ts_i,
    input  wire  [15:0]  lane_dst_busy_i,
    input  wire  [3:0]   lane_m_i,
    input  wire  [15:0]  lane_ev_pend_i,
    input  wire  [127:0] lane_sr_i,
    input  wire  [127:0] lane_crc_i,
    output logic [35:0]  obs_pc_o,
    output logic [255:0] obs_r_o,
    output logic [3:0]   obs_z_o,
    output logic [3:0]   obs_n_o,
    output logic [3:0]   obs_pause_o,
    output logic [3:0]   obs_step_o,
    output logic [15:0]  obs_armed_o,
    output logic [143:0] obs_tgt_o,
    output logic [3:0]   obs_wsta_o,
    output logic [3:0]   obs_capw_o
);
    localparam [2:0] NONE = 3'd0, READ = 3'd1, WRITE = 3'd2, POP = 3'd3, PUSH = 3'd4;

    // ------------------------------------------------------------- state
    logic [4:0]   ph;
    logic [31:0]  ir;
    logic [35:0]  pc;          // 9 per lane
    logic [255:0] rf;          // 16 x 4 per lane
    logic [3:0]   z, n, run, pause, step, attn, wsta, capw, qwait;
    logic [15:0]  armed;
    logic [143:0] tgt;

    // D/X register
    logic         xv;  logic [1:0] xl;
    logic         x_park; logic [1:0] x_pop; logic [27:0] x_pctl; logic x_pmem; logic [31:0] x_pdata;
    logic         x_ack, x_abort; logic [3:0] x_evclr;
    logic         x_cfg; logic [4:0] x_creg; logic x_chi; logic [15:0] x_cdata;
    logic [1:0]   x_cmd; logic [31:0] x_cmdd;
    logic         x_wen; logic [1:0] x_wreg; logic [1:0] x_wsrc; logic [15:0] x_wval;
    logic         x_fen, x_z, x_n;
`ifdef MUT_REGION
    logic         mr_v; logic [6:0] mr_off; logic [1:0] mr_hm; logic [31:0] mr_wd;
`endif

    // ------------------------------------------------------------- D stage
    wire        dv = !ph[1];                     // a lane owns D
    wire [1:0]  dl = ph[2] ? 2'd0 : ph[3] ? 2'd1 : ph[4] ? 2'd2 : 2'd3;

    wire [8:0]  cpc  = pc[9*dl +: 9];
    wire [63:0] cr   = rf[64*dl +: 64];
    wire [15:0] R0 = cr[15:0], R1 = cr[31:16], R2 = cr[47:32], R3 = cr[63:48];
    function automatic [15:0] RG(input [63:0] v, input [1:0] i);
        case (i) 2'd0: RG = v[15:0]; 2'd1: RG = v[31:16]; 2'd2: RG = v[47:32]; default: RG = v[63:48]; endcase
    endfunction
    function automatic [31:0] TAG(input [31:0] w, input [7:0] t);
        TAG = (t == 8'h0) ? w : {t, w[23:0]};
    endfunction

    wire        celig = dv && run[dl] && (!pause[dl] || step[dl]);
`ifdef MUT_HALTED_STROBE
    wire        celig_v = dv && (!pause[dl] || step[dl]);   // deliveries ignore run
`else
    wire        celig_v = celig;
`endif
    wire        busy  = lane_busy_i[dl];
    wire        pfull = lane_park_full_i[dl];
    wire        idle  = !busy && !pfull;
    wire        rv    = lane_res_valid_i[dl];
    wire [3:0]  rdst  = lane_res_dst_i[4*dl +: 4];
    wire [31:0] rdat  = lane_res_data_i[32*dl +: 32];
    wire [15:0] rts   = lane_res_ts_i[16*dl +: 16];
    wire [3:0]  dbz   = lane_dst_busy_i[4*dl +: 4];
    wire [3:0]  evp   = lane_ev_pend_i[4*dl +: 4];
    wire [3:0]  arm   = armed[4*dl +: 4];
    wire [3:0]  pend  = evp & arm;
    wire        qe    = qe_i[dl], qf = qf_i[dl];
    wire [31:0] sr    = lane_sr_i[32*dl +: 32];
    wire [31:0] crc   = lane_crc_i[32*dl +: 32];
`ifdef MUT_VEC_PRIO
    wire [1:0]  vk    = pend[3] ? 2'd3 : pend[2] ? 2'd2 : pend[1] ? 2'd1 : 2'd0;
`else
    wire [1:0]  vk    = pend[0] ? 2'd0 : pend[1] ? 2'd1 : pend[2] ? 2'd2 : 2'd3;
`endif

    wire [3:0]  opc = ir[31:28];
    wire [3:0]  so  = ir[27:24];

    // D-stage results
    logic [2:0]  d_op; logic [6:0] d_off; logic [1:0] d_hm; logic [31:0] d_wd;
    logic        d_put, d_get; logic [1:0] d_ci; logic [31:0] d_cwd;
    logic        d_pcwe; logic [8:0] d_pc;
    logic        d_rwe;  logic [1:0] d_rreg; logic [15:0] d_rval;
    logic        d_halt, d_irq, d_wsta_set, d_wsta_clr, d_capw_set, d_capw_clr, d_vec;
    logic        d_qw_we, d_qw;
    logic        d_arm_we; logic [1:0] d_ak; logic d_av; logic [8:0] d_at;
    // next D/X
    logic        n_park; logic [1:0] n_pop; logic [27:0] n_pctl; logic n_pmem; logic [31:0] n_pdata;
    logic        n_ack, n_abort; logic [3:0] n_evclr;
    logic        n_cfg; logic [4:0] n_creg; logic n_chi; logic [15:0] n_cdata;
    logic [1:0]  n_cmd; logic [31:0] n_cmdd;
    logic        n_wen; logic [1:0] n_wreg, n_wsrc; logic [15:0] n_wval;
    logic        n_fen, n_z, n_n;

    // scratch
    logic [3:0]  src; logic bad; logic [3:0] use_m; logic wt, dfail, tk;
    logic [15:0] A, B, Y; logic [16:0] S; logic NN;

    always @* begin
        d_op = NONE; d_off = 7'h0; d_hm = 2'b00; d_wd = 32'h0;
        d_put = 1'b0; d_get = 1'b0; d_ci = 2'd0; d_cwd = 32'h0;
        d_pcwe = 1'b0; d_pc = cpc;
        d_rwe = 1'b0; d_rreg = 2'd0; d_rval = 16'h0;
        d_halt = 1'b0; d_irq = 1'b0; d_wsta_set = 1'b0; d_wsta_clr = 1'b0;
        d_capw_set = 1'b0; d_capw_clr = 1'b0; d_vec = 1'b0;
        d_qw_we = 1'b0; d_qw = 1'b0;
        d_arm_we = 1'b0; d_ak = 2'd0; d_av = 1'b0; d_at = 9'h0;
        n_park = 1'b0; n_pop = 2'd0; n_pctl = 28'h0; n_pmem = 1'b0; n_pdata = 32'h0;
        n_ack = 1'b0; n_abort = 1'b0; n_evclr = 4'h0;
        n_cfg = 1'b0; n_creg = 5'h0; n_chi = 1'b0; n_cdata = 16'h0;
        n_cmd = 2'd0; n_cmdd = 32'h0;
        n_wen = 1'b0; n_wreg = 2'd0; n_wsrc = 2'd0; n_wval = 16'h0;
        n_fen = 1'b0; n_z = 1'b0; n_n = 1'b0;
        src = 4'd15; bad = 1'b0; use_m = 4'h0; wt = 1'b0; dfail = 1'b0; tk = 1'b0;
        A = 16'h0; B = 16'h0; Y = 16'h0; S = 17'h0; NN = 1'b0;

        if (!celig_v) begin
            // NONE
        end else if (rv) begin
            // ---------------- DELIVER
            d_qw_we = 1'b1;
            case (rdst)
                4'd0, 4'd1, 4'd2, 4'd3: begin
                    d_rwe = 1'b1; d_rreg = rdst[1:0]; d_rval = rdat[15:0]; n_ack = 1'b1;
                end
                4'd4: begin
`ifdef MUT_DELIV_LOST
                    wt = 1'b0;
`else
                    wt = qf;
`endif
                    if (!wt) begin d_op = PUSH; d_wd = rdat; n_ack = 1'b1; end
                end
                4'd5: begin
                    wt = qf;
                    if (!wt) begin
                        d_op = PUSH;
`ifdef MUT_CAP_SWAP
                        if (!capw[dl]) begin d_wd = rdat; d_capw_set = 1'b1; end
                        else begin d_wd = {ts_i[31:16] - {15'h0, ts_i[15:0] < rts}, rts}; d_capw_clr = 1'b1; n_ack = 1'b1; end
`elsif MUT_CAP_NOBORROW
                        if (!capw[dl]) begin d_wd = {ts_i[31:16], rts}; d_capw_set = 1'b1; end
                        else begin d_wd = rdat; d_capw_clr = 1'b1; n_ack = 1'b1; end
`else
                        if (!capw[dl]) begin d_wd = {ts_i[31:16] - {15'h0, ts_i[15:0] < rts}, rts}; d_capw_set = 1'b1; end
                        else begin d_wd = rdat; d_capw_clr = 1'b1; n_ack = 1'b1; end
`endif
                    end
                end
                4'd7: begin
                    d_op = WRITE; d_off = R3[6:0]; d_hm = 2'b11; d_wd = rdat;
                    d_rwe = 1'b1; d_rreg = 2'd3; d_rval = R3 + 16'd1; n_ack = 1'b1;
                end
                4'd8, 4'd9, 4'd10, 4'd11: begin
                    wt = ch_valid_i[rdst[1:0]];
                    if (!wt) begin d_put = 1'b1; d_ci = rdst[1:0]; d_cwd = rdat; n_ack = 1'b1; end
                end
                default: n_ack = 1'b1;
            endcase
            d_qw = wt;
        end else if (!celig) begin
            // NONE (differs from the first branch only under MUT_HALTED_STROBE)
        end else if (pend != 4'h0) begin
            // ---------------- VECTOR
            d_vec = 1'b1; d_pcwe = 1'b1; d_pc = tgt[36*dl + 9*vk +: 9];
            n_abort = 1'b1; n_evclr = 4'b0001 << vk;
        end else begin
            // ---------------- EXEC
            // source for FIELD / RAW
            src = (opc == 4'd1 && !ir[27]) ? 4'd15 : ir[17:14];
            // illegal codes
            case (opc)
                4'd0, 4'd1: bad = (src == 4'd5) || (src == 4'd14) || (ir[13:10] == 4'd6) ||
                                  (ir[13:10] == 4'd12) || (ir[13:10] == 4'd13) || (ir[13:10] == 4'd14);
                4'd3:  bad = ir[27:23] > 5'd13;
                4'd5:  bad = so > 4'd13;
                4'd7:  bad = ir[27:23] > 5'd18;
                4'd11: bad = so[3];
                4'd12: bad = so > 4'd5;
`ifdef MUT_UNDEF_NOP
                4'd13, 4'd14: bad = 1'b1;
                4'd15: bad = 1'b0;
`else
                4'd13, 4'd14, 4'd15: bad = 1'b1;
`endif
                default: bad = 1'b0;
            endcase
            // register uses
            case (opc)
                4'd0, 4'd1: use_m = (src < 4'd4) ? (4'b0001 << src[1:0]) : (src == 4'd7) ? 4'b1000 : 4'h0;
                4'd3:  use_m = ir[21] ? (4'b0001 << ir[1:0]) : 4'h0;
                4'd4, 4'd8: use_m = 4'b0001 << ir[27:26];
                4'd5:  use_m = (4'b0001 << ir[23:22]) |
                               ((!ir[19] && (so < 4'd9 || so == 4'd11)) ? (4'b0001 << ir[21:20]) : 4'h0);
                4'd6:  use_m = (4'b0001 << ir[26:25]) | (ir[20] ? 4'h0 : (4'b0001 << ir[23:22]));
                4'd11: use_m = (so == 4'd0 || so == 4'd1 || so == 4'd4 || so == 4'd5) ? (4'b0001 << ir[23:22]) : 4'h0;
                4'd12: use_m = (so == 4'd4) ? 4'b0011 : 4'h0;
                default: use_m = 4'h0;
            endcase
            // ALU operands and result
            A = RG(cr, ir[23:22]);
            B = ir[19] ? ir[15:0] : RG(cr, ir[21:20]);
            S = A + B;
            case (so)
                4'd0: Y = B;
                4'd1: Y = S[15:0];
`ifdef MUT_ALU_SUB
                4'd2: Y = A + B;
`else
                4'd2: Y = A - B;
`endif
                4'd3: Y = A - B;
                4'd4: Y = A & B;
                4'd5: Y = A | B;
                4'd6: Y = A ^ B;
                4'd7: Y = (B[4:0] < 5'd16) ? (A << B[3:0]) : 16'h0;
                4'd8: Y = (B[4:0] < 5'd16) ? (A >> B[3:0]) : 16'h0;
                4'd9: Y = A + 16'd1;
                4'd10: Y = A - 16'd1;
`ifdef MUT_ADDC
                4'd11: Y = S[15:0];
`else
                4'd11: Y = S[15:0] + {15'h0, S[16]};
`endif
                4'd12: Y = sr[15:0];
                default: Y = sr[31:16];
            endcase
            NN = (so == 4'd2 || so == 4'd3) ? (A < B) : Y[15];
            // branch condition
            case (ir[27:23])
                5'd0: tk = 1'b1;
`ifdef MUT_BR_INV
                5'd1: tk = z[dl];  5'd2: tk = z[dl];
`else
                5'd1: tk = z[dl];  5'd2: tk = !z[dl];
`endif
                5'd3: tk = n[dl];  5'd4: tk = !n[dl];
                5'd5: tk = lane_m_i[dl]; 5'd6: tk = !lane_m_i[dl];
                5'd7: tk = qe; 5'd8: tk = !qe; 5'd9: tk = qf; 5'd10: tk = !qf;
                5'd11, 5'd12, 5'd13, 5'd14: tk = evp[ir[27:23] - 5'd11];
                5'd15, 5'd16, 5'd17, 5'd18: tk = ch_valid_i[ir[27:23] - 5'd15];
                default: tk = 1'b0;
            endcase
            // wait conditions
            case (opc)
                4'd0, 4'd1: begin
                    dfail = (src == 4'd4 && qe) || (src[3:2] == 2'b10 && !ch_valid_i[src[1:0]]);
`ifdef MUT_PARK_FULL
                    wt = ((use_m & dbz) != 0) || dfail || (opc == 4'd1 && pfull);
`else
                    wt = pfull || ((use_m & dbz) != 0) || dfail;
`endif
                end
                4'd2: wt = pfull;
`ifdef MUT_SYNC_SETM
                4'd3: wt = pfull || (ir[21] && (use_m & dbz) != 0);
`else
                4'd3: wt = pfull || (ir[27:23] < 5'd10 && !idle) || (ir[21] && (use_m & dbz) != 0);
`endif
                4'd4, 4'd6, 4'd8: wt = pfull || ((use_m & dbz) != 0);
                4'd5: wt = pfull || ((use_m & dbz) != 0) || ((so == 4'd12 || so == 4'd13) && !idle);
                4'd7: wt = pfull || ((ir[27:23] == 5'd5 || ir[27:23] == 5'd6) && !idle);
                4'd9: begin
                    if (pfull && !wsta[dl]) wt = 1'b1;
                    else if ((evp & ir[27:24]) != 0) wt = 1'b0;
                    else if (wsta[dl] && idle) wt = 1'b0;
                    else if (!wsta[dl] && ir[11:0] != 12'h0 && idle) wt = 1'b0;
                    else wt = 1'b1;
                end
                4'd10: wt = pfull;
                4'd11: begin
                    case (so)
                        4'd0: dfail = qe;
                        4'd1, 4'd2, 4'd3, 4'd7: dfail = qf;
                        4'd4, 4'd6: dfail = ch_valid_i[ir[21:20]];
                        4'd5: dfail = !ch_valid_i[ir[21:20]];
                        default: dfail = 1'b0;
                    endcase
                    wt = pfull || dfail || ((use_m & dbz) != 0) || ((so == 4'd3 || so == 4'd6) && !idle);
                end
                4'd12: wt = pfull || ((so == 4'd1 || so == 4'd3 || so == 4'd4 || so == 4'd5) && !idle) ||
                            ((use_m & dbz) != 0);
                default: wt = 1'b0;
            endcase
`ifdef MUT_QWAIT
            if (opc == 4'd11 && so == 4'd0) dfail = 1'b0;
`endif

            if (bad) begin
                d_halt = 1'b1;
            end else if (wt) begin
                d_qw_we = 1'b1; d_qw = dfail;
`ifdef MUT_WAIT_PC
                if (opc == 4'd11 && so == 4'd0 && qe && !pfull) begin d_pcwe = 1'b1; d_pc = cpc + 9'd1; end
`endif
            end else begin
                d_qw_we = 1'b1; d_qw = 1'b0;
                d_pcwe = 1'b1; d_pc = cpc + 9'd1;
                case (opc)
                    4'd0, 4'd1: begin
                        if (src == 4'd4) d_op = POP;
                        if (src == 4'd7) begin d_op = READ; d_off = R3[6:0]; d_rwe = 1'b1; d_rreg = 2'd3; d_rval = R3 + 16'd1; end
                        if (src[3:2] == 2'b10) begin d_get = 1'b1; d_ci = src[1:0]; end
                        n_park = 1'b1; n_pop = opc[1:0]; n_pctl = ir[27:0];
                        n_pmem = (src == 4'd4 || src == 4'd7);
                        case (src)
                            4'd0, 4'd1, 4'd2, 4'd3: n_pdata = {16'h0, RG(cr, src[1:0])};
                            4'd8, 4'd9, 4'd10, 4'd11: n_pdata = ch_data_i[32*src[1:0] +: 32];
                            4'd12: n_pdata = {23'h0, ir[8:0]};
                            default: n_pdata = 32'h0;
                        endcase
                    end
                    4'd2: begin n_park = 1'b1; n_pop = 2'd2; n_pctl = ir[27:0]; end
                    4'd3: begin
                        n_cfg = 1'b1; n_creg = ir[27:23]; n_chi = ir[22];
                        n_cdata = ir[21] ? RG(cr, ir[1:0]) : ir[15:0];
                    end
                    4'd4: begin d_rwe = 1'b1; d_rreg = ir[27:26]; d_rval = ir[15:0]; end
                    4'd5: begin
                        n_wen = (so != 4'd3); n_wreg = ir[23:22]; n_wsrc = 2'd0; n_wval = Y;
`ifdef MUT_FLAG_SHL
                        n_fen = (so != 4'd7);
`else
                        n_fen = 1'b1;
`endif
                        n_z = (Y == 16'h0); n_n = NN;
                    end
                    4'd6: begin
                        d_off = ir[20] ? ir[6:0] : RG(cr, ir[23:22]) & 16'h007f;
                        if (ir[27]) begin
                            d_op = WRITE; d_hm = ir[24] ? 2'b10 : 2'b01;
                            d_wd = {RG(cr, ir[26:25]), RG(cr, ir[26:25])};
                        end else begin
                            d_op = READ;
                            n_wen = 1'b1; n_wreg = ir[26:25]; n_wsrc = ir[24] ? 2'd2 : 2'd1;
                        end
                        if (!ir[20] && ir[21]) begin d_rwe = 1'b1; d_rreg = ir[23:22]; d_rval = RG(cr, ir[23:22]) + 16'd1; end
                    end
                    4'd7: begin d_pc = tk ? ir[8:0] : cpc + 9'd1; end
                    4'd8: begin
`ifdef MUT_LOOP_OFF1
                        d_pc = (RG(cr, ir[27:26]) != 16'd0) ? ir[8:0] : cpc + 9'd1;
`else
                        d_pc = (RG(cr, ir[27:26]) != 16'd1) ? ir[8:0] : cpc + 9'd1;
`endif
                        n_wen = 1'b1; n_wreg = ir[27:26]; n_wsrc = 2'd0; n_wval = RG(cr, ir[27:26]) - 16'd1;
                    end
                    4'd9: begin
                        if ((evp & ir[27:24]) != 0) begin
                            d_wsta_clr = 1'b1; n_abort = wsta[dl];
                        end else if (wsta[dl]) begin
                            d_wsta_clr = 1'b1;
                        end else begin
                            d_pcwe = 1'b0; d_wsta_set = 1'b1;
                            n_park = 1'b1; n_pop = 2'd2; n_pctl = {16'h0, ir[11:0]};
                        end
                    end
                    4'd10: begin
                        d_arm_we = 1'b1; d_ak = ir[27:26]; d_av = ir[25]; d_at = ir[8:0];
                        n_evclr = ir[24] ? (4'b0001 << ir[27:26]) : 4'h0;
                    end
                    4'd11: begin
                        case (so)
                            4'd0: begin d_op = POP; n_wen = 1'b1; n_wreg = ir[23:22]; n_wsrc = 2'd1; end
                            4'd1: begin d_op = PUSH; d_wd = TAG({16'h0, RG(cr, ir[23:22])}, ir[7:0]); end
                            4'd2: begin d_op = PUSH; d_wd = TAG(ts_i, ir[7:0]); end
                            4'd3: begin d_op = PUSH; d_wd = TAG(crc, ir[7:0]); end
                            4'd4: begin d_put = 1'b1; d_ci = ir[21:20]; d_cwd = {16'h0, RG(cr, ir[23:22])}; end
                            4'd5: begin d_get = 1'b1; d_ci = ir[21:20];
                                        d_rwe = 1'b1; d_rreg = ir[23:22]; d_rval = ch_data_i[32*ir[21:20] +: 16]; end
                            4'd6: begin d_put = 1'b1; d_ci = ir[21:20]; d_cwd = sr; end
                            default: begin d_op = PUSH; d_wd = TAG({24'h0, ir[15:8]}, ir[7:0]); end
                        endcase
                    end
                    4'd12: begin
                        case (so)
                            4'd1: begin d_halt = 1'b1; d_pcwe = 1'b0; end
                            4'd2: d_irq = 1'b1;
                            4'd3: n_cmd = 2'd1;
                            4'd4: begin n_cmd = 2'd2; n_cmdd = {R1, R0}; end
                            4'd5: n_cmd = 2'd3;
                            default: ;
                        endcase
                    end
                    default: ;
                endcase
            end
        end
    end

    // MUT_REGION: delay MEM store requests by one cycle
    logic [2:0] o_op; logic [6:0] o_off; logic [1:0] o_hm; logic [31:0] o_wd;
    always @* begin
        o_op = d_op; o_off = d_off; o_hm = d_hm; o_wd = d_wd;
`ifdef MUT_REGION
        if (d_op == WRITE && d_hm != 2'b11) begin o_op = NONE; o_off = 7'h0; o_hm = 2'b00; o_wd = 32'h0; end
        if (mr_v && dv) begin o_op = WRITE; o_off = mr_off; o_hm = mr_hm; o_wd = mr_wd; end
`endif
    end
    assign dm_op_o    = o_op;
    assign dm_off_o   = o_off;
    assign dm_hmask_o = o_hm;
    assign dm_wdata_o = o_wd;
    assign ch_put_o   = d_put;
    assign ch_get_o   = d_get;
    assign ch_idx_o   = d_ci;
    assign ch_wdata_o = d_cwd;

    // ------------------------------------------------------------- outputs
    assign ph_o       = ph;
    assign pm_addr_o  = ph[0] ? pc[8:0] : ph[1] ? pc[17:9] : ph[2] ? pc[26:18] : ph[3] ? pc[35:27] : 9'h0;
    assign run_o      = run;
    assign lane_run_o = run;
    assign attn_o     = attn;
    assign lane_data_wait_o = qwait;
    wire [3:0] xsel   = xv ? (4'b0001 << xl) : 4'h0;
    assign lane_park_we_o   = x_park ? xsel : 4'h0;
    assign lane_park_op_o   = x_pop;
    assign lane_park_ctl_o  = x_pctl;
    assign lane_park_data_o = x_pmem ? dm_rdata_i : x_pdata;
    assign lane_res_ack_o   = x_ack ? xsel : 4'h0;
    assign lane_abort_o     = x_abort ? xsel : 4'h0;
`ifdef MUT_STROBE_SLOT
    assign lane_ev_clr_o    = xv ? ({12'h0, x_evclr} << (4*(xl + 2'd1))) : 16'h0;
`else
    assign lane_ev_clr_o    = xv ? ({12'h0, x_evclr} << (4*xl)) : 16'h0;
`endif
    assign lane_cfg_we_o    = x_cfg ? xsel : 4'h0;
    assign lane_cfg_reg_o   = x_creg;
    assign lane_cfg_hi_o    = x_chi;
    assign lane_cfg_data_o  = x_cdata;
    assign lane_cmd_o       = xv ? ({6'h0, x_cmd} << (2*xl)) : 8'h0;
    assign lane_cmd_data_o  = x_cmdd;
    assign obs_pc_o    = pc;
    assign obs_r_o     = rf;
    assign obs_z_o     = z;
    assign obs_n_o     = n;
    assign obs_pause_o = pause;
    assign obs_step_o  = step;
    assign obs_armed_o = armed;
    assign obs_tgt_o   = tgt;
    assign obs_wsta_o  = wsta;
    assign obs_capw_o  = capw;

    // host read mux
    wire [6:0] ha = hreg_addr_i;
    wire       hlane = (ha >= 7'h10) && (ha <= 7'h2f);
    wire [1:0] hl = ha[4:3] + 2'd2;      // 0x10 -> lane 0 (ha[4:3] = 2)
    always @* begin
        hreg_rdata_o = 32'h0;
        if (ha == 7'h04) hreg_rdata_o = {20'h0, run, 4'h0, attn};
        else if (hlane) begin
            case (ha[2:0])
                3'd0: hreg_rdata_o = {22'h0, qwait[hl], capw[hl], wsta[hl], lane_res_valid_i[hl],
                                      lane_park_full_i[hl], lane_busy_i[hl], attn[hl], step[hl], pause[hl], run[hl]};
                3'd1: hreg_rdata_o = {23'h0, pc[9*hl +: 9]};
                3'd2: hreg_rdata_o = {19'h0, armed[4*hl +: 4], qf_i[hl], qe_i[hl], lane_ev_pend_i[4*hl +: 4],
                                      lane_m_i[hl], n[hl], z[hl]};
                3'd3: hreg_rdata_o = rf[64*hl +: 32];
                3'd4: hreg_rdata_o = rf[64*hl + 32 +: 32];
                default: hreg_rdata_o = 32'h0;
            endcase
        end
    end

    // ------------------------------------------------------------- update
    wire        hw    = hreg_we_i && ph[1];
    wire [15:0] xwv   = (x_wsrc == 2'd0) ? x_wval : (x_wsrc == 2'd1) ? dm_rdata_i[15:0] : dm_rdata_i[31:16];
    integer i;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ph <= 5'b00001; ir <= 32'h0;
            pc <= 36'h0; rf <= 256'h0;
            z <= 4'h0; n <= 4'h0; pause <= 4'h0; step <= 4'h0; attn <= 4'h0;
            wsta <= 4'h0; capw <= 4'h0; qwait <= 4'h0; armed <= 16'h0; tgt <= 144'h0;
`ifdef MUT_RESET_RUN
            run <= 4'b0010;
`else
            run <= 4'h0;
`endif
            xv <= 1'b0; xl <= 2'd0;
            x_park <= 1'b0; x_pop <= 2'd0; x_pctl <= 28'h0; x_pmem <= 1'b0; x_pdata <= 32'h0;
            x_ack <= 1'b0; x_abort <= 1'b0; x_evclr <= 4'h0;
            x_cfg <= 1'b0; x_creg <= 5'h0; x_chi <= 1'b0; x_cdata <= 16'h0;
            x_cmd <= 2'd0; x_cmdd <= 32'h0;
            x_wen <= 1'b0; x_wreg <= 2'd0; x_wsrc <= 2'd0; x_wval <= 16'h0;
            x_fen <= 1'b0; x_z <= 1'b0; x_n <= 1'b0;
`ifdef MUT_REGION
            mr_v <= 1'b0; mr_off <= 7'h0; mr_hm <= 2'b00; mr_wd <= 32'h0;
`endif
        end else begin
            ph <= {ph[3:0], ph[4]};
            ir <= pm_rdata_i;
`ifdef MUT_REGION
            mr_v <= dv && d_op == WRITE && d_hm != 2'b11; mr_off <= d_off; mr_hm <= d_hm; mr_wd <= d_wd;
`endif
            // D/X register: every cycle
            xv <= dv; xl <= dl;
            x_park <= n_park; x_pop <= n_pop; x_pctl <= n_pctl; x_pmem <= n_pmem; x_pdata <= n_pdata;
            x_ack <= n_ack; x_abort <= n_abort; x_evclr <= n_evclr;
            x_cfg <= n_cfg; x_creg <= n_creg; x_chi <= n_chi; x_cdata <= n_cdata;
            x_cmd <= n_cmd; x_cmdd <= n_cmdd;
            x_wen <= n_wen; x_wreg <= n_wreg; x_wsrc <= n_wsrc; x_wval <= n_wval;
            x_fen <= n_fen; x_z <= n_z; x_n <= n_n;

            // X-stage writes of the X owner
            if (xv && x_wen) rf[64*xl + 16*x_wreg +: 16] <= xwv;
            if (xv && x_fen) begin z[xl] <= x_z; n[xl] <= x_n; end

            // D-stage writes of the D owner
            if (dv) begin
`ifdef MUT_STEP_KEEP
`else
                if (run[dl] && (!pause[dl] || step[dl])) step[dl] <= 1'b0;
`endif
                if (d_pcwe) pc[9*dl +: 9] <= d_pc;
                if (d_rwe)  rf[64*dl + 16*d_rreg +: 16] <= d_rval;
                if (d_qw_we) qwait[dl] <= d_qw;
                if (d_wsta_set) wsta[dl] <= 1'b1;
                if (d_wsta_clr) wsta[dl] <= 1'b0;
                if (d_capw_set) capw[dl] <= 1'b1;
                if (d_capw_clr) capw[dl] <= 1'b0;
                if (d_irq) attn[dl] <= 1'b1;
                if (d_arm_we) begin armed[4*dl + d_ak] <= d_av; tgt[36*dl + 9*d_ak +: 9] <= d_at; end
                if (d_vec) begin wsta[dl] <= 1'b0; capw[dl] <= 1'b0; qwait[dl] <= 1'b0; end
                if (d_halt) begin
                    run[dl] <= 1'b0; attn[dl] <= 1'b1; armed[4*dl +: 4] <= 4'h0;
                    wsta[dl] <= 1'b0; capw[dl] <= 1'b0; qwait[dl] <= 1'b0;
                end
            end

            // host writes
            if (hw) begin
                for (i = 0; i < 4; i = i + 1) begin
                    if (hreg_addr_i == 7'h10 + 8*i) begin
                        run[i] <= hreg_wdata_i[0]; pause[i] <= hreg_wdata_i[1]; step[i] <= hreg_wdata_i[2];
                        if (!hreg_wdata_i[0]) begin
                            armed[4*i +: 4] <= 4'h0; wsta[i] <= 1'b0; capw[i] <= 1'b0; qwait[i] <= 1'b0;
                        end
                    end
`ifdef MUT_HOST_PC_RUN
                    if (hreg_addr_i == 7'h11 + 8*i) pc[9*i +: 9] <= hreg_wdata_i[8:0];
`else
                    if (hreg_addr_i == 7'h11 + 8*i && !run[i]) pc[9*i +: 9] <= hreg_wdata_i[8:0];
`endif
                    if (hreg_addr_i == 7'h04 && hreg_wdata_i[i]) attn[i] <= 1'b0;
                end
            end
        end
    end
endmodule
