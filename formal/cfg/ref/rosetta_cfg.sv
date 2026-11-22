`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/lane.md section 5 (the lane's
//  mode-register file, spec name rosetta_lane_cfg / u_cfg) written by the
//  formal property writer, only to show that formal/cfg/*.sv can PASS and
//  that the properties are non-vacuous (the planted bugs below make tasks
//  FAIL). The real block is src/rosetta_cfg.sv, written independently.
//  Do not integrate, synthesise or copy this file.
//
//  Planted bugs (mutation testing), enabled one at a time with a define:
//    MUT_DEC_SWAP       PERIOD and PERIOD2 decoded at each other's number
//    MUT_DEC_ALIAS      decode looks at cfg_reg_i[3:0] only (26 writes EVC0)
//    MUT_EVC_RANGE      EVC index = (reg - 10) truncated to 2 bits: a write
//                       to register 14 lands in EVC0
//    MUT_HI_SWAP        CRCPOLY halves swapped
//    MUT_HI_IGNORED     a high-half write to PERIOD writes PERIOD
//    MUT_FIELD_POS      launch delay taken from cfg_data_i[7:0] (THR2 slot)
//    MUT_CODE_BITS      CODE[11:10] stored swapped
//    MUT_PARTIAL        CRCCFG[6] (final XOR) not stored
//    MUT_RST_VAL        PERIOD resets to 1
//    MUT_NO_RST         CRCCFG flops have no reset
//    MUT_SYNC_RST       STUFF has a synchronous reset
//    MUT_NO_WE          PULSE written without cfg_we_i
//    MUT_NO_RUN         register writes not gated by run_i
//    MUT_WRITE_LATE     PERIOD2 takes its new value one cycle late
//    MUT_COMB_READ      TICK output shows the write data in the write cycle
//    MUT_PULSE_HI       wr_tick also on a high-half write to TICK
//    MUT_CRC_NOHI       wr_crc missing on a high-half write to CRCPOLY
//    MUT_CRC9_HI        wr_crc also on a high-half write to CRCCFG
//    MUT_PULSE_LATE     wr_stuff one cycle late (registered)
//    MUT_PULSE_REG      wr_code on register 6 instead of 5
//    MUT_EVC_ORDER      wr_evc bits reversed (wr_evc[0] for EVC3)
//    MUT_PULSE_NORUN    wr_evc not gated by run_i
//    MUT_PULSE_ALIAS    pulses decode cfg_reg_i[3:0] only (16 raises wr_tick)
//    MUT_PULSE_MISSING  wr_crc never raised for CRCCFG
//  Amendment H3/H4 (TICK[15:14], CODE[12] stored; TICK[13] never stored):
//    MUT_T14_NOSTORE    TICK[14] (rone) not stored, reads 0
//    MUT_T15_NOSTORE    TICK[15] (rsp2) not stored, reads 0
//    MUT_T13_STORED     TICK[13] stored like the other bits
//    MUT_C12_NOSTORE    CODE[12] (rawedge) not stored, reads 0
//    MUT_T14_DBIT       TICK[14] taken from cfg_data_i[13]
//    MUT_T1415_SWAP     TICK[15] and TICK[14] taken from each other's bit
//    MUT_C12_DBIT       CODE[12] taken from cfg_data_i[13]
//    MUT_C12_TICKREG    CODE[12] loaded by a write to TICK (register 0)
//    MUT_T15_NORST      TICK[15:14] flops have no reset
//    MUT_C12_NORST      CODE[12] flop has no reset
// ===========================================================================
module rosetta_cfg (
    input  wire         clk, rst_n,
    input  wire         run_i,
    input  wire         cfg_we_i,
    input  wire  [4:0]  cfg_reg_i,
    input  wire         cfg_hi_i,
    input  wire  [15:0] cfg_data_i,
    output logic [15:0] tick_o,
    output logic [15:0] period_o,
    output logic [15:0] period2_o,
    output logic [7:0]  thr0_o,
    output logic [7:0]  dly_o,
    output logic [12:0] code_o,
    output logic [15:0] pulse_o,
    output logic [5:0]  stuff_o,
    output logic [31:0] crcpoly_o,
    output logic [6:0]  crccfg_o,
    output logic [63:0] evc_o,
    output logic        wr_tick_o,
    output logic        wr_code_o,
    output logic        wr_stuff_o,
    output logic        wr_crc_o,
    output logic [3:0]  wr_evc_o
);
    // ------------------------------------------------------------ decode
`ifdef MUT_NO_RUN
    wire we = cfg_we_i;
`else
    wire we = run_i & cfg_we_i;
`endif
    wire lo = we & ~cfg_hi_i;
    wire hi = we &  cfg_hi_i;

`ifdef MUT_DEC_ALIAS
    wire [4:0] rn = {1'b0, cfg_reg_i[3:0]};
`else
    wire [4:0] rn = cfg_reg_i;
`endif

    // wlo[r]: low-half write to register r (r = 0..13)
    logic [13:0] wlo;
    always_comb begin
        for (int r = 0; r < 14; r++) wlo[r] = lo && (rn == 5'(r));
`ifdef MUT_DEC_SWAP
        wlo[1] = lo && (rn == 5'd2);
        wlo[2] = lo && (rn == 5'd1);
`endif
`ifdef MUT_EVC_RANGE
        for (int k = 0; k < 4; k++)
            wlo[10 + k] = lo && (rn >= 5'd10) && (rn <= 5'd17) &&
                          (2'(rn - 5'd10) == 2'(k));
`endif
    end
    wire whi8 = hi && (rn == 5'd8);
`ifdef MUT_HI_IGNORED
    wire wper = (we && (rn == 5'd1));
`else
    wire wper = wlo[1];
`endif

    // ------------------------------------------------------------ registers
    logic [12:0] tick_q;
    logic [11:0] code_q;
    logic [15:0] p2_pend_d;
    logic        p2_pend_v;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_q    <= '0;
`ifdef MUT_RST_VAL
            period_o  <= 16'd1;
`else
            period_o  <= 16'd0;
`endif
            period2_o <= '0;
            thr0_o    <= '0;
            dly_o     <= '0;
            code_q    <= '0;
            pulse_o   <= '0;
            crcpoly_o <= '0;
            evc_o     <= '0;
            p2_pend_d <= '0;
            p2_pend_v <= 1'b0;
        end else begin
            if (wlo[0]) tick_q   <= cfg_data_i[12:0];
            if (wper)   period_o <= cfg_data_i;
`ifdef MUT_WRITE_LATE
            p2_pend_v <= wlo[2];
            p2_pend_d <= cfg_data_i;
            if (p2_pend_v) period2_o <= p2_pend_d;
`else
            if (wlo[2]) period2_o <= cfg_data_i;
`endif
            if (wlo[3]) thr0_o <= cfg_data_i[7:0];
`ifdef MUT_FIELD_POS
            if (wlo[4]) dly_o  <= cfg_data_i[7:0];
`else
            if (wlo[4]) dly_o  <= cfg_data_i[15:8];
`endif
`ifdef MUT_CODE_BITS
            if (wlo[5]) code_q <= {cfg_data_i[10], cfg_data_i[11], cfg_data_i[9:0]};
`else
            if (wlo[5]) code_q <= cfg_data_i[11:0];
`endif
`ifdef MUT_NO_WE
            if (run_i && !cfg_hi_i && rn == 5'd6) pulse_o <= cfg_data_i;
`else
            if (wlo[6]) pulse_o <= cfg_data_i;
`endif
`ifdef MUT_HI_SWAP
            if (wlo[8]) crcpoly_o[31:16] <= cfg_data_i;
            if (whi8)   crcpoly_o[15:0]  <= cfg_data_i;
`else
            if (wlo[8]) crcpoly_o[15:0]  <= cfg_data_i;
            if (whi8)   crcpoly_o[31:16] <= cfg_data_i;
`endif
            for (int k = 0; k < 4; k++)
                if (wlo[10 + k]) evc_o[16*k +: 16] <= cfg_data_i;
        end
    end

    // ---- amendment H3/H4: TICK[15:14] (rone, rsp2), CODE[12] (rawedge);
    //      TICK[13] (H1, on hold) is not stored and reads 0
    logic [1:0] t1514_q;
    logic       c12_q, t13_q;
`ifdef MUT_T14_DBIT
    wire  [1:0] t1514_d = {cfg_data_i[15], cfg_data_i[13]};
`elsif MUT_T1415_SWAP
    wire  [1:0] t1514_d = {cfg_data_i[14], cfg_data_i[15]};
`else
    wire  [1:0] t1514_d = cfg_data_i[15:14];
`endif
`ifdef MUT_C12_DBIT
    wire        c12_d   = cfg_data_i[13];
`else
    wire        c12_d   = cfg_data_i[12];
`endif
`ifdef MUT_C12_TICKREG
    wire        c12_we  = wlo[0];
`else
    wire        c12_we  = wlo[5];
`endif
`ifdef MUT_T15_NORST
    always_ff @(posedge clk) begin
        if (wlo[0]) t1514_q <= t1514_d;
    end
`else
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)      t1514_q <= 2'b00;
        else if (wlo[0]) t1514_q <= t1514_d;
    end
`endif
`ifdef MUT_C12_NORST
    always_ff @(posedge clk) begin
        if (c12_we) c12_q <= c12_d;
    end
`else
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)      c12_q <= 1'b0;
        else if (c12_we) c12_q <= c12_d;
    end
`endif
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)      t13_q <= 1'b0;
        else if (wlo[0]) t13_q <= cfg_data_i[13];
    end
`ifdef MUT_T13_STORED
    wire        t13_o = t13_q;
`else
    wire        t13_o = 1'b0;
`endif
`ifdef MUT_T14_NOSTORE
    wire        t14_o = 1'b0;
`else
    wire        t14_o = t1514_q[0];
`endif
`ifdef MUT_T15_NOSTORE
    wire        t15_o = 1'b0;
`else
    wire        t15_o = t1514_q[1];
`endif
`ifdef MUT_C12_NOSTORE
    assign code_o = {1'b0, code_q};
`else
    assign code_o = {c12_q, code_q};
`endif

`ifdef MUT_COMB_READ
    assign tick_o = {t15_o, t14_o, t13_o, wlo[0] ? cfg_data_i[12:0] : tick_q};
`else
    assign tick_o = {t15_o, t14_o, t13_o, tick_q};
`endif

`ifdef MUT_SYNC_RST
    always_ff @(posedge clk) begin
        if (!rst_n)     stuff_o <= '0;
        else if (wlo[7]) stuff_o <= cfg_data_i[5:0];
    end
`else
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)      stuff_o <= '0;
        else if (wlo[7]) stuff_o <= cfg_data_i[5:0];
    end
`endif

`ifdef MUT_NO_RST
    always_ff @(posedge clk) begin
        if (wlo[9]) crccfg_o <= cfg_data_i[6:0];
    end
`else
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)       crccfg_o <= '0;
`ifdef MUT_PARTIAL
        else if (wlo[9])  crccfg_o <= {1'b0, cfg_data_i[5:0]};
`else
        else if (wlo[9])  crccfg_o <= cfg_data_i[6:0];
`endif
    end
`endif

    // ------------------------------------------------------------ pulses
    // Combinational in the write cycle (lane.md 5.2). Decoded from the raw
    // inputs (not from wlo) so register-path mutants do not move them.
    wire       p_we = run_i & cfg_we_i;
    wire       p_lo = p_we & ~cfg_hi_i;
`ifdef MUT_PULSE_ALIAS
    wire [4:0] pr   = {1'b0, cfg_reg_i[3:0]};
`else
    wire [4:0] pr   = cfg_reg_i;
`endif

`ifdef MUT_PULSE_HI
    assign wr_tick_o = p_we && (pr == 5'd0);
`else
    assign wr_tick_o = p_lo && (pr == 5'd0);
`endif

`ifdef MUT_PULSE_REG
    assign wr_code_o = p_lo && (pr == 5'd6);
`else
    assign wr_code_o = p_lo && (pr == 5'd5);
`endif

`ifdef MUT_PULSE_LATE
    logic wr_stuff_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) wr_stuff_q <= 1'b0;
        else        wr_stuff_q <= p_lo && (pr == 5'd7);
    end
    assign wr_stuff_o = wr_stuff_q;
`else
    assign wr_stuff_o = p_lo && (pr == 5'd7);
`endif

    wire crc8 = p_we && (pr == 5'd8);
    wire crc9 = p_lo && (pr == 5'd9);
`ifdef MUT_CRC_NOHI
    assign wr_crc_o = (p_lo && (pr == 5'd8)) || crc9;
`elsif MUT_CRC9_HI
    assign wr_crc_o = crc8 || (p_we && (pr == 5'd9));
`elsif MUT_PULSE_MISSING
    assign wr_crc_o = crc8;
`else
    assign wr_crc_o = crc8 || crc9;
`endif

    logic [3:0] evc_p;
    always_comb begin
        for (int k = 0; k < 4; k++) begin
`ifdef MUT_PULSE_NORUN
            evc_p[k] = cfg_we_i && !cfg_hi_i && (pr == 5'(10 + k));
`else
            evc_p[k] = p_lo && (pr == 5'(10 + k));
`endif
        end
    end
`ifdef MUT_EVC_ORDER
    assign wr_evc_o = {evc_p[0], evc_p[1], evc_p[2], evc_p[3]};
`else
    assign wr_evc_o = evc_p;
`endif
endmodule
