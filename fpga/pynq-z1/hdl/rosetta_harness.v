// Rosetta FPGA test harness (PYNQ-Z1 / XC7Z020).
//
// Wraps the Tiny Tapeout top `tt_um_jesselevine727_rosetta` (via the generated
// shim module `rosetta_chip`) so the Zynq PS can play the RP2350's role:
//   * 60 MHz chip clock (MMCM) through a BUFGCE: free-run or single-step
//   * rst_n / ena / all ui_in, uio_in driven from AXI GPIO registers
//   * host SPI (ui[0]=SCK, ui[1]=CS_n, ui[2]=MOSI, uo[0]=MISO) from AXI Quad SPI
//   * per-input source select: processor / board header / loopback of any
//     chip output or uio pin (loopback is registered: +1 cycle, see README)
//   * all 24 chip pins on board headers (ui on Arduino IO0-7, uo on PMOD JA,
//     uio on PMOD JB through IOBUFs that honour uio_oe); headers are hi-Z
//     until CTRL.hdr_en is set
//   * 60 MHz logic capture of {uio_oe, uio, uo, ui} into a BRAM (2**CAP_AW
//     samples) with mask/value/edge/change trigger and pre-trigger
//   * chip clock frequency meters against FCLK0 and the 125 MHz board clock
//   * lockstep mode (regression against the Verilator harness): the chip
//     clock runs only for RUN_N-cycle runs and for host frames played by a
//     cycle-exact host-SPI engine (rh_hostspi), so the chip sees the same
//     input on the same edge as in toolchain/tests/rtl/rosetta_tb.cpp; a
//     change trace with 48-bit cycle stamps (rh_trace) records every pin
//     change; zero-delay uo->ui loopback; uio pins joined into open-drain
//     nets (wired-AND, pull-up); per-uio-pin input skew; a one-shot
//     inversion fault on the loopback wires; and an asynchronous UART far
//     end clocked by the 125 MHz oscillator (rh_async_uart)
//
// All register fields are documented in README.md.
`timescale 1ns / 1ps
`default_nettype none

module rosetta_harness #(
    parameter integer CAP_AW = 14,  // capture depth = 2**CAP_AW samples (14 -> 16384)
    parameter integer TRC_AW = 14   // change-trace depth = 2**TRC_AW entries
) (
    // ---- clocks / resets -------------------------------------------------
    input  wire        clk60,       // 60 MHz, MMCM CLKOUT0 via BUFG (harness + AXI clock)
    input  wire        clk60_raw,   // 60 MHz, MMCM CLKOUT1, unbuffered -> BUFGCE -> chip clk
    input  wire        aresetn,     // active-low reset, synchronous to clk60
    input  wire        ref_fclk,    // PS FCLK_CLK0 (100 MHz), frequency reference
    input  wire        sysclk,      // 125 MHz board clock (H16), independent reference
    input  wire        mmcm_locked,

    // ---- AXI GPIO channels -------------------------------------------------
    input  wire [31:0] ctrl,        // gpio_ctrl  ch1 (W)  CTRL
    output wire [31:0] status,      // gpio_ctrl  ch2 (R)  STATUS
    input  wire [31:0] drive,       // gpio_pins  ch1 (W)  DRIVE
    output wire [31:0] chip_out,    // gpio_pins  ch2 (R)  CHIP
    input  wire [31:0] src_sel,     // gpio_route ch1 (W)  SRC
    input  wire [31:0] route2,      // gpio_route ch2 (W)  SRC2
    input  wire [31:0] loop_lo,     // gpio_loop  ch1 (W)  LOOP_LO
    input  wire [31:0] loop_hi,     // gpio_loop  ch2 (W)  LOOP_HI
    input  wire [31:0] trig_mask,   // gpio_trig  ch1 (W)  TRIG_MASK
    input  wire [31:0] trig_value,  // gpio_trig  ch2 (W)  TRIG_VALUE
    input  wire [31:0] cap_cfg,     // gpio_cap   ch1 (W)  CAP_CFG
    output wire [31:0] cap_status,  // gpio_cap   ch2 (R)  CAP_STATUS
    output wire [31:0] pads,        // gpio_mon   ch1 (R)  PADS
    output wire [31:0] chip_cycles, // gpio_mon   ch2 (R)  CYCLES
    output wire [31:0] freq_sys,    // gpio_freq  ch1 (R)  FREQ_SYS
    output wire [31:0] freq_fclk,   // gpio_freq  ch2 (R)  FREQ_FCLK
    input  wire [31:0] ls_ctrl,     // gpio_ls    ch1 (W)  LS_CTRL
    output wire [31:0] ls_status,   // gpio_ls    ch2 (R)  LS_STATUS
    input  wire [31:0] run_n,       // gpio_run   ch1 (W)  RUN_N
    input  wire [31:0] host_cmd,    // gpio_run   ch2 (W)  HOST_CMD
    input  wire [31:0] host_data,   // gpio_host  ch1 (W)  HOST_DATA
    output wire [31:0] host_rdata,  // gpio_host  ch2 (R)  HOST_RDATA
    input  wire [31:0] nets,        // gpio_net   ch1 (W)  NETS
    input  wire [31:0] trace_rp,    // gpio_net   ch2 (W)  TRACE_RP
    input  wire [31:0] skew_lo,     // gpio_skew  ch1 (W)  SKEW_LO
    input  wire [31:0] skew_hi,     // gpio_skew  ch2 (W)  SKEW_HI
    input  wire [31:0] fault_off,   // gpio_fault ch1 (W)  FAULT_OFF
    input  wire [31:0] fault_len,   // gpio_fault ch2 (W)  FAULT_LEN
    output wire [31:0] cyc_lo,      // gpio_cyc   ch1 (R)  CYC_LO
    output wire [31:0] cyc_hi,      // gpio_cyc   ch2 (R)  CYC_HI
    input  wire [31:0] async_inc,   // gpio_async ch1 (W)  ASYNC_INC
    input  wire [31:0] async_cmd,   // gpio_async ch2 (W)  ASYNC_CMD
    input  wire [31:0] wave_wp,     // gpio_wave  ch1 (W)  WAVE_WP
    output wire [31:0] wave_status, // gpio_wave  ch2 (R)  WAVE_STATUS
    input  wire [31:0] chmap_lo,    // gpio_chmap ch1 (W)  CHMAP_LO
    input  wire [31:0] chmap_hi,    // gpio_chmap ch2 (W)  CHMAP_HI
    input  wire [31:0] pulls,       // gpio_pull  ch1 (W)  PULL
    output wire [31:0] contention,  // gpio_pull  ch2 (R)  CONTENTION
    input  wire [31:0] can_cfg,     // gpio_can   ch1 (W)  CAN_CFG
    input  wire [31:0] can_masks,   // gpio_can   ch2 (W)  CAN_MASKS
    input  wire [31:0] line_ctrl,   // gpio_line  ch1 (W)  LINE_CTRL
    output wire [31:0] line_status, // gpio_line  ch2 (R)  LINE_STATUS

    // ---- AXI Quad SPI (standard mode, master) ------------------------------
    input  wire        spi_sck,
    input  wire        spi_mosi,
    output wire        spi_miso,
    input  wire        spi_ss_n,

    // ---- capture BRAM, port B of a true dual-port blk_mem_gen ---------------
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 CAP_BRAM CLK" *)
    (* X_INTERFACE_PARAMETER = "MASTER_TYPE BRAM_CTRL,MEM_ECC NONE,MEM_WIDTH 32,MEM_SIZE 65536,READ_WRITE_MODE READ_WRITE,READ_LATENCY 1" *)
    (* X_INTERFACE_MODE = "Master" *)
    output wire        cap_bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 CAP_BRAM RST" *)
    output wire        cap_bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 CAP_BRAM EN" *)
    output wire        cap_bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 CAP_BRAM WE" *)
    output wire [3:0]  cap_bram_we,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 CAP_BRAM ADDR" *)
    output wire [31:0] cap_bram_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 CAP_BRAM DIN" *)
    output wire [31:0] cap_bram_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 CAP_BRAM DOUT" *)
    input  wire [31:0] cap_bram_dout,

    // ---- trace word 0 BRAM, port B of a true dual-port blk_mem_gen -------
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC0_BRAM CLK" *)
    (* X_INTERFACE_PARAMETER = "MASTER_TYPE BRAM_CTRL,MEM_ECC NONE,MEM_WIDTH 32,MEM_SIZE 65536,READ_WRITE_MODE READ_WRITE,READ_LATENCY 1" *)
    (* X_INTERFACE_MODE = "Master" *)
    output wire        trc0_bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC0_BRAM RST" *)
    output wire        trc0_bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC0_BRAM EN" *)
    output wire        trc0_bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC0_BRAM WE" *)
    output wire [3:0]  trc0_bram_we,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC0_BRAM ADDR" *)
    output wire [31:0] trc0_bram_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC0_BRAM DIN" *)
    output wire [31:0] trc0_bram_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC0_BRAM DOUT" *)
    input  wire [31:0] trc0_bram_dout,

    // ---- trace word 1 BRAM, port B of a true dual-port blk_mem_gen -------
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC1_BRAM CLK" *)
    (* X_INTERFACE_PARAMETER = "MASTER_TYPE BRAM_CTRL,MEM_ECC NONE,MEM_WIDTH 32,MEM_SIZE 65536,READ_WRITE_MODE READ_WRITE,READ_LATENCY 1" *)
    (* X_INTERFACE_MODE = "Master" *)
    output wire        trc1_bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC1_BRAM RST" *)
    output wire        trc1_bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC1_BRAM EN" *)
    output wire        trc1_bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC1_BRAM WE" *)
    output wire [3:0]  trc1_bram_we,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC1_BRAM ADDR" *)
    output wire [31:0] trc1_bram_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC1_BRAM DIN" *)
    output wire [31:0] trc1_bram_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC1_BRAM DOUT" *)
    input  wire [31:0] trc1_bram_dout,

    // ---- trace word 2 BRAM, port B of a true dual-port blk_mem_gen -------
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC2_BRAM CLK" *)
    (* X_INTERFACE_PARAMETER = "MASTER_TYPE BRAM_CTRL,MEM_ECC NONE,MEM_WIDTH 32,MEM_SIZE 65536,READ_WRITE_MODE READ_WRITE,READ_LATENCY 1" *)
    (* X_INTERFACE_MODE = "Master" *)
    output wire        trc2_bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC2_BRAM RST" *)
    output wire        trc2_bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC2_BRAM EN" *)
    output wire        trc2_bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC2_BRAM WE" *)
    output wire [3:0]  trc2_bram_we,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC2_BRAM ADDR" *)
    output wire [31:0] trc2_bram_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC2_BRAM DIN" *)
    output wire [31:0] trc2_bram_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 TRC2_BRAM DOUT" *)
    input  wire [31:0] trc2_bram_dout,

    // ---- wave BRAM (port B of a true dual-port blk_mem_gen; the harness reads) --
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAVE_BRAM CLK" *)
    (* X_INTERFACE_PARAMETER = "MASTER_TYPE BRAM_CTRL,MEM_ECC NONE,MEM_WIDTH 32,MEM_SIZE 65536,READ_WRITE_MODE READ_WRITE,READ_LATENCY 1" *)
    (* X_INTERFACE_MODE = "Master" *)
    output wire        wave_bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAVE_BRAM RST" *)
    output wire        wave_bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAVE_BRAM EN" *)
    output wire        wave_bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAVE_BRAM WE" *)
    output wire [3:0]  wave_bram_we,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAVE_BRAM ADDR" *)
    output wire [31:0] wave_bram_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAVE_BRAM DIN" *)
    output wire [31:0] wave_bram_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAVE_BRAM DOUT" *)
    input  wire [31:0] wave_bram_dout,

    // ---- line BRAM (port B of a true dual-port blk_mem_gen; the harness reads) --
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LINE_BRAM CLK" *)
    (* X_INTERFACE_PARAMETER = "MASTER_TYPE BRAM_CTRL,MEM_ECC NONE,MEM_WIDTH 32,MEM_SIZE 65536,READ_WRITE_MODE READ_WRITE,READ_LATENCY 1" *)
    (* X_INTERFACE_MODE = "Master" *)
    output wire        line_bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LINE_BRAM RST" *)
    output wire        line_bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LINE_BRAM EN" *)
    output wire        line_bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LINE_BRAM WE" *)
    output wire [3:0]  line_bram_we,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LINE_BRAM ADDR" *)
    output wire [31:0] line_bram_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LINE_BRAM DIN" *)
    output wire [31:0] line_bram_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LINE_BRAM DOUT" *)
    input  wire [31:0] line_bram_dout,

    // ---- board pins ---------------------------------------------------------
    inout  wire [7:0]  ck_io,       // Arduino IO0..IO7  <-> ui[0..7]
    inout  wire [7:0]  ja,          // PMOD JA            <-  uo[0..7]
    inout  wire [7:0]  jb,          // PMOD JB           <-> uio[0..7]
    output wire [3:0]  led
);

    // =========================================================================
    // CTRL fields
    // =========================================================================
    wire        c_rst_n     = ctrl[0];
    wire        c_ena       = ctrl[1];
    wire        c_clk_run   = ctrl[2];
    wire        c_step_tgl  = ctrl[3];
    wire        c_hdr_en    = ctrl[4];
    wire        c_cap_arm   = ctrl[5];
    wire        c_cap_force = ctrl[6];
    wire [1:0]  c_trig_mode = ctrl[9:8];
    wire [15:0] c_cap_div   = ctrl[31:16];

    // =========================================================================
    // LS_CTRL fields (lockstep regression mode, README "Lockstep mode")
    // =========================================================================
    wire        l_trace_en   = ls_ctrl[3];
    wire        l_loop_dir   = ls_ctrl[5];
    wire        l_uio_iso    = ls_ctrl[6];
    wire        l_bp_en      = ls_ctrl[7];
    wire [3:0]  l_fault_src  = ls_ctrl[15:12];
    wire        l_host_eng   = ls_ctrl[16];

    // toggle bits -> one-cycle pulses
    reg  [8:0]  ls_tgl_q = 9'd0;
    always @(posedge clk60) ls_tgl_q <= ls_ctrl[8:0];
    wire        p_run_go     = ls_ctrl[0] ^ ls_tgl_q[0];
    wire        p_host_go    = ls_ctrl[1] ^ ls_tgl_q[1];
    wire        p_trace_clr  = ls_ctrl[2] ^ ls_tgl_q[2];
    wire        p_cyc_clr    = ls_ctrl[4] ^ ls_tgl_q[4];
    wire        p_fault_arm  = ls_ctrl[8] ^ ls_tgl_q[8];

    // =========================================================================
    // Chip clock: BUFGCE on a second, phase-aligned 60 MHz MMCM output.
    // chip_ce (registered on clk60) decides whether the chip gets the NEXT
    // rising edge, so "chip_ce==1 at a clk60 edge" == "the chip is clocked at
    // this edge"; the cycle counters, the capture, the trace and the host
    // engine use exactly that. Lockstep requests (RUN_N runs and host-engine
    // frames) pause while the change trace is nearly full (LS_CTRL.bp_en),
    // so no change is lost however slowly the processor drains it.
    // =========================================================================
    reg step_tgl_q = 1'b0;
    reg chip_ce    = 1'b0;
    reg [31:0] run_rem = 32'd0;
    wire        eng_busy_n;
    wire        trc_almost_full;
    wire [31:0] run_rem_n = p_run_go ? run_n : (run_rem - {31'd0, chip_ce && run_rem != 32'd0});
    wire        wave_ready_n;
    wire        ls_want   = ((run_rem_n != 32'd0) | eng_busy_n) & wave_ready_n;
    wire        bp_stall  = l_bp_en & trc_almost_full;
    always @(posedge clk60) begin
        if (!aresetn) begin
            step_tgl_q <= c_step_tgl;
            chip_ce    <= 1'b0;
            run_rem    <= 32'd0;
        end else begin
            step_tgl_q <= c_step_tgl;
            run_rem    <= run_rem_n;
            chip_ce    <= c_clk_run | (c_step_tgl ^ step_tgl_q) | (ls_want & ~bp_stall);
        end
    end

    wire chip_clk;
    BUFGCE u_chip_bufgce (.I(clk60_raw), .CE(chip_ce), .O(chip_clk));

    // Edge counter clocked by the delivered chip clock itself: it measures the
    // real BUFGCE output (and keeps it alive even if the chip has no flops).
    // Read in the clk60 domain: same MMCM VCO and divider, so a timed path.
    reg [31:0] cycles = 32'd0;
    always @(posedge chip_clk) cycles <= cycles + 32'd1;
    assign chip_cycles = cycles;

    // Lockstep cycle counter: chip edges delivered since CYC clear (48 bits),
    // the Verilator harness's `cyc`.
    reg [47:0] lcyc = 48'd0;
    always @(posedge clk60) begin
        if (p_cyc_clr)    lcyc <= 48'd0;
        else if (chip_ce) lcyc <= lcyc + 48'd1;
    end
    assign cyc_lo = lcyc[31:0];
    assign cyc_hi = {16'd0, lcyc[47:32]};

    // =========================================================================
    // The chip
    // =========================================================================
    wire [7:0] ui_eff;     // what the chip sees on ui_in
    wire [7:0] uio_eff;    // what the chip sees on uio_in (= uio pin value)
    wire [7:0] uo_out, uio_out, uio_oe;

    rosetta_chip u_chip (
        .ui_in  (ui_eff),
        .uo_out (uo_out),
        .uio_in (uio_eff),
        .uio_out(uio_out),
        .uio_oe (uio_oe),
        .ena    (c_ena),
        .clk    (chip_clk),
        .rst_n  (c_rst_n)
    );

    // =========================================================================
    // Lockstep host engine (ui0..2 when LS_CTRL.host_eng) and the
    // asynchronous UART far end (any one ui pin when ASYNC_CMD.route)
    // =========================================================================
    wire        eng_sclk, eng_csn, eng_mosi, eng_busy;
    wire [31:0] eng_rdata;
    rh_hostspi u_hostspi (
        .clk(clk60), .rstn(aresetn), .ce(chip_ce), .go(p_host_go),
        .rd(host_cmd[7]), .addr(host_cmd[6:0]), .data(host_data), .miso(uo_out[0]),
        .sclk(eng_sclk), .csn(eng_csn), .mosi(eng_mosi),
        .busy(eng_busy), .busy_n(eng_busy_n), .rdata(eng_rdata));
    wire [2:0]  eng_val = {eng_mosi, eng_csn, eng_sclk};
    assign host_rdata = eng_rdata;

    reg         aw_tgl_q = 1'b0;
    always @(posedge clk60) aw_tgl_q <= async_cmd[14];
    wire        async_tx, async_busy;
    rh_async_uart u_auart (
        .clk60(clk60), .rstn(aresetn), .we(async_cmd[14] ^ aw_tgl_q),
        .waddr(async_cmd[13:8]), .wdata(async_cmd[7:0]), .go_tgl(async_cmd[15]),
        .sysclk(sysclk), .inc(async_inc), .count_m1(async_cmd[21:16]),
        .parity(async_cmd[23:22]), .stop2(async_cmd[24]), .gap(async_cmd[31:29]),
        .tx(async_tx), .busy60(async_busy));
    wire [7:0]  async_mask = async_cmd[28] ? (8'd1 << async_cmd[27:25]) : 8'd0;

    // =========================================================================
    // One-shot fault (the Verilator harness's F command): armed by a toggle;
    // at the next falling edge of uo[fault_src] (seen at a chip edge) every
    // loopback wire into ui is inverted for chip edges
    // [edge + FAULT_OFF, edge + FAULT_OFF + FAULT_LEN).
    // =========================================================================
    reg         f_armed = 1'b0, f_seen = 1'b0, f_prev = 1'b1;
    reg  [47:0] f_lo = 48'd0, f_hi = 48'd0;
    wire        f_src   = uo_out[l_fault_src[2:0]];
    wire        f_fall  = f_armed & f_prev & ~f_src;
    wire        fault_inv = (f_seen && lcyc >= f_lo && lcyc < f_hi)
                          | (f_fall && fault_off == 32'd0 && fault_len != 32'd0);
    always @(posedge clk60) begin
        if (!aresetn) begin
            f_armed <= 1'b0;
            f_seen  <= 1'b0;
            f_prev  <= 1'b1;
        end else if (p_fault_arm) begin
            f_armed <= 1'b1;
            f_seen  <= 1'b0;
            f_prev  <= f_src;
        end else if (chip_ce && f_armed) begin
            f_prev <= f_src;
            if (f_fall) begin
                f_armed <= 1'b0;
                f_seen  <= 1'b1;
                f_lo    <= lcyc + {16'd0, fault_off};
                f_hi    <= lcyc + {16'd0, fault_off} + {16'd0, fault_len};
            end
        end
    end

    // =========================================================================
    // Input routing. Input index i: 0..7 = ui[i], 8..15 = uio[i-8].
    // Priority per input: async UART (ui only) > loopback > header >
    // host engine / host SPI (ui0..2) > processor.
    // Loopback source index k: 0..7 = uo[k], 8..15 = uio pin k (uio_eff).
    // A loopback goes through one clk60 register, except a uo source with
    // LS_CTRL.loop_dir = 1, which is a plain wire (as in the Verilator
    // harness; safe because every chip output comes from a flop).
    // =========================================================================
    wire [15:0] src_ext  = src_sel[15:0];
    wire [15:0] src_loop = src_sel[31:16];
    wire [2:0]  src_spi  = route2[2:0];
    wire [63:0] loop_sel = {loop_hi, loop_lo};

    // one clk60 register stage: breaks combinational loops through the chip
    reg  [15:0] loop_src_q = 16'd0;
    always @(posedge clk60) loop_src_q <= {uio_eff, uo_out};

    wire [7:0]  ck_in, ja_in, jb_in;            // pad readbacks
    wire [15:0] pad_val  = {jb_in, ck_in};
    wire [15:0] gpio_val = drive[15:0];
    wire [2:0]  spi_val  = {spi_mosi, spi_ss_n, spi_sck};

    wire [15:0] in_nonext;                      // source ignoring the header option
    wire [15:0] in_sel;                         // final selected source

    genvar g;
    generate
        for (g = 0; g < 16; g = g + 1) begin : g_route
            wire [3:0] ls = loop_sel[4*g +: 4];
            wire lb_r = loop_src_q[ls];
            wire lb_w = (l_loop_dir && !ls[3]) ? uo_out[ls[2:0]] : lb_r;
            wire lb;
            wire base;
            wire asy;
            if (g < 8) begin : g_ui
                assign lb  = lb_w ^ fault_inv;
                assign asy = async_mask[g];
            end else begin : g_uio
                assign lb  = lb_w;
                assign asy = 1'b0;
            end
            if (g < 3) begin : g_host
                assign base = l_host_eng ? eng_val[g] : (src_spi[g] ? spi_val[g] : gpio_val[g]);
            end else begin : g_plain
                assign base = gpio_val[g];
            end
            assign in_nonext[g] = asy ? async_tx : (src_loop[g] ? lb : base);
            assign in_sel[g]    = asy ? async_tx : (src_loop[g] ? lb : (src_ext[g] ? pad_val[g] : base));
        end
    endgenerate

    wire [7:0]  ui_fin;                 // after waveform channels, pulls, async lines, CAN
    assign ui_eff  = ui_fin;

    // uio pins. A pin in an open-drain net (NETS: four 8-bit pin masks) reads
    // the net: low when any pin of the net has uio_oe = 1 and uio_out = 0,
    // else high (pull-up). Any other pin reads back its own value while the
    // chip drives it, as on silicon (LS_CTRL.uio_iso = 1: it reads its
    // selected source regardless, as an unconnected pin in the Verilator
    // harness). Then each pin can read its value SKEW cycles late (chip
    // edges, 0-63), the Verilator harness's S command.
    wire [7:0]  pull  = uio_oe & ~uio_out;
    wire [7:0]  n0 = nets[7:0], n1 = nets[15:8], n2 = nets[23:16], n3 = nets[31:24];
    wire [7:0]  in_net  = n0 | n1 | n2 | n3;
    wire [7:0]  net_val = ~((n0 & {8{|(pull & n0)}}) | (n1 & {8{|(pull & n1)}})
                          | (n2 & {8{|(pull & n2)}}) | (n3 & {8{|(pull & n3)}}));
    wire [7:0]  uio_plain = l_uio_iso ? in_sel[15:8]
                          : ((uio_oe & uio_out) | (~uio_oe & in_sel[15:8]));
    wire [7:0]  uio_res = (in_net & net_val) | (~in_net & uio_plain);
    wire [7:0]  uio_fin;                // after waveform channels, pulls, async lines, CAN
    // =========================================================================
    // Waveform player (lockstep; the Verilator harness's M U V P K commands)
    // and the asynchronous lines (board only). Per pin p (0-7 ui, 8-15 uio):
    // the highest waveform channel mapped to p (CHMAP), and the async line
    // routed to p (LINE_CTRL), which wins over the channels. A ui pin takes the
    // driving source's value, else its pull (PULL) if set, else its normal
    // source. A uio pin with a channel, a line or a pull reads the driving
    // source, else the chip's uio_out while uio_oe = 1, else the pull (or 1).
    // A chip edge where a channel and the chip both drive a uio pin adds one
    // per pin to CONTENTION; for the async lines, LINE_STATUS[15:4].
    // =========================================================================
    wire [7:0]  wv_d, wv_v;
    wire        wave_empty;
    wire [15:0] wave_rp;
    wire [13:0] wave_raddr;
    rh_wave #(.AW(14)) u_wave (
        .clk(clk60), .rstn(aresetn), .ce(chip_ce), .wp(wave_wp[15:0]), .rp(wave_rp),
        .cur_d(wv_d), .cur_v(wv_v), .ready_n(wave_ready_n), .empty(wave_empty),
        .bram_en(wave_bram_en), .bram_addr(wave_raddr), .bram_dout(wave_bram_dout));
    assign wave_bram_clk  = clk60;
    assign wave_bram_rst  = 1'b0;
    assign wave_bram_we   = 4'd0;
    assign wave_bram_din  = 32'd0;
    assign wave_bram_addr = {16'd0, wave_raddr, 2'b00};
    assign wave_status    = {wave_rp, 15'd0, wave_empty};

    wire [1:0]  ln_val, ln_drv;
    wire        ln_busy, ln_ready;
    wire [13:0] ln_raddr;
    rh_line #(.AW(14)) u_line (
        .clk60(clk60), .sysclk(sysclk), .go_tgl(line_ctrl[0]), .n_events(line_ctrl[31:16]),
        .busy60(ln_busy), .ready60(ln_ready), .val(ln_val), .drv(ln_drv),
        .bram_clk(line_bram_clk), .bram_en(line_bram_en), .bram_addr(ln_raddr),
        .bram_dout(line_bram_dout));
    assign line_bram_rst  = 1'b0;
    assign line_bram_we   = 4'd0;
    assign line_bram_din  = 32'd0;
    assign line_bram_addr = {16'd0, ln_raddr, 2'b00};
    // LINE_CTRL: [0] go toggle, [5:1] line 0 pin (0-15 pin, 16 CAN external
    // node, other: off), [10:6] line 1 pin, [31:16] number of events
    wire [4:0]  ln_pin0 = line_ctrl[5:1], ln_pin1 = line_ctrl[10:6];

    // CHMAP_LO / CHMAP_HI: channel k in byte k (0-3) / k-4 (4-7): [4] mapped, [3:0] pin
    reg  [15:0] p_wd, p_wv, p_has, l_wd, l_wv, l_has;
    integer pp, kk;
    always @(*) begin
        for (pp = 0; pp < 16; pp = pp + 1) begin
            p_has[pp] = 1'b0; p_wd[pp] = 1'b0; p_wv[pp] = 1'b0;
            for (kk = 0; kk < 8; kk = kk + 1)
                if ((kk < 4 ? chmap_lo[8*kk + 4] : chmap_hi[8*(kk-4) + 4]) &&
                    (kk < 4 ? chmap_lo[8*kk +: 4] : chmap_hi[8*(kk-4) +: 4]) == pp[3:0]) begin
                    p_has[pp] = 1'b1;                        // the highest channel wins
                    p_wd[pp]  = wv_d[kk];
                    p_wv[pp]  = wv_v[kk];
                end
            l_has[pp] = (ln_pin1 == pp) | (ln_pin0 == pp);
            l_wd[pp]  = (ln_pin1 == pp) ? ln_drv[1] : (ln_pin0 == pp) & ln_drv[0];
            l_wv[pp]  = (ln_pin1 == pp) ? ln_val[1] : (ln_pin0 == pp) & ln_val[0];
        end
    end
    wire [15:0] pull_on = pulls[15:0], pull_val = pulls[31:16];

    // ui pins
    wire [7:0]  ui_chan;
    // uio pins (uio_res from the nets / uio_iso logic below)
    wire [7:0]  uio_chan;
    wire [7:0]  cont_bits, cont_ln_bits;
    generate
        for (g = 0; g < 8; g = g + 1) begin : g_chan
            wire ld_ui  = l_has[g] & l_wd[g];
            wire wd_ui  = p_has[g] & p_wd[g];
            assign ui_chan[g] = ld_ui ? l_wv[g] : wd_ui ? p_wv[g] : pull_on[g] ? pull_val[g] : in_sel[g];
            wire ld = l_has[g+8] & l_wd[g+8];
            wire wd = p_has[g+8] & p_wd[g+8];
            wire any = p_has[g+8] | l_has[g+8] | pull_on[g+8];
            assign uio_chan[g] = !any ? uio_res[g]
                               : ld ? l_wv[g+8] : wd ? p_wv[g+8]
                               : uio_oe[g] ? uio_out[g] : (pull_on[g+8] ? pull_val[g+8] : 1'b1);
            assign cont_bits[g]    = wd & uio_oe[g];
            assign cont_ln_bits[g] = ld & uio_oe[g];
        end
    endgenerate
    reg  [31:0] cont_cnt = 32'd0;
    reg  [11:0] cont_ln_cnt = 12'd0;
    always @(posedge clk60) if (chip_ce) begin
        cont_cnt    <= cont_cnt + cont_bits[0] + cont_bits[1] + cont_bits[2] + cont_bits[3]
                     + cont_bits[4] + cont_bits[5] + cont_bits[6] + cont_bits[7];
        cont_ln_cnt <= cont_ln_cnt + cont_ln_bits[0] + cont_ln_bits[1] + cont_ln_bits[2] + cont_ln_bits[3]
                     + cont_ln_bits[4] + cont_ln_bits[5] + cont_ln_bits[6] + cont_ln_bits[7];
    end
    assign contention = cont_cnt;

    // =========================================================================
    // CAN bus model (the Verilator harness's Y command). CAN_CFG: [7:0] dtx,
    // [15:8] drx, [19:16] node enables, [31:20] RXD ui pin of nodes 0-3 (3 bits
    // each); CAN_MASKS: TXD uio pin mask of nodes 0-3 (bytes 0-3). The external
    // node is waveform channel 7 (dominant while it drives 0), or async line 0
    // when LINE_CTRL routes it to the bus (pin code 16). The delay lines record
    // every chip edge from configuration (initial value 1).
    // =========================================================================
    wire [7:0]  can_dtx = can_cfg[7:0], can_drx = can_cfg[15:8];
    wire [3:0]  can_en  = can_cfg[19:16];
    wire [7:0]  pullm   = uio_oe & ~uio_out;
    wire        ext_wave = ~(wv_d[7] & ~wv_v[7]);
    wire        ext_line = (ln_pin0 == 5'd16) ? ~(ln_drv[0] & ~ln_val[0]) : 1'b1;
    wire [3:0]  net_k, netd_k;
    generate
        for (g = 0; g < 4; g = g + 1) begin : g_cannet
            assign net_k[g] = ~|(pullm & can_masks[8*g +: 8]);
            reg [255:0] nh = {256{1'b1}};
            always @(posedge clk60) if (chip_ce) nh <= {nh[254:0], net_k[g]};
            assign netd_k[g] = (can_dtx == 8'd0) ? net_k[g] : nh[can_dtx - 8'd1];
        end
    endgenerate
    wire        can_bus = ext_wave & ext_line & (&(netd_k | ~can_en));
    reg  [255:0] bh = {256{1'b1}};
    always @(posedge clk60) if (chip_ce) bh <= {bh[254:0], can_bus};
    wire        can_rx = (can_drx == 8'd0) ? can_bus : bh[can_drx - 8'd1];
    reg  [7:0]  ui_can, uio_can;
    integer nk;
    always @(*) begin
        ui_can  = ui_chan;
        uio_can = uio_chan;
        for (nk = 0; nk < 4; nk = nk + 1)
            if (can_en[nk]) begin
                ui_can[can_cfg[20 + 3*nk +: 3]] = can_rx;
                uio_can = (uio_can & ~can_masks[8*nk +: 8]) | (net_k[nk] ? can_masks[8*nk +: 8] : 8'd0);
            end
    end
    assign ui_fin  = ui_can;
    assign uio_fin = uio_can;
    assign line_status = {wave_rp, cont_ln_cnt, 1'b0, ln_ready, ln_busy, wave_empty};

    wire [47:0] skew = {skew_hi[29:24], skew_hi[21:16], skew_hi[13:8], skew_hi[5:0],
                        skew_lo[29:24], skew_lo[21:16], skew_lo[13:8], skew_lo[5:0]};
    generate
        for (g = 0; g < 8; g = g + 1) begin : g_skew
            reg  [63:0] uh = {64{1'b1}};         // resolved value, one bit per chip edge
            wire [5:0]  d  = skew[6*g +: 6];
            always @(posedge clk60) if (chip_ce) uh <= {uh[62:0], uio_fin[g]};
            assign uio_eff[g] = (d == 6'd0) ? uio_fin[g] : uh[d - 6'd1];
        end
    endgenerate

    assign spi_miso = uo_out[0];

    // =========================================================================
    // Board headers. Every pad is an IOBUF so the processor can read the
    // actual pad level back (PADS register).
    //   ck_io[i] (ui[i]) : input when SRC.ext[i]=1, else drives the value the
    //                      chip sees (monitor) when hdr_en=1
    //   ja[i]    (uo[i]) : output when hdr_en=1
    //   jb[i]    (uio[i]): output when hdr_en=1 and uio_oe[i]=1, else input
    // =========================================================================
    generate
        for (g = 0; g < 8; g = g + 1) begin : g_io
            IOBUF u_ck (.IO(ck_io[g]), .I(in_nonext[g]), .T(~(c_hdr_en & ~src_ext[g])), .O(ck_in[g]));
            IOBUF u_ja (.IO(ja[g]),    .I(uo_out[g]),    .T(~c_hdr_en),                 .O(ja_in[g]));
            IOBUF u_jb (.IO(jb[g]),    .I(uio_out[g]),   .T(~(c_hdr_en & uio_oe[g])),   .O(jb_in[g]));
        end
    endgenerate

    // =========================================================================
    // Logic capture
    // =========================================================================
    wire [31:0] sample = {uio_oe, uio_eff, uo_out, ui_eff};
    wire        cap_armed, cap_trig, cap_done;
    wire [CAP_AW-1:0] cap_trig_addr, cap_wr_addr;
    wire [CAP_AW-1:0] cap_bram_waddr;

    rh_capture #(.AW(CAP_AW)) u_cap (
        .clk         (clk60),
        .rstn        (aresetn),
        .sample_valid(chip_ce),
        .sample      (sample),
        .arm         (c_cap_arm),
        .force_trig  (c_cap_force),
        .mode        (c_trig_mode),
        .mask        (trig_mask),
        .value       (trig_value),
        .length      (cap_cfg[15:0]),
        .pre         (cap_cfg[31:16]),
        .div         (c_cap_div),
        .armed       (cap_armed),
        .triggered   (cap_trig),
        .done        (cap_done),
        .trig_addr   (cap_trig_addr),
        .wr_addr     (cap_wr_addr),
        .bram_en     (cap_bram_en),
        .bram_we     (cap_bram_we),
        .bram_waddr  (cap_bram_waddr),
        .bram_din    (cap_bram_din)
    );

    assign cap_bram_clk  = clk60;
    assign cap_bram_rst  = 1'b0;
    assign cap_bram_addr = {{(30-CAP_AW){1'b0}}, cap_bram_waddr, 2'b00};   // byte address

    // CAP_AW <= 14 (zero-extended into 14-bit fields)
    wire [13:0] trig14 = cap_trig_addr;
    wire [13:0] wr14   = cap_wr_addr;
    assign cap_status = {cap_done, cap_trig, wr14, 2'b00, trig14};

    // =========================================================================
    // Change trace (lockstep regression): one entry per pin change, stamped
    // with the lockstep cycle counter
    // =========================================================================
    wire [15:0]       trc_wp;
    wire              trc_overflow, trc_first;
    wire              trc_bram_en_w;
    wire [3:0]        trc_we;
    wire [TRC_AW-1:0] trc_waddr;
    wire [31:0]       trc_din0, trc_din1, trc_din2;
    rh_trace #(.AW(TRC_AW)) u_trace (
        .clk(clk60), .rstn(aresetn), .valid(chip_ce), .cycle(lcyc),
        .pins({uio_eff, uio_oe, uio_out, uo_out, ui_eff}),
        .flags({4'd0, ln_val, async_tx, fault_inv}),
        .en(l_trace_en), .clear(p_trace_clr), .rp(trace_rp[15:0]),
        .wp(trc_wp), .overflow(trc_overflow), .almost_full(trc_almost_full),
        .first_pending(trc_first),
        .bram_en(trc_bram_en_w), .bram_we(trc_we), .bram_waddr(trc_waddr),
        .din0(trc_din0), .din1(trc_din1), .din2(trc_din2));
    wire [31:0] trc_addr = {{(30-TRC_AW){1'b0}}, trc_waddr, 2'b00};   // byte address
    assign trc0_bram_clk = clk60;  assign trc1_bram_clk = clk60;  assign trc2_bram_clk = clk60;
    assign trc0_bram_rst = 1'b0;   assign trc1_bram_rst = 1'b0;   assign trc2_bram_rst = 1'b0;
    assign trc0_bram_en  = trc_bram_en_w; assign trc1_bram_en = trc_bram_en_w; assign trc2_bram_en = trc_bram_en_w;
    assign trc0_bram_we  = trc_we; assign trc1_bram_we = trc_we;  assign trc2_bram_we = trc_we;
    assign trc0_bram_addr = trc_addr; assign trc1_bram_addr = trc_addr; assign trc2_bram_addr = trc_addr;
    assign trc0_bram_din = trc_din0; assign trc1_bram_din = trc_din1; assign trc2_bram_din = trc_din2;

    assign ls_status = {trc_wp, 7'd0, trc_first, bp_stall, chip_ce, async_busy, f_seen, f_armed,
                        trc_overflow, eng_busy, run_rem != 32'd0};

    // =========================================================================
    // Frequency meters: chip clock edges counted per 1 s of reference clock
    // =========================================================================
    wire [3:0] seq_sys, seq_fclk;
    rh_freqmeter #(.REF_CYCLES(125000000)) u_fm_sys (
        .ref_clk(sysclk),   .clk(clk60), .count(cycles), .result(freq_sys),  .seq(seq_sys));
    rh_freqmeter #(.REF_CYCLES(100000000)) u_fm_fclk (
        .ref_clk(ref_fclk), .clk(clk60), .count(cycles), .result(freq_fclk), .seq(seq_fclk));

    // =========================================================================
    // Read-back words
    // =========================================================================
    localparam [7:0] HW_ID = 8'h52;    // 'R'
    assign status   = {HW_ID, seq_fclk, seq_sys, ui_eff,
                       3'b000, chip_ce, cap_done, cap_trig, cap_armed, mmcm_locked};
    assign chip_out = {uio_eff, uio_oe, uio_out, uo_out};
    assign pads     = {8'h00, jb_in, ja_in, ck_in};

    // =========================================================================
    // LEDs: LD0 chip-clock heartbeat, LD1 rst_n, LD2 capture done, LD3 hdr_en
    // =========================================================================
    reg [3:0] led_q = 4'd0;
    always @(posedge clk60) led_q <= {c_hdr_en, cap_done, c_rst_n, cycles[25]};
    assign led = led_q;

endmodule

`default_nettype wire
