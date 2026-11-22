// Self-checking testbench for rosetta_harness with the placeholder chip
// (uo_out = ui_in + uio_in). Run with sim/run_sim.sh.
`timescale 1ns / 1ps
`default_nettype none

module tb_harness;
    reg clk60 = 0, sysclk = 0, fclk = 0;
    always #8.333 clk60  = ~clk60;     // 60 MHz
    always #4.000 sysclk = ~sysclk;    // 125 MHz
    always #5.000 fclk   = ~fclk;      // 100 MHz

    reg aresetn = 0;
    reg [31:0] ctrl = 32'h6, drive = 0, src_sel = 0, route2 = 32'h7, loop_lo = 0, loop_hi = 0;
    reg [31:0] trig_mask = 0, trig_value = 0, cap_cfg = 0;
    wire [31:0] status, chip_out, cap_status, pads, chip_cycles, freq_sys, freq_fclk;
    reg spi_sck = 0, spi_mosi = 0, spi_ss_n = 1;
    wire spi_miso;
    wire cap_bram_clk, cap_bram_rst, cap_bram_en;
    wire [3:0] cap_bram_we;
    wire [31:0] cap_bram_addr, cap_bram_din;
    wire [7:0] ck_io, ja, jb;
    wire [3:0] led;
    reg  [7:0] ck_drv = 8'hzz;

    // weak board pulls; an external driver on ck_io when wanted
    genvar gi;
    generate for (gi = 0; gi < 8; gi = gi + 1) begin : g_pull
        pulldown (ck_io[gi]); pulldown (ja[gi]); pullup (jb[gi]);
        assign ck_io[gi] = ck_drv[gi];
    end endgenerate

    rosetta_harness #(.CAP_AW(8)) dut (
        .clk60(clk60), .clk60_raw(clk60), .aresetn(aresetn), .ref_fclk(fclk), .sysclk(sysclk),
        .mmcm_locked(1'b1),
        .ctrl(ctrl), .status(status), .drive(drive), .chip_out(chip_out),
        .src_sel(src_sel), .route2(route2), .loop_lo(loop_lo), .loop_hi(loop_hi),
        .trig_mask(trig_mask), .trig_value(trig_value), .cap_cfg(cap_cfg), .cap_status(cap_status),
        .pads(pads), .chip_cycles(chip_cycles), .freq_sys(freq_sys), .freq_fclk(freq_fclk),
        .ls_ctrl(32'd0), .ls_status(), .run_n(32'd0), .host_cmd(32'd0), .host_data(32'd0),
        .host_rdata(), .nets(32'd0), .trace_rp(32'd0), .skew_lo(32'd0), .skew_hi(32'd0),
        .fault_off(32'd0), .fault_len(32'd0), .cyc_lo(), .cyc_hi(), .async_inc(32'd0), .async_cmd(32'd0),
        .trc0_bram_clk(), .trc0_bram_rst(), .trc0_bram_en(), .trc0_bram_we(), .trc0_bram_addr(),
        .trc0_bram_din(), .trc0_bram_dout(32'd0),
        .trc1_bram_clk(), .trc1_bram_rst(), .trc1_bram_en(), .trc1_bram_we(), .trc1_bram_addr(),
        .trc1_bram_din(), .trc1_bram_dout(32'd0),
        .trc2_bram_clk(), .trc2_bram_rst(), .trc2_bram_en(), .trc2_bram_we(), .trc2_bram_addr(),
        .trc2_bram_din(), .trc2_bram_dout(32'd0),
        .spi_sck(spi_sck), .spi_mosi(spi_mosi), .spi_miso(spi_miso), .spi_ss_n(spi_ss_n),
        .cap_bram_clk(cap_bram_clk), .cap_bram_rst(cap_bram_rst), .cap_bram_en(cap_bram_en),
        .cap_bram_we(cap_bram_we), .cap_bram_addr(cap_bram_addr), .cap_bram_din(cap_bram_din),
        .cap_bram_dout(32'd0), .ck_io(ck_io), .ja(ja), .jb(jb), .led(led));
    defparam dut.u_fm_sys.REF_CYCLES  = 1250;   // 10 us windows in sim
    defparam dut.u_fm_fclk.REF_CYCLES = 1000;

    reg [31:0] mem [0:255];
    always @(posedge cap_bram_clk) if (cap_bram_en && cap_bram_we == 4'hf) mem[cap_bram_addr[9:2]] <= cap_bram_din;

    integer errors = 0;
    task check(input [255:0] what, input [31:0] got, input [31:0] exp);
        begin
            if (got !== exp) begin errors = errors + 1; $display("FAIL %0s: got %h exp %h", what, got, exp); end
            else $display("ok   %0s = %h", what, got);
        end
    endtask
    task waitc(input integer n); begin repeat (n) @(posedge clk60); end endtask

    integer i, a, start, n_samp, trig_at;
    reg [31:0] c0, s;
    initial begin
        waitc(5); aresetn <= 1; waitc(5);
        check("STATUS id", status[31:24], 8'h52);

        // --- adder through processor-driven pins
        route2 <= 0;                       // ui[0..2] from GPIO too
        @(posedge clk60) drive <= {16'h0, 8'h10, 8'h35}; waitc(4);
        check("uo = 0x35+0x10", chip_out[7:0], 8'h45);
        check("ui_eff", status[15:8], 8'h35);
        @(posedge clk60) drive <= {16'h0, 8'hF0, 8'h20}; waitc(4);
        check("uo = 0x20+0xF0 (wrap)", chip_out[7:0], 8'h10);

        // --- host SPI routing
        route2 <= 7; spi_sck <= 1; spi_ss_n <= 0; spi_mosi <= 1; waitc(3);
        check("ui[2:0] from SPI", status[10:8], 3'b101);
        check("miso = uo[0]", spi_miso, chip_out[0]);
        route2 <= 0; spi_ss_n <= 1;

        // --- loopback uo[1] -> ui[7]
        @(posedge clk60) drive <= {16'h0, 8'h01, 8'h01};
        loop_lo <= (32'd1 << 28); src_sel <= (32'd1 << (16 + 7)); waitc(6);
        check("loop ui_eff", status[15:8], 8'h81);
        check("loop uo", chip_out[7:0], 8'h82);
        // loopback uo[1] -> uio_in[0]
        src_sel <= (32'd1 << (16 + 8)); loop_lo <= 0; loop_hi <= 32'd1; waitc(6);
        check("loop uio_eff", chip_out[31:24], 8'h01);
        src_sel <= 0; loop_hi <= 0;

        // --- header: pads tri-stated until hdr_en, then uo appears on ja
        @(posedge clk60) drive <= {16'h0, 8'h00, 8'h5a}; waitc(4);
        check("ja hi-Z (pulldown)", pads[15:8], 8'h00);
        check("jb hi-Z (pullup)", pads[23:16], 8'hff);
        ctrl <= ctrl | 32'h10; waitc(4);
        check("ja = uo", pads[15:8], 8'h5a);
        check("ck_io monitor = ui", pads[7:0], 8'h5a);
        // external drive on ck_io[3], select header source for ui[3]
        ck_drv <= 8'bzzzz_1zzz; src_sel <= 32'h0000_0008; @(posedge clk60) drive <= 0; waitc(4);
        check("ui[3] from header", status[15:8], 8'h08);
        ck_drv <= 8'hzz; src_sel <= 0; ctrl <= ctrl & ~32'h10;

        // --- capture: change trigger on ui, length 64, pre 8
        @(posedge clk60) drive <= 32'h0000_0011; waitc(3);
        trig_mask <= 32'h0000_00ff; cap_cfg <= {16'd8, 16'd64};
        ctrl <= (ctrl & ~32'h0000_0300) | (32'd3 << 8) | 32'h20;   // mode 3, arm
        waitc(40);
        check("armed, not triggered", status[3:1], 3'b001);
        @(posedge clk60) drive <= 32'h0000_0022; waitc(5); @(posedge clk60) drive <= 32'h0000_0033;
        waitc(80);
        check("done+triggered", status[3:1], 3'b110);
        trig_at = cap_status[13:0];
        start = (trig_at - 8) & 255;
        s = mem[(start + 7) & 255]; check("pre sample ui", s[7:0], 8'h11);
        s = mem[(start + 8) & 255]; check("trigger sample ui", s[7:0], 8'h22);
        check("trigger sample uo", s[15:8], 8'h22);
        s = mem[(start + 13) & 255]; check("trigger+5 still 0x22", s[7:0], 8'h22);
        s = mem[(start + 14) & 255]; check("trigger+6 = 0x33 (6 cycles later)", s[7:0], 8'h33);
        check("wr_addr = trig+56", cap_status[29:16], (trig_at + 56) & 255);
        ctrl <= ctrl & ~32'h20; waitc(3);
        check("disarmed", status[3:1], 3'b000);

        // --- level trigger with immediate re-arm and decimation 1 (every 2nd)
        @(posedge clk60) drive <= 32'h0000_0044;
        trig_mask <= 32'h0000_00ff; trig_value <= 32'h55; cap_cfg <= {16'd0, 16'd16};
        ctrl <= (ctrl & ~32'hffff_0300) | (32'd1 << 8) | (32'd1 << 16) | 32'h20; waitc(10);
        @(posedge clk60) drive <= 32'h0000_0055; waitc(60);
        check("level trig done", status[3:1], 3'b110);
        s = mem[cap_status[13:0]]; check("level trigger sample", s[7:0], 8'h55);
        ctrl <= ctrl & ~32'hffff_0320; waitc(3);

        // --- single step
        ctrl <= ctrl & ~32'h4; waitc(4);
        c0 = chip_cycles; waitc(10);
        check("clock stopped", chip_cycles, c0);
        for (i = 0; i < 3; i = i + 1) begin ctrl <= ctrl ^ 32'h8; waitc(4); end
        check("3 steps", chip_cycles - c0, 3);
        ctrl <= ctrl | 32'h4;

        // --- frequency meters (sim windows: 10 us -> 600 cycles)
        #25000;
        check("freq_sys ~600",  (freq_sys  >= 599 && freq_sys  <= 601), 1);
        check("freq_fclk ~600", (freq_fclk >= 599 && freq_fclk <= 601), 1);
        $display("     freq_sys=%0d freq_fclk=%0d", freq_sys, freq_fclk);

        if (errors == 0) $display("ALL TESTS PASSED"); else $display("%0d FAILURES", errors);
        $finish;
    end
endmodule
`default_nettype wire
