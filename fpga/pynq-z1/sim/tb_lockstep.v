// Lockstep testbench: the harness with the real chip, driven through its
// register interface the way python/board_server.py drives the board, by a
// command file in the Verilator harness's line protocol
// (toolchain/tests/rtl/rosetta_tb.cpp): R W X L N S F C D Q.
// Replies go to the output file in the same format, so
//     Vtt_um_jesselevine727_rosetta < cmds > ref.txt
//     vvp tb.vvp +cmds=cmds +out=fpga.txt ; diff ref.txt fpga.txt
// checks that the FPGA harness reproduces the Verilator harness edge for edge.
// Run with sim/run_lockstep.sh.
`timescale 1ns / 1ps
`default_nettype none

module tb_lockstep;
    reg clk60 = 0, sysclk = 0, fclk = 0;
    always #8.333 clk60  = ~clk60;     // 60 MHz
    always #4.000 sysclk = ~sysclk;    // 125 MHz
    always #5.000 fclk   = ~fclk;      // 100 MHz

    reg aresetn = 0;
    reg [31:0] ctrl = 32'h3, drive = 32'h0000ff00, src_sel = 0, route2 = 32'h0, loop_lo = 0, loop_hi = 0;
    reg [31:0] trig_mask = 0, trig_value = 0, cap_cfg = 0;
    reg [31:0] ls_ctrl = 0, run_n = 0, host_cmd = 0, host_data = 0, nets = 0, trace_rp = 0;
    reg [31:0] skew_lo = 0, skew_hi = 0, fault_off = 0, fault_len = 0, async_inc = 0, async_cmd = 0;
    reg [31:0] wave_wp = 0, chmap_lo = 0, chmap_hi = 0, pulls = 0, can_cfg = 0, can_masks = 0;
    reg [31:0] line_ctrl = 32'h7FE;
    wire [31:0] wave_status, contention, line_status;
    wire w_clk, w_rst, w_en, l_clk, l_rst, l_en;
    wire [3:0] w_we, l_we;
    wire [31:0] w_addr, w_din, l_addr, l_din;
    reg  [31:0] w_dout = 0, l_dout = 0;
    wire [31:0] status, chip_out, cap_status, pads, chip_cycles, freq_sys, freq_fclk;
    wire [31:0] ls_status, host_rdata, cyc_lo, cyc_hi;
    wire spi_miso;
    wire cap_bram_clk, cap_bram_rst, cap_bram_en;
    wire [3:0] cap_bram_we;
    wire [31:0] cap_bram_addr, cap_bram_din;
    wire [2:0] t_clk, t_rst, t_en;
    wire [3:0] t_we0, t_we1, t_we2;
    wire [31:0] t_addr0, t_addr1, t_addr2, t_din0, t_din1, t_din2;
    wire [7:0] ck_io, ja, jb;
    wire [3:0] led;

    genvar gi;
    generate for (gi = 0; gi < 8; gi = gi + 1) begin : g_pull
        pulldown (ck_io[gi]); pulldown (ja[gi]); pullup (jb[gi]);
    end endgenerate

    rosetta_harness #(.CAP_AW(8), .TRC_AW(14)) dut (
        .clk60(clk60), .clk60_raw(clk60), .aresetn(aresetn), .ref_fclk(fclk), .sysclk(sysclk),
        .mmcm_locked(1'b1),
        .ctrl(ctrl), .status(status), .drive(drive), .chip_out(chip_out),
        .src_sel(src_sel), .route2(route2), .loop_lo(loop_lo), .loop_hi(loop_hi),
        .trig_mask(trig_mask), .trig_value(trig_value), .cap_cfg(cap_cfg), .cap_status(cap_status),
        .pads(pads), .chip_cycles(chip_cycles), .freq_sys(freq_sys), .freq_fclk(freq_fclk),
        .ls_ctrl(ls_ctrl), .ls_status(ls_status), .run_n(run_n), .host_cmd(host_cmd),
        .host_data(host_data), .host_rdata(host_rdata), .nets(nets), .trace_rp(trace_rp),
        .skew_lo(skew_lo), .skew_hi(skew_hi), .fault_off(fault_off), .fault_len(fault_len),
        .cyc_lo(cyc_lo), .cyc_hi(cyc_hi), .async_inc(async_inc), .async_cmd(async_cmd),
        .wave_wp(wave_wp), .wave_status(wave_status), .chmap_lo(chmap_lo), .chmap_hi(chmap_hi),
        .pulls(pulls), .contention(contention), .can_cfg(can_cfg), .can_masks(can_masks),
        .line_ctrl(line_ctrl), .line_status(line_status),
        .wave_bram_clk(w_clk), .wave_bram_rst(w_rst), .wave_bram_en(w_en), .wave_bram_we(w_we),
        .wave_bram_addr(w_addr), .wave_bram_din(w_din), .wave_bram_dout(w_dout),
        .line_bram_clk(l_clk), .line_bram_rst(l_rst), .line_bram_en(l_en), .line_bram_we(l_we),
        .line_bram_addr(l_addr), .line_bram_din(l_din), .line_bram_dout(l_dout),
        .spi_sck(1'b0), .spi_mosi(1'b0), .spi_miso(spi_miso), .spi_ss_n(1'b1),
        .cap_bram_clk(cap_bram_clk), .cap_bram_rst(cap_bram_rst), .cap_bram_en(cap_bram_en),
        .cap_bram_we(cap_bram_we), .cap_bram_addr(cap_bram_addr), .cap_bram_din(cap_bram_din),
        .cap_bram_dout(32'd0),
        .trc0_bram_clk(t_clk[0]), .trc0_bram_rst(t_rst[0]), .trc0_bram_en(t_en[0]), .trc0_bram_we(t_we0),
        .trc0_bram_addr(t_addr0), .trc0_bram_din(t_din0), .trc0_bram_dout(32'd0),
        .trc1_bram_clk(t_clk[1]), .trc1_bram_rst(t_rst[1]), .trc1_bram_en(t_en[1]), .trc1_bram_we(t_we1),
        .trc1_bram_addr(t_addr1), .trc1_bram_din(t_din1), .trc1_bram_dout(32'd0),
        .trc2_bram_clk(t_clk[2]), .trc2_bram_rst(t_rst[2]), .trc2_bram_en(t_en[2]), .trc2_bram_we(t_we2),
        .trc2_bram_addr(t_addr2), .trc2_bram_din(t_din2), .trc2_bram_dout(32'd0),
        .ck_io(ck_io), .ja(ja), .jb(jb), .led(led));
    defparam dut.u_fm_sys.REF_CYCLES  = 1250;
    defparam dut.u_fm_fclk.REF_CYCLES = 1000;

    reg [31:0] m0 [0:16383];
    reg [31:0] m1 [0:16383];
    reg [31:0] m2 [0:16383];
    always @(posedge clk60) if (t_en[0] && t_we0 == 4'hf) m0[t_addr0[15:2]] <= t_din0;
    always @(posedge clk60) if (t_en[1] && t_we1 == 4'hf) m1[t_addr1[15:2]] <= t_din1;
    always @(posedge clk60) if (t_en[2] && t_we2 == 4'hf) m2[t_addr2[15:2]] <= t_din2;
    reg [31:0] mw [0:16383];
    always @(posedge w_clk) if (w_en) w_dout <= mw[w_addr[15:2]];
    reg [31:0] ml [0:16383];
    always @(posedge l_clk) if (l_en) l_dout <= ml[l_addr[15:2]];

    // ---------------------------------------------------------------- "processor"
    task waitc(input integer n); begin repeat (n) @(posedge clk60); end endtask
    task tgl(input integer b); begin @(posedge clk60) ls_ctrl[b] <= ~ls_ctrl[b]; waitc(3); end endtask
    task wait_idle; begin while (ls_status[1:0] != 2'b00) @(posedge clk60); waitc(2); end endtask

    // software copy of the trace (as the board server keeps it)
    integer sw_n = 0;
    integer wq_rem = 0;                 // waveform cycles still queued (as the server tracks it)
    task wq_used(input integer n); begin wq_rem = (wq_rem > n) ? wq_rem - n : 0; end endtask
    reg [47:0] sw_c [0:262143];
    reg [39:0] sw_p [0:262143];
    task drain;
        integer wp, i, idx;
        begin
            wp = ls_status[31:16];
            while (trace_rp[15:0] != wp[15:0]) begin
                idx = trace_rp[13:0];
                sw_c[sw_n] = {m1[idx][15:0], m0[idx]};
                sw_p[sw_n] = {m2[idx], m1[idx][23:16]};   // {uio_in, uio_oe, uio_out, uo, ui}
                sw_n = sw_n + 1;
                trace_rp[15:0] = trace_rp[15:0] + 1;
            end
            waitc(2);
        end
    endtask
    task run(input integer n);
        begin
            if (n > 0) begin
                @(posedge clk60) run_n <= n;
                tgl(0);
                while (ls_status[0]) begin waitc(50); drain; end
                wait_idle;
                wq_used(n);
            end
            drain;
        end
    endtask
    task host(input [7:0] cmd, input [31:0] d);
        begin
            @(posedge clk60) begin host_cmd <= cmd; host_data <= d; end
            tgl(1);
            while (ls_status[1]) begin waitc(50); drain; end
            wait_idle;
            wq_used(cmd[7] ? 792 : 664);
            drain;
        end
    endtask

    integer nnets = 0, nwires = 0, ncan = 0, a3, a4;
    integer fin, fout, r, a1, a2, i;
    reg [8*80-1:0] line;
    reg [8*256-1:0] cmds_path, out_path;
    reg [7:0] c;
    initial begin
        if (!$value$plusargs("cmds=%s", cmds_path)) cmds_path = "cmds.txt";
        if (!$value$plusargs("out=%s", out_path)) out_path = "fpga.txt";
        fin = $fopen(cmds_path, "r");
        fout = $fopen(out_path, "w");
        waitc(5); aresetn <= 1; waitc(5);
        // session start, as board_server.py does it
        ls_ctrl <= (32'd1 << 16) | (32'd1 << 7) | (32'd1 << 6) | (32'd1 << 5) | (32'd1 << 3);
        waitc(2);
        tgl(2); tgl(4);
        trace_rp <= {16'd0, ls_status[31:16]}; waitc(2);
        ctrl <= 32'h2;           // rst_n = 0, ena, clock stopped (a negedge, so the
                                 // chip's async-reset flops start at 0 as on the FPGA)
        run(10);
        @(posedge clk60) ctrl <= 32'h3;
        run(10);
        while (!$feof(fin)) begin
            line = 0;
            r = $fgets(line, fin);
            if (r > 0) begin
                // first character of the line
                c = 0;
                for (i = 79; i >= 0; i = i - 1) if (c == 0 && line[8*i +: 8] != 0) c = line[8*i +: 8];
                case (c)
                    "R": begin r = $sscanf(line, "R %d", a1); run(a1); $fdisplay(fout, "ok"); end
                    "W": begin r = $sscanf(line, "W %h %h", a1, a2); host({1'b0, a1[6:0]}, a2); $fdisplay(fout, "ok"); end
                    "X": begin r = $sscanf(line, "X %h", a1); host({1'b1, a1[6:0]}, 0); $fdisplay(fout, "%08x", host_rdata); end
                    "L": begin
                        r = $sscanf(line, "L %d %d", a1, a2);
                        loop_lo[4*a2 +: 4] <= a1[3:0]; src_sel[16 + a2] <= 1'b1;
                        if (nwires == 0) ls_ctrl[15:12] <= a1[3:0];
                        nwires = nwires + 1; waitc(2); $fdisplay(fout, "ok");
                    end
                    "N": begin r = $sscanf(line, "N %h", a1); nets[8*nnets +: 8] <= a1[7:0]; nnets = nnets + 1; waitc(2); $fdisplay(fout, "ok"); end
                    "S": begin
                        r = $sscanf(line, "S %d %d", a1, a2);
                        if (a1 < 4) skew_lo[8*a1 +: 6] <= a2[5:0]; else skew_hi[8*(a1-4) +: 6] <= a2[5:0];
                        waitc(2); $fdisplay(fout, "ok");
                    end
                    "F": begin
                        r = $sscanf(line, "F %d %d", a1, a2);
                        fault_off <= a1; fault_len <= a2; waitc(2); tgl(8); $fdisplay(fout, "ok");
                    end
                    "C": begin $fdisplay(fout, "%0d", {cyc_hi[15:0], cyc_lo}); end
                    "M": begin
                        r = $sscanf(line, "M %d %d", a1, a2);
                        if (a1 < 4) chmap_lo[8*a1 +: 5] <= (a2 < 0) ? 5'd0 : {1'b1, a2[3:0]};
                        else        chmap_hi[8*(a1-4) +: 5] <= (a2 < 0) ? 5'd0 : {1'b1, a2[3:0]};
                        waitc(2); $fdisplay(fout, "ok");
                    end
                    "U": begin
                        r = $sscanf(line, "U %d %d", a1, a2);
                        pulls[a1] <= (a2 >= 0); pulls[16 + a1] <= (a2 == 1);
                        waitc(2); $fdisplay(fout, "ok");
                    end
                    "v": begin                  // one run of a V command (pre-split): no reply
                        r = $sscanf(line, "v %h %h %d", a1, a2, a3);
                        mw[wave_wp[13:0]] = {a1[7:0], a2[7:0], a3[15:0]};
                        @(posedge clk60); #1 wave_wp[15:0] = wave_wp[15:0] + 1;
                        wq_rem = wq_rem + a3;
                    end
                    "V": begin waitc(2); $fdisplay(fout, "ok"); end
                    "P": begin r = $sscanf(line, "P %d", a1); run(wq_rem + a1); $fdisplay(fout, "ok"); end
                    "K": begin $fdisplay(fout, "%0d", contention); end
                    "T": begin r = $sscanf(line, "T %d", a1); ls_ctrl[3] <= (a1 != 0); waitc(2); $fdisplay(fout, "ok"); end
                    "Y": begin                  // pre-split: "Y dtx drx n m0 r0 m1 r1 m2 r2 m3 r3"
                        r = $sscanf(line, "Y %d %d %d", a1, a2, a3);
                        can_cfg[7:0] <= a1; can_cfg[15:8] <= a2;
                        can_cfg[19:16] <= (a3 == 0) ? 4'h0 : (a3 == 1) ? 4'h1 : (a3 == 2) ? 4'h3 : (a3 == 3) ? 4'h7 : 4'hF;
                        $fdisplay(fout, "ok");
                    end
                    "y": begin                  // node k: "y k mask rx"
                        r = $sscanf(line, "y %d %h %d", a1, a2, a3);
                        can_masks[8*a1 +: 8] <= a2[7:0]; can_cfg[20 + 3*a1 +: 3] <= a3[2:0];
                        waitc(2);
                    end
                    "D": begin
                        drain;
                        for (i = 0; i < sw_n; i = i + 1)
                            $fdisplay(fout, "%0d %02x %02x %02x %02x %02x", sw_c[i], sw_p[i][15:8], sw_p[i][7:0],
                                      sw_p[i][23:16], sw_p[i][31:24], sw_p[i][39:32]);
                        sw_n = 0;
                        $fdisplay(fout, "end");
                    end
                    "Q": begin $fclose(fout); $finish; end
                    default: ;
                endcase
            end
        end
        $fclose(fout);
        $finish;
    end
endmodule
`default_nettype wire
