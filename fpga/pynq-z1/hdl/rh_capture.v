// Logic capture for the Rosetta harness.
//
// One 32-bit sample per chip clock edge (optionally decimated) is written into
// a circular BRAM buffer. Arming (rising edge of `arm`) first fills `pre`
// samples, then waits for the trigger, then writes `length-pre-1` more samples
// after the trigger sample and stops (done). The trigger sample is at
// trig_addr; the window is [trig_addr-pre, trig_addr+length-pre-1] (mod depth).
// Holding `arm` low returns to idle and clears the flags.
//
// Trigger modes (mode), evaluated on each stored sample s (p = previous sample):
//   0 immediate        : first eligible sample
//   1 level            : (s & mask) == (value & mask)
//   2 edge into match  : match(s) && !match(p)
//   3 change           : ((s ^ p) & mask) != 0
// force_trig=1 triggers on the next eligible sample regardless of mode.
`timescale 1ns / 1ps
`default_nettype none

module rh_capture #(
    parameter integer AW = 14
) (
    input  wire          clk,
    input  wire          rstn,
    input  wire          sample_valid,   // chip is clocked at this clk edge
    input  wire [31:0]   sample,         // pin values the chip samples at this edge
    input  wire          arm,
    input  wire          force_trig,
    input  wire [1:0]    mode,
    input  wire [31:0]   mask,
    input  wire [31:0]   value,
    input  wire [15:0]   length,         // total samples, 1..2**AW (0 means 2**AW)
    input  wire [15:0]   pre,            // pre-trigger samples, < length
    input  wire [15:0]   div,            // store 1 of every div+1 chip cycles
    output wire          armed,
    output reg           triggered,
    output wire          done,
    output reg  [AW-1:0] trig_addr,
    output reg  [AW-1:0] wr_addr,
    output wire          bram_en,
    output wire [3:0]    bram_we,
    output wire [AW-1:0] bram_waddr,
    output wire [31:0]   bram_din
);

    localparam [2:0] S_IDLE = 3'd0, S_FILL = 3'd1, S_WAIT = 3'd2, S_POST = 3'd3, S_DONE = 3'd4;

    reg  [2:0]  state;
    reg         arm_q;
    reg  [15:0] div_cnt;
    reg  [31:0] s1;          // registered sample
    reg         s1_v;
    reg  [31:0] prev;
    reg         prev_v;
    reg  [16:0] cnt;         // pre-fill count, then post-trigger remaining

    wire arm_rise = arm & ~arm_q;
    wire [16:0] len17  = (length == 16'd0) ? (17'd1 << AW) : {1'b0, length};
    wire [16:0] post_n = len17 - {1'b0, pre} - 17'd1;   // samples after the trigger one

    wire match_now  = ((s1   ^ value) & mask) == 32'd0;
    wire match_prev = ((prev ^ value) & mask) == 32'd0;
    reg  cond;
    always @(*) begin
        case (mode)
            2'd0: cond = 1'b1;
            2'd1: cond = match_now;
            2'd2: cond = prev_v & match_now & ~match_prev;
            default: cond = prev_v & (((s1 ^ prev) & mask) != 32'd0);
        endcase
        cond = cond | force_trig;
    end

    wire writing = s1_v & ((state == S_FILL) | (state == S_WAIT) | (state == S_POST));

    always @(posedge clk) begin
        if (!rstn) begin
            state     <= S_IDLE;
            arm_q     <= 1'b0;
            div_cnt   <= 16'd0;
            s1        <= 32'd0;
            s1_v      <= 1'b0;
            prev      <= 32'd0;
            prev_v    <= 1'b0;
            cnt       <= 17'd0;
            wr_addr   <= {AW{1'b0}};
            trig_addr <= {AW{1'b0}};
            triggered <= 1'b0;
        end else begin
            arm_q <= arm;

            // stage 1: sample register + decimation
            if (arm_rise) begin
                div_cnt <= 16'd0;
                s1_v    <= 1'b0;
            end else if (sample_valid) begin
                s1_v    <= (div_cnt == 16'd0);
                s1      <= sample;
                div_cnt <= (div_cnt >= div) ? 16'd0 : div_cnt + 16'd1;
            end else begin
                s1_v    <= 1'b0;
            end

            // stage 2: store + trigger FSM
            if (!arm) begin
                state     <= S_IDLE;
                triggered <= 1'b0;
            end else if (arm_rise) begin
                state     <= (pre == 16'd0) ? S_WAIT : S_FILL;
                wr_addr   <= {AW{1'b0}};
                cnt       <= 17'd0;
                prev_v    <= 1'b0;
                triggered <= 1'b0;
            end else if (writing) begin
                wr_addr <= wr_addr + {{(AW-1){1'b0}}, 1'b1};
                prev    <= s1;
                prev_v  <= 1'b1;
                case (state)
                    S_FILL: begin
                        cnt <= cnt + 17'd1;
                        if (cnt + 17'd1 >= {1'b0, pre}) state <= S_WAIT;
                    end
                    S_WAIT: if (cond) begin
                        trig_addr <= wr_addr;
                        triggered <= 1'b1;
                        cnt       <= post_n;
                        state     <= (post_n == 17'd0) ? S_DONE : S_POST;
                    end
                    S_POST: begin
                        cnt <= cnt - 17'd1;
                        if (cnt == 17'd1) state <= S_DONE;
                    end
                    default: ;
                endcase
            end
        end
    end

    assign armed      = (state != S_IDLE) & (state != S_DONE);
    assign done       = (state == S_DONE);
    assign bram_en    = writing;
    assign bram_we    = {4{writing}};
    assign bram_waddr = wr_addr;
    assign bram_din   = s1;

endmodule

`default_nettype wire
