`default_nettype none
// ===========================================================================
//  Proof top for rosetta_stuff.
//
//  stuff_fv contains two independent proof problems sharing clk and rst_n:
//
//   u_dut      one rosetta_stuff with every input free (cfg changes only in
//              reset or with clear_i), checked by stuff_props (exact
//              behaviour, violation, single pulse, restart after violation,
//              clear, disabled incl. EQUAL N = 1, reset).
//   u_rt       round-trip harness (stuff_rt_harness): free data source,
//              transmitter using instance A, a wire with independent timing
//              on each side, receiver using instance B, a new free cfg per
//              frame, checked by stuff_rt_props (round trip, progress, no
//              violation, max run on the wire).
//
//  rst_n is low in the first cycle (assumed in stuff_props) and free after.
//  Spec v2.
// ===========================================================================

module stuff_fv (
    input wire       clk,
    input wire       rst_n,
    // free inputs of the single instance
    input wire       cfg_en_i,
    input wire [2:0] cfg_n_i,
    input wire       cfg_mode_i,
    input wire       clear_i,
    input wire       bit_valid_i,
    input wire       bit_i
);
    wire stuff_next_o, stuff_bit_o, violation_o;

    rosetta_stuff u_dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .cfg_en_i     (cfg_en_i),
        .cfg_n_i      (cfg_n_i),
        .cfg_mode_i   (cfg_mode_i),
        .clear_i      (clear_i),
        .bit_valid_i  (bit_valid_i),
        .bit_i        (bit_i),
        .stuff_next_o (stuff_next_o),
        .stuff_bit_o  (stuff_bit_o),
        .violation_o  (violation_o)
    );

    // CFG_CYCLE_CHECK(1): outputs are also checked in a clear cycle in which
    // cfg changes (spec v2 text). Use 0 to leave those cycles unchecked.
    stuff_props #(.STANDALONE(1), .CFG_CYCLE_CHECK(1)) u_props (
        .clk          (clk),
        .rst_n        (rst_n),
        .cfg_en_i     (cfg_en_i),
        .cfg_n_i      (cfg_n_i),
        .cfg_mode_i   (cfg_mode_i),
        .clear_i      (clear_i),
        .bit_valid_i  (bit_valid_i),
        .bit_i        (bit_i),
        .stuff_next_o (stuff_next_o),
        .stuff_bit_o  (stuff_bit_o),
        .violation_o  (violation_o)
    );

    stuff_rt_harness u_rt (
        .clk   (clk),
        .rst_n (rst_n)
    );
endmodule


// ===========================================================================
//  Round-trip harness.
//
//  source      : free data bit src_bit, offered every cycle; the transmitter
//                takes it only when it sends a data bit.
//  transmitter : when it sends (tx_go), the wire bit is A.stuff_bit_o if
//                A.stuff_next_o, else the next source bit. A watches the
//                wire it drives.
//  wire        : FIFO of DEPTH bits. The receiver takes bits at its own free
//                pace (rx_go), so A and B see the same bit sequence with
//                different timing.
//  receiver    : B watches every wire bit it takes; a bit taken while
//                B.stuff_next_o = 1 is dropped (and checked by B), others are
//                output as data.
//  frames      : clr_req starts a new frame when the wire is empty: A clears
//                at once, B clears later (free rx_clr_go) before taking any
//                bit of the new frame.
//  cfg         : free (all values: enable, N = 0-7, both modes), chosen per
//                frame: A takes a new cfg in reset or in its clear cycle, B
//                takes the same cfg in its own clear cycle (spec v2: cfg
//                changes only in reset or with clear_i).
// ===========================================================================
module stuff_rt_harness (
    input wire clk,
    input wire rst_n
);
    localparam DEPTH = 2;

    (* anyseq *) wire [4:0] cfg_new;   // {en, n[2:0], mode} for a new frame

    (* anyseq *) wire src_bit;
    (* anyseq *) wire tx_go;
    (* anyseq *) wire rx_go;
    (* anyseq *) wire clr_req;
    (* anyseq *) wire rx_clr_go;

    // wire FIFO, head at fb[0]
    logic [1:0]       cnt;
    logic [DEPTH-1:0] fb;
    logic             pend;

    wire a_sn, a_sb, a_viol;
    wire b_sn, b_sb, b_viol;

    wire a_clear = clr_req && (cnt == 2'd0) && !pend;
    wire a_valid = tx_go && !a_clear && (cnt < DEPTH);
    wire a_bit   = a_sn ? a_sb : src_bit;

    wire b_clear = pend && rx_clr_go;
    wire b_valid = rx_go && !pend && (cnt != 2'd0);
    wire b_bit   = fb[0];

    // per-side cfg: A changes in reset or its clear cycle; B adopts A's
    // (frame) cfg in its own clear cycle. A cannot clear again while B has
    // not cleared, so A's cfg is the frame cfg while pend is set.
    logic [4:0] a_cfg_q, b_cfg_q;
    wire  [4:0] a_cfg = (!rst_n || a_clear) ? cfg_new : a_cfg_q;
    wire  [4:0] b_cfg = !rst_n ? a_cfg : (b_clear ? a_cfg : b_cfg_q);
    always @(posedge clk) begin
        a_cfg_q <= a_cfg;
        b_cfg_q <= b_cfg;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt  <= 2'd0;
            fb   <= '0;
            pend <= 1'b0;
        end else begin
            case ({a_valid, b_valid})
                2'b10: begin fb[cnt] <= a_bit; cnt <= cnt + 2'd1; end   // cnt < DEPTH
                2'b01: begin fb <= fb >> 1;    cnt <= cnt - 2'd1; end
                2'b11: begin fb <= fb >> 1;    fb[cnt - 2'd1] <= a_bit; end
                default: ;
            endcase
            if (a_clear)      pend <= 1'b1;
            else if (b_clear) pend <= 1'b0;
        end
    end

    rosetta_stuff u_a (
        .clk          (clk),
        .rst_n        (rst_n),
        .cfg_en_i     (a_cfg[4]),
        .cfg_n_i      (a_cfg[3:1]),
        .cfg_mode_i   (a_cfg[0]),
        .clear_i      (a_clear),
        .bit_valid_i  (a_valid),
        .bit_i        (a_bit),
        .stuff_next_o (a_sn),
        .stuff_bit_o  (a_sb),
        .violation_o  (a_viol)
    );

    rosetta_stuff u_b (
        .clk          (clk),
        .rst_n        (rst_n),
        .cfg_en_i     (b_cfg[4]),
        .cfg_n_i      (b_cfg[3:1]),
        .cfg_mode_i   (b_cfg[0]),
        .clear_i      (b_clear),
        .bit_valid_i  (b_valid),
        .bit_i        (b_bit),
        .stuff_next_o (b_sn),
        .stuff_bit_o  (b_sb),
        .violation_o  (b_viol)
    );

    stuff_rt_props #(.DEPTH(DEPTH)) u_rt_props (
        .clk            (clk),
        .rst_n          (rst_n),
        .a_cfg_en_i     (a_cfg[4]),
        .a_cfg_n_i      (a_cfg[3:1]),
        .a_cfg_mode_i   (a_cfg[0]),
        .a_clear_i      (a_clear),
        .a_bit_valid_i  (a_valid),
        .a_bit_i        (a_bit),
        .a_stuff_next_o (a_sn),
        .a_stuff_bit_o  (a_sb),
        .a_violation_o  (a_viol),
        .b_cfg_en_i     (b_cfg[4]),
        .b_cfg_n_i      (b_cfg[3:1]),
        .b_cfg_mode_i   (b_cfg[0]),
        .b_clear_i      (b_clear),
        .b_bit_valid_i  (b_valid),
        .b_bit_i        (b_bit),
        .b_stuff_next_o (b_sn),
        .b_stuff_bit_o  (b_sb),
        .b_violation_o  (b_viol)
    );
endmodule
