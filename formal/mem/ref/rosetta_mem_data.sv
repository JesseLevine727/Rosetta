`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL of rosetta_mem_data. NOT the
//  design (src/rosetta_mem_data.sv was not read). Written from
//  docs/blocks/mem.md section 3 (and host.md section 4 for register
//  addresses) only, to show the properties in formal/mem/mem_data_props.sv
//  pass on a spec-conforming implementation and fail on plausible bugs
//  (planted under MUT_<NAME> defines, one at a time; see run_mutants.sh).
//
//  Per lane: isz, osz (2 bits), in_wr, in_rd, out_wr, out_rd (7 bits) = 128
//  flops, async active-low reset. Macro drive is combinational (mem.md 3.4),
//  through the thin wrapper rosetta_sram512x32, instance u_macro.
// ===========================================================================
module rosetta_mem_data (
    input  wire         clk,
    input  wire         rst_n,
    input  wire  [4:0]  ph_i,
    input  wire  [2:0]  dm_op_i,
    input  wire  [6:0]  dm_off_i,
    input  wire  [1:0]  dm_hmask_i,
    input  wire  [31:0] dm_wdata_i,
    input  wire         h_en_i,
    input  wire         h_we_i,
    input  wire  [8:0]  h_addr_i,
    input  wire  [31:0] h_wdata_i,
    input  wire  [3:0]  run_i,
    input  wire         hreg_we_i,
    input  wire  [6:0]  hreg_addr_i,
    input  wire  [31:0] hreg_wdata_i,
    output logic [31:0] hreg_rdata_o,
    output logic [31:0] rdata_o,
    output logic [3:0]  qe_o,
    output logic [3:0]  qf_o,
    output logic [3:0]  qne_o,
    output logic [15:0] obs_qcfg_o,
    output logic [27:0] obs_in_wr_o,
    output logic [27:0] obs_in_rd_o,
    output logic [27:0] obs_out_wr_o,
    output logic [27:0] obs_out_rd_o
);
    localparam [2:0] OP_NONE = 3'd0, OP_READ = 3'd1, OP_WRITE = 3'd2,
                     OP_POP = 3'd3, OP_PUSH = 3'd4;

    // ------------------------------------------------------------------ state
    logic [3:0][1:0] isz, osz;
    logic [3:0][6:0] in_wr, in_rd, out_wr, out_rd;

    // ---------------------------------------------------------------- helpers
    function automatic [6:0] f_size(input [1:0] c);       // S = 8 << c
`ifdef MUT_SIZE
        f_size = 7'd16 << c;                               // S doubled (64 -> 0)
`else
        f_size = 7'd8 << c;
`endif
    endfunction
    function automatic [6:0] f_m(input [1:0] c);           // m = 2S - 1
        case (c)
            2'd0: f_m = 7'd15;
            2'd1: f_m = 7'd31;
            2'd2: f_m = 7'd63;
            default: f_m = 7'd127;
        endcase
    endfunction
    function automatic [6:0] f_idx(input [6:0] p, input [1:0] c);
`ifdef MUT_IDX_MASK
        f_idx = p & f_m(c);                                // wrap bit kept in index
`else
        f_idx = p & (f_size(c) - 7'd1);
`endif
    endfunction
    function automatic f_empty(input [6:0] wr, input [6:0] rd, input [1:0] c);
`ifdef MUT_EMPTY_RAW
        f_empty = (wr == rd);                              // pointers not masked
`else
        f_empty = ((wr & f_m(c)) == (rd & f_m(c)));
`endif
    endfunction
    function automatic f_full(input [6:0] wr, input [6:0] rd, input [1:0] c);
`ifdef MUT_FULL_IDX
        f_full = ((wr ^ rd) & (f_size(c) - 7'd1)) == 7'd0 && !f_empty(wr, rd, c) && (wr[6] != rd[6]);
`else
        f_full = (((wr ^ rd) & f_m(c)) == f_size(c));
`endif
    endfunction
    function automatic [6:0] f_next(input [6:0] p, input [1:0] c);
`ifdef MUT_WRAP
        f_next = ((p & f_m(c)) + 7'd1) & (f_size(c) - 7'd1); // wrap bit lost
`elsif MUT_NEXT_RAW
        f_next = p + 7'd1;                                 // not re-masked
`else
        f_next = ((p & f_m(c)) + 7'd1) & f_m(c);
`endif
    endfunction

    // ------------------------------------------------------------------ flags
    logic [3:0] fi_full, fo_empty;
    always_comb begin
        for (int l = 0; l < 4; l++) begin
            qe_o[l]     = f_empty(in_wr[l], in_rd[l], isz[l]);
            fi_full[l]  = f_full(in_wr[l], in_rd[l], isz[l]);
`ifdef MUT_QF_ISZ
            qf_o[l]     = f_full(out_wr[l], out_rd[l], isz[l]); // wrong size code
`else
            qf_o[l]     = f_full(out_wr[l], out_rd[l], osz[l]);
`endif
            fo_empty[l] = f_empty(out_wr[l], out_rd[l], osz[l]);
            qne_o[l]    = !fo_empty[l];
        end
    end

    // ------------------------------------------------------------------ owner
    logic [1:0] o;
    always_comb begin
`ifdef MUT_OWNER
        o = ph_i[0] ? 2'd0 : ph_i[2] ? 2'd3 : ph_i[3] ? 2'd1 : 2'd2; // L0/L3 swapped
`else
        o = ph_i[0] ? 2'd3 : ph_i[2] ? 2'd0 : ph_i[3] ? 2'd1 : 2'd2;
`endif
    end

`ifdef MUT_HOST_ANYPH
    wire host_ph = 1'b1;                       // host requests honoured in any phase
`elsif MUT_LANE_IN_PH1
    wire host_ph = ph_i[1] && h_en_i;          // lane request in ph = 1 not ignored
`else
    wire host_ph = ph_i[1];
`endif

    // ------------------------------------------------------------ macro drive
    logic        en, we;
    logic [8:0]  addr;
    logic [31:0] wdata, bmask;
    logic        pop_tk, push_tk;
    wire  [1:0]  isz_o = isz[o];
    wire  [1:0]  osz_o = osz[o];
    wire  [6:0]  in_off  = f_idx(in_rd[o], isz_o);
`ifdef MUT_OUT_BASE_ISZ
    wire  [6:0]  out_base = 7'd0 - f_size(isz_o);          // base from isz
`elsif MUT_OUT_BASE_OFF1
    wire  [6:0]  out_base = 7'd127 - f_size(osz_o);        // base off by one
`else
    wire  [6:0]  out_base = 7'd0 - f_size(osz_o);          // 128 - So
`endif
    wire  [6:0]  out_off = out_base | f_idx(out_wr[o], osz_o);

    always_comb begin
        en = 1'b0; we = 1'b0; addr = 9'h0; wdata = 32'h0; bmask = 32'h0;
        pop_tk = 1'b0; push_tk = 1'b0;
        if (host_ph && (h_en_i || ph_i[1])) begin
            if (h_en_i) begin
                en = 1'b1; we = h_we_i; addr = h_addr_i;
                if (h_we_i) begin wdata = h_wdata_i; bmask = 32'hFFFF_FFFF; end
            end
        end else begin
            case (dm_op_i)
                OP_READ: begin
                    en = 1'b1;
`ifdef MUT_REGION
                    addr = {dm_off_i[6:5] | o, dm_off_i};  // region from offset bits
`else
                    addr = {o, dm_off_i};
`endif
                end
                OP_WRITE: begin
`ifdef MUT_WRITE_MASK0
                    if (1'b1) begin                        // mask 0 still accesses
`else
                    if (dm_hmask_i != 2'b00) begin
`endif
                        en = 1'b1; we = 1'b1; addr = {o, dm_off_i}; wdata = dm_wdata_i;
`ifdef MUT_HMASK_SWAP
                        bmask = {{16{dm_hmask_i[0]}}, {16{dm_hmask_i[1]}}};
`else
                        bmask = {{16{dm_hmask_i[1]}}, {16{dm_hmask_i[0]}}};
`endif
                    end
                end
                OP_POP: begin
`ifdef MUT_POP_EMPTY
                    if (1'b1) begin                        // pop when empty
`else
                    if (!qe_o[o]) begin
`endif
                        en = 1'b1; addr = {o, in_off}; pop_tk = 1'b1;
                    end
                end
                OP_PUSH: begin
`ifdef MUT_PUSH_FULL
                    if (1'b1) begin                        // push when full
`else
                    if (!qf_o[o]) begin
`endif
                        en = 1'b1; we = 1'b1; addr = {o, out_off}; wdata = dm_wdata_i;
`ifdef MUT_PUSH_MASK
                        bmask = {{16{dm_hmask_i[1]}}, {16{dm_hmask_i[0]}}};
`else
                        bmask = 32'hFFFF_FFFF;
`endif
                        push_tk = 1'b1;
                    end
                end
                default: ;
            endcase
`ifdef MUT_IDLE_READ
            if (!en) begin en = 1'b1; addr = {o, dm_off_i}; end // dummy read: rdata not held
`endif
        end
    end

    // ------------------------------------------------------ register interface
    function automatic [6:0] a_qcfg(input integer l); a_qcfg = 7'h15 + 7'(8 * l); endfunction
    function automatic [6:0] a_qin (input integer l); a_qin  = 7'h16 + 7'(8 * l); endfunction
    function automatic [6:0] a_qout(input integer l); a_qout = 7'h17 + 7'(8 * l); endfunction

`ifdef MUT_HADDR_ALIAS
    wire [6:0] ha = {1'b0, hreg_addr_i[5:0]};   // address bit 6 not decoded
`else
    wire [6:0] ha = hreg_addr_i;
`endif

`ifdef MUT_HREG_NOPH
    wire hwr = hreg_we_i;                       // register write outside ph = 1
`else
    wire hwr = hreg_we_i && ph_i[1];
`endif

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int l = 0; l < 4; l++) begin
`ifdef MUT_RST_SZ
                isz[l] <= 2'd1;
`else
                isz[l] <= 2'd0;
`endif
                osz[l]    <= 2'd0;
                in_wr[l]  <= 7'd0;
                in_rd[l]  <= 7'd0;
                out_wr[l] <= 7'd0;
                out_rd[l] <= 7'd0;
            end
        end else begin
            if (pop_tk)  in_rd[o]  <= f_next(in_rd[o], isz_o);
`ifndef MUT_PUSH_NOPTR
            if (push_tk) out_wr[o] <= f_next(out_wr[o], osz_o);
`endif
            if (hwr) begin
                for (int l = 0; l < 4; l++) begin
                    if (ha == a_qcfg(l)) begin
`ifdef MUT_QCFG_SWAP
                        isz[l] <= hreg_wdata_i[3:2];
                        osz[l] <= hreg_wdata_i[1:0];
`else
                        isz[l] <= hreg_wdata_i[1:0];
                        osz[l] <= hreg_wdata_i[3:2];
`endif
                    end
                    if (ha == a_qin(l)) begin
                        in_wr[l] <= hreg_wdata_i[6:0];
`ifdef MUT_RUN_IGNORE
                        in_rd[l] <= hreg_wdata_i[14:8];   // written while running
`else
                        if (!run_i[l]) in_rd[l] <= hreg_wdata_i[14:8];
`endif
                    end
                    if (ha == a_qout(l)) begin
                        out_rd[l] <= hreg_wdata_i[6:0];
`ifdef MUT_RUN_INV
                        if (run_i[l]) out_wr[l] <= hreg_wdata_i[14:8]; // run test inverted
`else
                        if (!run_i[l]) out_wr[l] <= hreg_wdata_i[14:8];
`endif
                    end
                end
            end
        end
    end

    always_comb begin
        hreg_rdata_o = 32'h0;
`ifdef MUT_IRQSTAT
        if (ha == 7'h04) hreg_rdata_o = {28'h0, qne_o};  // wrong bits
`else
        if (ha == 7'h04) hreg_rdata_o = {24'h0, qne_o, 4'h0};
`endif
        for (int l = 0; l < 4; l++) begin
            if (ha == a_qcfg(l)) hreg_rdata_o = {28'h0, osz[l], isz[l]};
            if (ha == a_qin(l))
`ifdef MUT_QIN_FLAGS
                hreg_rdata_o = {14'h0, qe_o[l], fi_full[l], 1'b0, in_rd[l], 1'b0, in_wr[l]};
`else
                hreg_rdata_o = {14'h0, fi_full[l], qe_o[l], 1'b0, in_rd[l], 1'b0, in_wr[l]};
`endif
            if (ha == a_qout(l))
                hreg_rdata_o = {14'h0, qf_o[l], fo_empty[l], 1'b0, out_wr[l], 1'b0, out_rd[l]};
        end
    end

    always_comb begin
        for (int l = 0; l < 4; l++) begin
            obs_qcfg_o[4*l +: 4]   = {osz[l], isz[l]};
            obs_in_wr_o[7*l +: 7]  = in_wr[l];
            obs_in_rd_o[7*l +: 7]  = in_rd[l];
            obs_out_wr_o[7*l +: 7] = out_wr[l];
            obs_out_rd_o[7*l +: 7] = out_rd[l];
        end
    end

    // ------------------------------------------------------------------ macro
    logic [31:0] dout;
    assign rdata_o = dout;

    rosetta_sram512x32 u_macro (
        .clk   (clk),
        .en    (en),
        .we    (we),
        .addr  (addr),
        .wdata (wdata),
        .bmask (bmask),
        .rdata (dout)
    );
endmodule
