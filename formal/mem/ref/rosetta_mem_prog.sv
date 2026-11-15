`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL of rosetta_mem_prog. NOT the
//  design (src/rosetta_mem_prog.sv was not read). Written from
//  docs/blocks/mem.md section 2 only, to show the properties in
//  formal/mem/mem_prog_props.sv pass on a spec-conforming implementation and
//  fail on plausible bugs (planted under MUT_<NAME> defines, one at a time; see
//  run_mutants.sh).
//
//  Combinational drive of the macro (mem.md 2.2), through the thin macro
//  wrapper rosetta_sram512x32 (en / we / addr / wdata / bmask), instance
//  u_macro. No flops, no reset.
// ===========================================================================
module rosetta_mem_prog (
    input  wire         clk,
    input  wire  [4:0]  ph_i,
    input  wire  [8:0]  fetch_addr_i,
    input  wire         h_en_i,
    input  wire         h_we_i,
    input  wire  [8:0]  h_addr_i,
    input  wire  [31:0] h_wdata_i,
    output logic [31:0] rdata_o
);
    logic        en, we;
    logic [8:0]  addr;
    logic [31:0] wdata, bmask;

`ifdef MUT_HOST_PH3
    // host served at ph = 3 instead of ph = 4 (wrong phase index)
    wire host_ph = ph_i[3];
`else
    wire host_ph = ph_i[4];
`endif

    always_comb begin
        en = 1'b0; we = 1'b0; addr = 9'h0; wdata = 32'h0; bmask = 32'h0;
`ifdef MUT_WEN_ANYPH
        // host write honoured in any phase (h_en_i outside ph = 4 not ignored)
        if (h_en_i && h_we_i) begin
            en = 1'b1; we = 1'b1; addr = h_addr_i; wdata = h_wdata_i; bmask = 32'hFFFF_FFFF;
        end else
`endif
`ifdef MUT_HEN_ANYPH
        // any host access honoured in any phase
        if (h_en_i) begin
            en = 1'b1; we = h_we_i; addr = h_addr_i;
            if (h_we_i) begin wdata = h_wdata_i; bmask = 32'hFFFF_FFFF; end
        end else
`endif
        if (!host_ph) begin
            en   = 1'b1;
`ifdef MUT_ADDR_BIT
            addr = {1'b0, fetch_addr_i[7:0]};       // fetch address bit 8 lost
`else
            addr = fetch_addr_i;
`endif
`ifdef MUT_ZERO_DIN
            wdata = h_wdata_i;                       // A_DIN not 0 on a read
`endif
        end else if (h_en_i) begin
            en   = 1'b1;
            we   = h_we_i;
`ifdef MUT_HREAD_FETCH
            addr = h_we_i ? h_addr_i : fetch_addr_i; // host read uses fetch address
`else
            addr = h_addr_i;
`endif
            if (h_we_i) begin
`ifdef MUT_WDATA
                wdata = h_wdata_i ^ 32'h0000_0100;   // one data bit inverted
`else
                wdata = h_wdata_i;
`endif
`ifdef MUT_BM_HALF
                bmask = 32'h0000_FFFF;               // host write only low half
`else
                bmask = 32'hFFFF_FFFF;
`endif
            end
        end else begin
`ifdef MUT_IDLE_READ
            en   = 1'b1;                             // idle ph = 4 still reads
            addr = fetch_addr_i;
`endif
        end
    end

    logic [31:0] dout;

`ifdef MUT_RDATA_REG
    // extra register on the read data (one cycle of extra latency)
    logic [31:0] dout_q;
    always_ff @(posedge clk) dout_q <= dout;
    assign rdata_o = dout_q;
`else
    assign rdata_o = dout;
`endif

`ifndef ROSETTA_SRAM_BEHAV
 `define MEMP_DIRECT_PINS
`endif
`ifdef MUT_DLY0
 `ifdef MEMP_DIRECT_PINS
  `define MEMP_DIRECT
 `endif
`endif
`ifdef MUT_WRTHRU
 `ifdef MEMP_DIRECT_PINS
  `define MEMP_DIRECT
 `endif
`endif

`ifdef MEMP_DIRECT
    // Pin-level mutants: the macro instantiated directly (no thin wrapper).
    // Only meaningful with a model of the macro (prove task); under
    // ROSETTA_SRAM_BEHAV the mutant is the clean design.
    RM_IHPSG13_1P_512x32_c2_bm_bist u_macro (
        .A_CLK       (clk),
        .A_MEN       (en),
        .A_WEN       (en & we),
 `ifdef MUT_WRTHRU
        .A_REN       (en),                           // write-through on writes
 `else
        .A_REN       (en & ~we),
 `endif
        .A_ADDR      (addr),
        .A_DIN       (wdata),
 `ifdef MUT_DLY0
        .A_DLY       (1'b0),
 `else
        .A_DLY       (1'b1),
 `endif
        .A_DOUT      (dout),
        .A_BM        (bmask),
        .A_BIST_CLK  (1'b0),
        .A_BIST_EN   (1'b0),
        .A_BIST_MEN  (1'b0),
        .A_BIST_WEN  (1'b0),
        .A_BIST_REN  (1'b0),
        .A_BIST_ADDR (9'h0),
        .A_BIST_DIN  (32'h0),
        .A_BIST_BM   (32'h0)
    );
 `undef MEMP_DIRECT
`else
    rosetta_sram512x32 u_macro (
        .clk   (clk),
        .en    (en),
        .we    (we),
        .addr  (addr),
        .wdata (wdata),
        .bmask (bmask),
        .rdata (dout)
    );
`endif
`ifdef MEMP_DIRECT_PINS
 `undef MEMP_DIRECT_PINS
`endif
endmodule
