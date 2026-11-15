`default_nettype none
// ===========================================================================
//  Proof top for rosetta_mem_prog.
//
//  One copy of the block, every input free (ph_i one-hot by assumption in
//  the checker). chk_rst_n resets the checker only (the block has no reset):
//  low in cycle 0, free afterwards.
//
//  `MEM_PROBE (task prove): the macro is the probe model mem_probe.sv
//  (src/rosetta_sram512x32.sv read WITHOUT ROSETTA_SRAM_BEHAV instantiates
//  the macro by name); the checker decodes the pins from rdata_o.
//  Otherwise (tasks bmc, cover): src/rosetta_sram512x32.sv with
//  ROSETTA_SRAM_BEHAV, the full behavioural memory (arbitrary initial array
//  and read register).
// ===========================================================================

module mem_prog_fv (
    input wire         clk,
    input wire         chk_rst_n,
    input wire [4:0]   ph_i,
    input wire [8:0]   fetch_addr_i,
    input wire         h_en_i,
    input wire         h_we_i,
    input wire [8:0]   h_addr_i,
    input wire [31:0]  h_wdata_i
);
`ifdef MEM_PROBE
    localparam PROBE = 1;
`else
    localparam PROBE = 0;
`endif
    // Marked spec choice (see mem_prog_props.sv header and the .sby):
    //   MEM_LAX_ZERO  the 0 entries of the drive table are don't-care
`ifdef MEM_LAX_ZERO
    localparam STRICT_ZERO = 0;
`else
    localparam STRICT_ZERO = 1;
`endif

    wire [31:0] rdata;

    rosetta_mem_prog u_dut (
        .clk          (clk),
        .ph_i         (ph_i),
        .fetch_addr_i (fetch_addr_i),
        .h_en_i       (h_en_i),
        .h_we_i       (h_we_i),
        .h_addr_i     (h_addr_i),
        .h_wdata_i    (h_wdata_i),
        .rdata_o      (rdata)
    );

    mem_prog_props #(.PROBE(PROBE), .STRICT_ZERO(STRICT_ZERO)) u_props (
        .clk          (clk),
        .chk_rst_n    (chk_rst_n),
        .ph_i         (ph_i),
        .fetch_addr_i (fetch_addr_i),
        .h_en_i       (h_en_i),
        .h_we_i       (h_we_i),
        .h_addr_i     (h_addr_i),
        .h_wdata_i    (h_wdata_i),
        .rdata_o      (rdata)
    );
endmodule
