/*
 * Synthesis/lint black box for the IHP SG13CMOS5L SRAM hard macro
 * (instanced only in src/rosetta_sram512x32.sv, as u_macro).
 * Simulation defines SIM and uses the PDK behavioural model instead
 * ($PDK_ROOT/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/verilog/, with -DFUNCTIONAL);
 * a build with ROSETTA_SRAM_BEHAV does not instance the macro at all.
 */
`ifndef SIM
`ifndef ROSETTA_SRAM_BEHAV
/* verilator lint_off DECLFILENAME */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNDRIVEN */
(* blackbox *)
module RM_IHPSG13_1P_512x32_c2_bm_bist (
    input  wire        A_CLK,
    input  wire        A_MEN,
    input  wire        A_WEN,
    input  wire        A_REN,
    input  wire  [8:0] A_ADDR,
    input  wire [31:0] A_DIN,
    input  wire        A_DLY,
    output wire [31:0] A_DOUT,
    input  wire [31:0] A_BM,
    input  wire        A_BIST_CLK,
    input  wire        A_BIST_EN,
    input  wire        A_BIST_MEN,
    input  wire        A_BIST_WEN,
    input  wire        A_BIST_REN,
    input  wire  [8:0] A_BIST_ADDR,
    input  wire [31:0] A_BIST_DIN,
    input  wire [31:0] A_BIST_BM
);
endmodule
/* verilator lint_on UNDRIVEN */
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on DECLFILENAME */
`endif
`endif
