`default_nettype none
// Full behavioural model of the SRAM macro (mem.md 4.1, written from the
// spec): arbitrary initial contents and A_DOUT (no reset, no init), per-bit
// mask, write-through when A_WEN = A_REN = 1 (never used by the wrappers).
// For BMC and cover only (16,416 state bits per macro).
module RM_IHPSG13_1P_512x32_c2_bm_bist (
    input  wire        A_CLK,
    input  wire        A_MEN,
    input  wire        A_WEN,
    input  wire        A_REN,
    input  wire [8:0]  A_ADDR,
    input  wire [31:0] A_DIN,
    input  wire        A_DLY,
    output wire [31:0] A_DOUT,
    input  wire [31:0] A_BM,
    input  wire        A_BIST_CLK,
    input  wire        A_BIST_EN,
    input  wire        A_BIST_MEN,
    input  wire        A_BIST_WEN,
    input  wire        A_BIST_REN,
    input  wire [8:0]  A_BIST_ADDR,
    input  wire [31:0] A_BIST_DIN,
    input  wire [31:0] A_BIST_BM
);
    reg [31:0] mem [0:511];
    reg [31:0] dout;
    always @(posedge A_CLK)
        if (A_MEN && A_WEN) begin
            mem[A_ADDR] <= (mem[A_ADDR] & ~A_BM) | (A_DIN & A_BM);
            if (A_REN) dout <= (mem[A_ADDR] & ~A_BM) | (A_DIN & A_BM);
        end else if (A_MEN && A_REN)
            dout <= mem[A_ADDR];
    assign A_DOUT = dout;
endmodule
