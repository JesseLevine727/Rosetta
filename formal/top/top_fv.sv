`default_nettype none
`include "top_wb.svh"
// ===========================================================================
//  Proof tops for the chip top (formal/top/*.sby). Every top instantiates the
//  design module tt_um_jesselevine727_rosetta by its Tiny Tapeout ports
//  only; which components are real and which are shells (gen_top.py), and
//  which SRAM macro model is used, is chosen by the files each .sby reads.
//  The white-box buses wb / wb2 are driven by `connect -set` after flatten
//  (top_wb*.ys).
//
//    top_fv       one copy, every TT input free (rst_n low in the first
//                 cycle). Checker: top_wire_props (define TOP_CHK_WIRE) or
//                 top_chip_props (TOP_CHK_CHIP) or top_core_props
//                 (TOP_CHK_CORE).
//    top_ni_fv    two copies u_a, u_b for T8 (top_ni_props).
// ===========================================================================

module top_fv (
    input wire       clk,
    input wire       rst_n,
    input wire       ena,
    input wire [7:0] ui_in,
    input wire [7:0] uio_in
);
    wire [7:0] uo_out, uio_out, uio_oe;

    tt_um_jesselevine727_rosetta u_dut (
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe),
        .ena     (ena),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    wire [`TOP_WBW-1:0] wb;      // driven by connect -set (top_wb.ys)

`ifdef TOP_CHK_WIRE
    top_wire_props u_props (
        .clk(clk), .rst_n(rst_n), .ena(ena), .ui_in(ui_in), .uio_in(uio_in),
        .uo_out(uo_out), .uio_out(uio_out), .uio_oe(uio_oe), .wb(wb));
`endif
`ifdef TOP_CHK_CHIP
    top_chip_props u_props (
        .clk(clk), .rst_n(rst_n), .ena(ena), .ui_in(ui_in), .uio_in(uio_in),
        .uo_out(uo_out), .uio_out(uio_out), .uio_oe(uio_oe), .wb(wb));
`endif
`ifdef TOP_CHK_T9
    top_t9_props u_props (
        .clk(clk), .rst_n(rst_n), .ena(ena), .ui_in(ui_in), .uio_in(uio_in),
        .uo_out(uo_out), .uio_out(uio_out), .uio_oe(uio_oe), .wb(wb));
`endif
`ifdef TOP_CHK_CORE
    top_core_props u_props (
        .clk(clk), .rst_n(rst_n), .ena(ena), .ui_in(ui_in), .uio_in(uio_in),
        .uo_out(uo_out), .uio_out(uio_out), .uio_oe(uio_oe), .wb(wb));
`endif
endmodule
