`default_nettype none
// ===========================================================================
//  PROPERTY WRITER'S PRIVATE REFERENCE MODEL -- THIS IS NOT THE DESIGN.
//
//  Throwaway implementation of docs/blocks/channels.md written by the formal
//  property writer, only to show that formal/chan/*.sv can PASS and that the
//  properties are non-vacuous (the planted bugs below make tasks FAIL).
//  The real block is src/rosetta_chan.sv, written independently.
//  Do not integrate, synthesise or copy this file.
//
//  A host clear and a put in one cycle leave valid = 0 and write the data
//  (section 2 rules applied flop by flop); the proofs exclude that cycle
//  (coordinator ruling 1), so this choice is never checked.
//
//  Planted bugs (mutation testing), enabled one at a time with a define:
//    MUT_PUT_OVERWRITE      a put to a valid channel overwrites its data
//    MUT_PUT_NO_DATA        a successful put sets valid but keeps old data
//    MUT_GET_CLEARS_DATA    a successful get also zeroes the data
//    MUT_GET_EMPTY_SETS     a get from an empty channel sets valid (toggle)
//    MUT_GET_BYPASS         put + get in one cycle on an empty channel: the
//                           get takes the value being put (valid stays 0)
//    MUT_IDX_DECODE         channel index bits swapped (1 <-> 2) on writes
//    MUT_CLEAR_LOSES        a put beats a host clear of the same channel
//                           (expected UNDETECTABLE: ruling 1 excludes it)
//    MUT_CLEAR_ANY_ADDR     host clear acts on a write to any address
//    MUT_CLEAR_DATA         host clear also zeroes the data
//    MUT_TS_STALL_PUT       ts holds in a cycle with ch_put_i
//    MUT_TS_SKIP_HWE        ts advances by 2 in a cycle with hreg_we_i
//    MUT_TS_SATURATE        ts saturates at 0xFFFFFFFF instead of wrapping
//    MUT_TS_NO_RESET        ts flop has no reset
//    MUT_VALID_NO_RESET     valid flops have no reset
//    MUT_DATA_NO_RESET      data flops have no reset
//    MUT_RD_ADDR            TIME is read at 0x02 instead of 0x03
//    MUT_RD_ALIAS           read decode ignores hreg_addr_i[6:4], so other
//                           addresses (e.g. 0x46, 0x13) return chan data
//    MUT_RD_REG             hreg_rdata_o registered (one cycle late)
// ===========================================================================
module rosetta_chan (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          ch_put_i,
    input  wire          ch_get_i,
    input  wire  [1:0]   ch_idx_i,
    input  wire  [31:0]  ch_wdata_i,
    output logic [3:0]   ch_valid_o,
    output logic [127:0] ch_data_o,
    output logic [31:0]  ts_o,
    input  wire          hreg_we_i,
    input  wire  [6:0]   hreg_addr_i,
    input  wire  [31:0]  hreg_wdata_i,
    output logic [31:0]  hreg_rdata_o
);
    localparam [6:0] A_TIME   = 7'h03;
    localparam [6:0] A_CHSTAT = 7'h06;

`ifdef MUT_IDX_DECODE
    wire [1:0] idx = {ch_idx_i[0], ch_idx_i[1]};
`else
    wire [1:0] idx = ch_idx_i;
`endif

`ifdef MUT_CLEAR_ANY_ADDR
    wire       clr   = hreg_we_i;
`else
    wire       clr   = hreg_we_i && (hreg_addr_i == A_CHSTAT);
`endif
    wire [3:0] clr_m = clr ? hreg_wdata_i[3:0] : 4'h0;
    wire       v_idx = ch_valid_o[idx];
    wire       put_ok = ch_put_i && !v_idx;
    wire       get_ok = ch_get_i &&  v_idx;

    // ---- valid ------------------------------------------------------------
    logic [3:0] valid_n;
    always_comb begin
        valid_n = ch_valid_o;
`ifdef MUT_GET_BYPASS
        if (put_ok && !ch_get_i) valid_n[idx] = 1'b1;
`else
        if (put_ok)             valid_n[idx] = 1'b1;
`endif
        if (get_ok)             valid_n[idx] = 1'b0;
`ifdef MUT_GET_EMPTY_SETS
        if (ch_get_i && !v_idx && !ch_put_i) valid_n[idx] = 1'b1;
`endif
`ifdef MUT_CLEAR_LOSES
        valid_n = valid_n & ~(clr_m & ~((put_ok ? 4'b0001 : 4'b0000) << idx));
`else
        valid_n = valid_n & ~clr_m;
`endif
    end

`ifdef MUT_VALID_NO_RESET
    always_ff @(posedge clk) ch_valid_o <= valid_n;
`else
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) ch_valid_o <= 4'h0;
        else        ch_valid_o <= valid_n;
    end
`endif

    // ---- data -------------------------------------------------------------
    logic [127:0] data_n;
    always_comb begin
        data_n = ch_data_o;
`ifdef MUT_PUT_OVERWRITE
        if (ch_put_i)
`else
        if (put_ok)
`endif
`ifdef MUT_PUT_NO_DATA
            data_n = data_n;
`else
            data_n[32*idx +: 32] = ch_wdata_i;
`endif
`ifdef MUT_GET_CLEARS_DATA
        if (get_ok) data_n[32*idx +: 32] = 32'h0;
`endif
`ifdef MUT_CLEAR_DATA
        if (clr_m[0]) data_n[31:0]   = 32'h0;
        if (clr_m[1]) data_n[63:32]  = 32'h0;
        if (clr_m[2]) data_n[95:64]  = 32'h0;
        if (clr_m[3]) data_n[127:96] = 32'h0;
`endif
    end

`ifdef MUT_DATA_NO_RESET
    always_ff @(posedge clk) ch_data_o <= data_n;
`else
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) ch_data_o <= 128'h0;
        else        ch_data_o <= data_n;
    end
`endif

    // ---- timestamp --------------------------------------------------------
    logic [31:0] ts_n;
    always_comb begin
`ifdef MUT_TS_STALL_PUT
        ts_n = ch_put_i ? ts_o : ts_o + 32'd1;
`elsif MUT_TS_SKIP_HWE
        ts_n = ts_o + (hreg_we_i ? 32'd2 : 32'd1);
`elsif MUT_TS_SATURATE
        ts_n = (ts_o == 32'hFFFF_FFFF) ? ts_o : ts_o + 32'd1;
`else
        ts_n = ts_o + 32'd1;
`endif
    end

`ifdef MUT_TS_NO_RESET
    always_ff @(posedge clk) ts_o <= ts_n;
`else
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) ts_o <= 32'h0;
        else        ts_o <= ts_n;
    end
`endif

    // ---- register reads (combinational) -----------------------------------
    logic [31:0] rd;
    always_comb begin
        rd = 32'h0;
`ifdef MUT_RD_ALIAS
        case (hreg_addr_i[3:0])
            4'h3: rd = ts_o;
            4'h6: rd = {28'h0, ch_valid_o};
            4'h8: rd = ch_data_o[31:0];
            4'h9: rd = ch_data_o[63:32];
            4'hA: rd = ch_data_o[95:64];
            4'hB: rd = ch_data_o[127:96];
            default: rd = 32'h0;
        endcase
`else
        case (hreg_addr_i)
`ifdef MUT_RD_ADDR
            7'h02: rd = ts_o;
`else
            A_TIME: rd = ts_o;
`endif
            A_CHSTAT: rd = {28'h0, ch_valid_o};
            7'h08: rd = ch_data_o[31:0];
            7'h09: rd = ch_data_o[63:32];
            7'h0A: rd = ch_data_o[95:64];
            7'h0B: rd = ch_data_o[127:96];
            default: rd = 32'h0;
        endcase
`endif
    end

`ifdef MUT_RD_REG
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) hreg_rdata_o <= 32'h0;
        else        hreg_rdata_o <= rd;
    end
`else
    assign hreg_rdata_o = rd;
`endif

endmodule
