// ===========================================================================
//  Shared helper functions of the field spec (docs/blocks/field.md 1, 2.1,
//  5.4), included by field_props.sv inside its modules. Written from the
//  spec only.
// ===========================================================================

// mask(n): the low n bits set, n = 0..32
function automatic [31:0] f_mask(input [5:0] n);
    f_mask = n[5] ? 32'hFFFF_FFFF : ((32'h1 << n[4:0]) - 32'h1);
endfunction

// 16-bit mask for the seek window, n' = 0..16
function automatic [15:0] f_mask16(input [4:0] n);
    f_mask16 = n[4] ? 16'hFFFF : ((16'h1 << n[3:0]) - 16'h1);
endfunction

// n = ctl[25:21] + 1 (1..32)
function automatic [5:0] f_n(input [27:0] c);
    f_n = {1'b0, c[25:21]} + 6'd1;
endfunction

// n' = min(n, 16)
function automatic [4:0] f_np(input [27:0] c);
    f_np = c[25] ? 5'd16 : ({1'b0, c[24:21]} + 5'd1);
endfunction

// INS(S, b) (5.4)
function automatic [31:0] f_ins(input [31:0] s, input b, input [5:0] n, input lsb);
    if (!lsb) f_ins = {s[30:0], b} & f_mask(n);
    else      f_ins = ((s >> 1) & f_mask(n - 6'd1)) | ({31'h0, b} << (n - 6'd1));
endfunction

// WIN(W, b) (5.4): INS on 16 bits with n'
function automatic [15:0] f_win(input [15:0] w, input b, input [4:0] np, input lsb);
    if (!lsb) f_win = {w[14:0], b} & f_mask16(np);
    else      f_win = ((w >> 1) & f_mask16(np - 5'd1)) | ({15'h0, b} << (np - 5'd1));
endfunction

// tagged(v, tag) (lane.md 1)
function automatic [31:0] f_tagged(input [31:0] v, input [7:0] tag);
    f_tagged = (tag != 8'h0) ? {tag, v[23:0]} : v;
endfunction

// R(d) of lane.md 7.6 as a one-hot nibble (0 = none)
function automatic [3:0] f_rbit(input [3:0] d);
    case (d)
        4'd0: f_rbit = 4'b0001;
        4'd1: f_rbit = 4'b0010;
        4'd2: f_rbit = 4'b0100;
        4'd3: f_rbit = 4'b1000;
        4'd7: f_rbit = 4'b1000;   // MEM marks R3
        default: f_rbit = 4'b0000;
    endcase
endfunction

// D(op): destination of a parked/current op (FIELD, or RAW with direction
// sample), NONE otherwise
function automatic [3:0] f_dst(input [1:0] op, input [27:0] c);
    if (op == 2'd0 || (op == 2'd1 && !c[27])) f_dst = c[13:10];
    else                                     f_dst = 4'd15;
endfunction

// tag of an op: (src != IMM) ? ctl[7:0] : 0
function automatic [7:0] f_tag(input [27:0] c);
    f_tag = (c[17:14] != 4'd12) ? c[7:0] : 8'h0;
endfunction

// seek = (src != IMM) && ctl[8], SAMPLE and EXPECT only (2.1, ruling 32)
function automatic f_seek(input [27:0] c);
    f_seek = (c[17:14] != 4'd12) && c[8] && c[26];
endfunction

// TX = kind in {DRIVE, EXCHANGE}
function automatic f_tx(input [27:0] c);
    f_tx = (c[27:26] == 2'd0) || (c[27:26] == 2'd2);
endfunction

// bt for a FIELD with ctl c under mode
function automatic f_bt(input [1:0] mode, input [27:0] c);
    f_bt = (mode == 2'd0) || (mode == 2'd3 && f_tx(c));
endfunction

// pk_srcdep: FIELD or RAW DRIVE with src in {CRC, SR}
function automatic f_srcdep(input [1:0] op, input [27:0] c);
    f_srcdep = (op == 2'd0 || (op == 2'd1 && c[27])) &&
               (c[17:14] == 4'd6 || c[17:14] == 4'd13);
endfunction

// SR at promotion (4.1); keep = SR value otherwise
function automatic [31:0] f_load(input [1:0] op, input [27:0] c, input [31:0] data,
                                 input [31:0] crcv, input [31:0] sr, input [31:0] keep);
    logic [31:0] v;
    v = (c[17:14] == 4'd6) ? crcv : (c[17:14] == 4'd13) ? sr : data;
    if (op == 2'd0) begin
        if (f_seek(c)) f_load = {v[15:0] & f_mask16(f_np(c)), 16'h0};
        else           f_load = v & f_mask(f_n(c));
    end else if (op == 2'd1 && c[27]) begin
        f_load = v;
    end else begin
        f_load = keep;
    end
endfunction
