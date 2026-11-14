`default_nettype none
// ===========================================================================
//  Formal properties for rosetta_pins (lane output registers).
//
//  Written from docs/blocks/pins.md, docs/blocks/lane.md 1-6 and 12.2,
//  field.md 7.1 (when pin_we_i is raised) and docs/decisions.md ONLY. The
//  RTL was not read. Every property is about the block's ports.
//
//  Contents
//   * Executable model of section 2 (val[3:0], oe[3:0]). out_val_o and
//     out_oe_o are compared with it in EVERY cycle, reset cycles included.
//     Because the model's val/oe are flops and out_oe_o = oe & run_i, this
//     also proves required property 6 (no combinational path from any input
//     but run_i to an output): the outputs equal a function of the previous
//     cycle for every value of the current inputs.
//   * Required properties 1-6 of section 5 restated on the ports without
//     the model (a_p1_* .. a_p6_*), per output j.
//   * Section 4: a halted or never-started lane enables nothing, and an
//     enable rises only through the listed inputs (the set E of top.md T9).
//   * Ownership (section 3): out2 only by PIN, out1 by data requests only
//     with w1, out3 by clock requests, out0 by data requests; PIN wins.
//   * `align` vs immediate: `align` is resolved in rosetta_field (field.md
//     7.1: it only chooses the cycle in which pin_we_i is raised). At this
//     block a PIN acts exactly at the end of its pin_we_i cycle: visible in
//     the next cycle (a_p4_*), never in its own cycle (a_p6_*), never later.
//   * Covers (required property 7).
//
//  The block has no environment assumptions besides reset in cycle 0
//  (section 5 lists none; PIN-versus-request collisions are defined by row
//  order and are proven, not excluded).
//
//  Assertions inside generate loops carry their name as `// @name`.
// ===========================================================================

module pins_props #(
    parameter STANDALONE = 1
) (
    input wire       clk,
    input wire       rst_n,
    input wire       run_i,
    input wire       d_we_i,
    input wire [4:0] d_pay_i,
    input wire       c_we_i,
    input wire [1:0] c_pay_i,
    input wire       pin_we_i,
    input wire [3:0] pin_mask_i,
    input wire [3:0] pin_val_i,
    input wire [3:0] pin_oe_i,
    input wire [3:0] out_val_o,
    input wire [3:0] out_oe_o
);
    // payloads {v0, e0, w1, v1, e1} and {v3, e3}
    wire v0 = d_pay_i[4], e0 = d_pay_i[3], w1 = d_pay_i[2], v1 = d_pay_i[1], e1 = d_pay_i[0];
    wire v3 = c_pay_i[1], e3 = c_pay_i[0];

    // ---------------------------------------------------------------------
    // Environment: reset in the first cycle only.
    // ---------------------------------------------------------------------
    generate
        if (STANDALONE) begin : g_env
            always @* begin
                if ($initstate) begin
                    env_rst: assume (!rst_n);
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------------------
    // Per-output request decode (rows 2-5 of section 2)
    // ---------------------------------------------------------------------
    wire [3:0] nm_pin = pin_we_i ? pin_mask_i : 4'h0;                // row 2
    wire [3:0] nm_dat = {1'b0, 1'b0, d_we_i && w1, d_we_i};           // rows 3, 4
    wire [3:0] nm_clk = {c_we_i, 3'b000};                             // row 5
    wire [3:0] rq_val = {v3, 1'b0, v1, v0};
    wire [3:0] rq_oe  = {e3, 1'b0, e1, e0};
    // enabling inputs per output (section 4, set E): only these can raise oe
    wire [3:0] en_in  = (nm_pin & pin_oe_i) | (nm_dat & {2'b00, e1, e0}) | (nm_clk & {e3, 3'b000});

    // ---------------------------------------------------------------------
    // Executable model (section 2)
    // ---------------------------------------------------------------------
    logic [3:0] m_val, m_oe;
    logic [3:0] m_val_n, m_oe_n;
    always @* begin
        for (int j = 0; j < 4; j++) begin
            if (!run_i)             begin m_val_n[j] = m_val[j];     m_oe_n[j] = 1'b0;        end
            else if (nm_pin[j])     begin m_val_n[j] = pin_val_i[j]; m_oe_n[j] = pin_oe_i[j]; end
            else if (nm_dat[j])     begin m_val_n[j] = rq_val[j];    m_oe_n[j] = rq_oe[j];    end
            else if (nm_clk[j])     begin m_val_n[j] = rq_val[j];    m_oe_n[j] = rq_oe[j];    end
            else                    begin m_val_n[j] = m_val[j];     m_oe_n[j] = m_oe[j];     end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin m_val <= 4'h0; m_oe <= 4'h0; end
        else        begin m_val <= m_val_n; m_oe <= m_oe_n; end
    end

`ifndef PINS_NO_MODEL_CHECK
    always @* begin
        a_m_val: assert (out_val_o == m_val);
        a_m_oe:  assert (out_oe_o  == (m_oe & {4{run_i}}));
    end
`endif

    // ---------------------------------------------------------------------
    // Port-level statements (no model)
    // ---------------------------------------------------------------------
    // Reset: everything released and 0 (asynchronous)
    always @* begin
        if (!rst_n) begin
            a_rst_val: assert (out_val_o == 4'h0);
            a_rst_oe:  assert (out_oe_o  == 4'h0);
        end
        // 2. halted: nothing enabled, in the same cycle (the gate)
        if (!run_i) begin
            a_p2_gate: assert (out_oe_o == 4'h0);
        end
    end

    // 3. from reset until an enabling input (with run_i), nothing enabled;
    //    until any write (with run_i), all values 0
    logic s_en, s_wr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin s_en <= 1'b0; s_wr <= 1'b0; end
        else begin
            if (run_i && en_in != 4'h0) s_en <= 1'b1;
            if (run_i && (pin_we_i || d_we_i || c_we_i)) s_wr <= 1'b1;
        end
    end
    always @* begin
        if (!s_en) begin
            a_p3_noen:  assert (out_oe_o == 4'h0);
        end
        if (!s_wr) begin
            a_p3_noval: assert (out_val_o == 4'h0);
        end
    end

    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : g_out
            always @(posedge clk) begin
                if (!$initstate && rst_n && $past(rst_n)) begin
                    // 2. after a halted cycle the enable is 0 (visible as
                    //    soon as run_i returns); the value is kept
                    if ($past(!run_i)) begin
                        assert (!out_oe_o[g]);  // @a_p2_after_halt
                        assert (out_val_o[g] == $past(out_val_o[g]));  // @a_p2_val_kept
                    end
                    // 1./4. row 2: PIN wins, regardless of other requests
                    if ($past(run_i && nm_pin[g])) begin
                        assert (out_val_o[g] == $past(pin_val_i[g]));  // @a_p4_pin_val
                        if (run_i) begin
                            assert (out_oe_o[g] == $past(pin_oe_i[g]));  // @a_p4_pin_oe
                        end
                    end
                    // 1. rows 3-5: the request's value and enable
                    if ($past(run_i && !nm_pin[g] && (nm_dat[g] || nm_clk[g]))) begin
                        assert (out_val_o[g] == $past(rq_val[g]));  // @a_p1_req_val
                        if (run_i) begin
                            assert (out_oe_o[g] == $past(rq_oe[g]));  // @a_p1_req_oe
                        end
                    end
                    // 5./row 6: an output named by no request keeps both
                    if ($past(run_i && !nm_pin[g] && !nm_dat[g] && !nm_clk[g])) begin
                        assert (out_val_o[g] == $past(out_val_o[g]));  // @a_p5_keep_val
                        if (run_i) begin
                            assert (out_oe_o[g] == $past(out_oe_o[g]));  // @a_p5_keep_oe
                        end
                    end
                    // section 4: an enable rises only through an enabling
                    // input for this output in the previous cycle
                    if (out_oe_o[g] && !$past(out_oe_o[g])) begin
                        assert ($past(run_i && en_in[g]));  // @a_p4_only_listed
                    end
                    // a value changes only through a request naming this
                    // output (ownership; out2 only by PIN)
                    if (out_val_o[g] != $past(out_val_o[g])) begin
                        assert ($past(run_i && (nm_pin[g] || nm_dat[g] || nm_clk[g])));  // @a_own_val
                    end
                end
                // 6./align: in a PIN's own cycle the outputs still show the
                // old state (no bypass from pin_* to the outputs)
                if (!$initstate && rst_n && $past(rst_n) && pin_we_i && pin_mask_i[g]) begin
                    assert (out_val_o[g] == $past(run_i ? (nm_pin[g] ? pin_val_i[g] :
                                                           (nm_dat[g] || nm_clk[g]) ? rq_val[g] :
                                                           out_val_o[g]) : out_val_o[g]));  // @a_p6_pin_noearly
                end
            end
        end
    endgenerate

    // out2 is written by PIN only (section 3)
    always @(posedge clk) begin
        if (!$initstate && rst_n && $past(rst_n && run_i && !(pin_we_i && pin_mask_i[2]))) begin
            a_own_out2_val: assert (out_val_o[2] == $past(out_val_o[2]));
            if (run_i) begin
                a_own_out2_oe: assert (out_oe_o[2] == $past(out_oe_o[2]));
            end
        end
    end

    // ---------------------------------------------------------------------
    // Covers (required property 7)
    // ---------------------------------------------------------------------
    generate
        if (STANDALONE) begin : g_cover
            always @(posedge clk) begin
                if (!$initstate && rst_n && $past(rst_n)) begin
                    // PIN and clock request in one cycle on out3 (PIN wins,
                    // with values that differ)
                    c_pin_clk:    cover ($past(run_i && pin_we_i && pin_mask_i[3] && c_we_i &&
                                               pin_val_i[3] != v3 && pin_oe_i[3] != e3) && run_i);
                    // PIN and data request in one cycle on out0 and out1
                    c_pin_dat:    cover ($past(run_i && pin_we_i && pin_mask_i[1:0] == 2'b11 && d_we_i && w1 &&
                                               pin_val_i[0] != v0 && pin_val_i[1] != v1) && run_i);
                    // data request with and without w1
                    c_dat_w1:     cover ($past(run_i && d_we_i && w1 && e1 && !pin_we_i) && out_oe_o[1]);
                    c_dat_now1:   cover ($past(run_i && d_we_i && !w1 && e1 && !pin_we_i && !out_oe_o[1]) &&
                                         run_i && !out_oe_o[1] && out_oe_o[0]);
                    // release keeps the value, clears the enable
                    c_release:    cover ($past(run_i && d_we_i && !e0 && v0 && out_oe_o[0] && !pin_we_i) && run_i);
                    // halt with outputs enabled; restart released
                    c_halt_en:    cover ($past(run_i && out_oe_o == 4'hF) && !run_i);
                    c_restart:    cover ($past(!run_i, 2) && $past(!run_i) && run_i && out_val_o == 4'hF);
                    // a PIN with an empty mask (WAIT timeout) changes nothing
                    c_pin_empty:  cover ($past(run_i && pin_we_i && pin_mask_i == 4'h0 && pin_oe_i == 4'hF) &&
                                         run_i && out_oe_o == 4'h0);
                    // a PIN with enable 0 changes a value without enabling
                    c_pin_val_only: cover ($past(run_i && pin_we_i && pin_mask_i[2] && !pin_oe_i[2] && pin_val_i[2]) &&
                                           !s_en && out_val_o[2]);
                    // all four enabled at once
                    c_all_en:     cover (out_oe_o == 4'hF);
                    // inputs active while halted (ignored)
                    c_halt_inputs: cover ($past(!run_i && pin_we_i && pin_mask_i == 4'hF && d_we_i && c_we_i));
                end
            end
            for (g = 0; g < 4; g = g + 1) begin : g_cov_out
                always @(posedge clk) begin
                    if (!$initstate && rst_n && $past(rst_n)) begin
                        // row 1: halt clears an enabled output
                        cover ($past(!run_i && m_oe[g]) && run_i && !out_oe_o[g]);  // c_row1
                        // row 2: PIN changes value and enable
                        cover ($past(run_i && nm_pin[g] && pin_val_i[g] != out_val_o[g] &&
                                     pin_oe_i[g] != out_oe_o[g]) && run_i);  // c_row2
                        // row 6: kept while other outputs change
                        cover ($past(run_i && !nm_pin[g] && !nm_dat[g] && !nm_clk[g] && out_oe_o[g] &&
                                     (nm_pin | nm_dat | nm_clk) != 4'h0) && run_i && out_oe_o[g]);  // c_row6
                    end
                end
            end
            always @(posedge clk) begin
                if (!$initstate && rst_n && $past(rst_n)) begin
                    // rows 3, 4, 5 change value and enable
                    c_row3: cover ($past(run_i && d_we_i && !pin_we_i && v0 != out_val_o[0] && e0 != out_oe_o[0]) && run_i);
                    c_row4: cover ($past(run_i && d_we_i && w1 && !pin_we_i && v1 != out_val_o[1] && e1 != out_oe_o[1]) && run_i);
                    c_row5: cover ($past(run_i && c_we_i && !pin_we_i && v3 != out_val_o[3] && e3 != out_oe_o[3]) && run_i);
                end
            end
        end
    endgenerate
endmodule
