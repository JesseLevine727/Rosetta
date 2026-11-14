`default_nettype none
// ===========================================================================
//  Proof top for rosetta_chan.
//
//  Three independent copies of the block share clk; every input is free.
//
//   u_env   rst_n low in cycle 0, then free. The sequencer/host guarantees
//           are assumed (chan_props ENV_SEQ = 1): at most one of put/get,
//           put only to an empty channel, get only from a valid one, no
//           CHSTAT write together with a put or get. Normal operation and
//           all the functional covers.
//   u_free  rst_n low in cycle 0, then free. Only the CHSTAT/put-get
//           exclusion (coordinator ruling 1) is assumed (ENV_SEQ = 0): puts
//           to full channels, gets from empty ones, put + get in one cycle,
//           any other host write with a lane op. Proves the block's own
//           "second guard" (section 2) and covers it.
//   u_wrap  proof-only free initial state (FREE_INIT = 1): w_rst_n is free
//           from cycle 0, so the block may start out of reset in any state
//           and the model starts equal to its outputs. Same assertions as
//           the others, from a superset of initial states (stronger, not
//           weaker); exists to make the timestamp wrap 0xFFFFFFFF -> 0
//           reachable for the cover task, and makes prove/bmc check every
//           transition from every output state directly. Remove with
//           `read -define FV_NO_WRAP` if the design ever holds state that is
//           not visible at its outputs (the spec's 164 flops are all
//           visible: data, valid, ts).
// ===========================================================================

module chan_fv (
    input wire         clk,
    input wire         rst_n,
    // u_env inputs
    input wire         e_put,
    input wire         e_get,
    input wire [1:0]   e_idx,
    input wire [31:0]  e_wdata,
    input wire         e_we,
    input wire [6:0]   e_addr,
    input wire [31:0]  e_hwdata,
    // u_free inputs
    input wire         f_put,
    input wire         f_get,
    input wire [1:0]   f_idx,
    input wire [31:0]  f_wdata,
    input wire         f_we,
    input wire [6:0]   f_addr,
    input wire [31:0]  f_hwdata,
    // u_wrap inputs (own reset, free from cycle 0)
    input wire         w_rst_n,
    input wire         w_put,
    input wire         w_get,
    input wire [1:0]   w_idx,
    input wire [31:0]  w_wdata,
    input wire         w_we,
    input wire [6:0]   w_addr,
    input wire [31:0]  w_hwdata
);
    // ---------------------------------------------------------------- u_env
    wire [3:0]   e_valid;
    wire [127:0] e_data;
    wire [31:0]  e_ts, e_rdata;

    rosetta_chan u_env (
        .clk(clk), .rst_n(rst_n),
        .ch_put_i(e_put), .ch_get_i(e_get), .ch_idx_i(e_idx), .ch_wdata_i(e_wdata),
        .ch_valid_o(e_valid), .ch_data_o(e_data), .ts_o(e_ts),
        .hreg_we_i(e_we), .hreg_addr_i(e_addr), .hreg_wdata_i(e_hwdata),
        .hreg_rdata_o(e_rdata)
    );

    chan_props #(.STANDALONE(1), .ENV_SEQ(1), .FREE_INIT(0)) u_env_props (
        .clk(clk), .rst_n(rst_n),
        .ch_put_i(e_put), .ch_get_i(e_get), .ch_idx_i(e_idx), .ch_wdata_i(e_wdata),
        .ch_valid_o(e_valid), .ch_data_o(e_data), .ts_o(e_ts),
        .hreg_we_i(e_we), .hreg_addr_i(e_addr), .hreg_wdata_i(e_hwdata),
        .hreg_rdata_o(e_rdata)
    );

    // --------------------------------------------------------------- u_free
    wire [3:0]   f_valid;
    wire [127:0] f_data;
    wire [31:0]  f_ts, f_rdata;

    rosetta_chan u_free (
        .clk(clk), .rst_n(rst_n),
        .ch_put_i(f_put), .ch_get_i(f_get), .ch_idx_i(f_idx), .ch_wdata_i(f_wdata),
        .ch_valid_o(f_valid), .ch_data_o(f_data), .ts_o(f_ts),
        .hreg_we_i(f_we), .hreg_addr_i(f_addr), .hreg_wdata_i(f_hwdata),
        .hreg_rdata_o(f_rdata)
    );

    chan_props #(.STANDALONE(1), .ENV_SEQ(0), .FREE_INIT(0)) u_free_props (
        .clk(clk), .rst_n(rst_n),
        .ch_put_i(f_put), .ch_get_i(f_get), .ch_idx_i(f_idx), .ch_wdata_i(f_wdata),
        .ch_valid_o(f_valid), .ch_data_o(f_data), .ts_o(f_ts),
        .hreg_we_i(f_we), .hreg_addr_i(f_addr), .hreg_wdata_i(f_hwdata),
        .hreg_rdata_o(f_rdata)
    );

`ifndef FV_NO_WRAP
    // --------------------------------------------------------------- u_wrap
    wire [3:0]   w_valid;
    wire [127:0] w_data;
    wire [31:0]  w_ts, w_rdata;

    rosetta_chan u_wrap (
        .clk(clk), .rst_n(w_rst_n),
        .ch_put_i(w_put), .ch_get_i(w_get), .ch_idx_i(w_idx), .ch_wdata_i(w_wdata),
        .ch_valid_o(w_valid), .ch_data_o(w_data), .ts_o(w_ts),
        .hreg_we_i(w_we), .hreg_addr_i(w_addr), .hreg_wdata_i(w_hwdata),
        .hreg_rdata_o(w_rdata)
    );

    chan_props #(.STANDALONE(1), .ENV_SEQ(0), .FREE_INIT(1)) u_wrap_props (
        .clk(clk), .rst_n(w_rst_n),
        .ch_put_i(w_put), .ch_get_i(w_get), .ch_idx_i(w_idx), .ch_wdata_i(w_wdata),
        .ch_valid_o(w_valid), .ch_data_o(w_data), .ts_o(w_ts),
        .hreg_we_i(w_we), .hreg_addr_i(w_addr), .hreg_wdata_i(w_hwdata),
        .hreg_rdata_o(w_rdata)
    );
`endif
endmodule
