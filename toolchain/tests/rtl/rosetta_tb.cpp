// Cycle-level harness for the Rosetta chip top (src/rosetta_top.sv) under
// Verilator, driven line by line over stdin by toolchain/tests/rtlsim.py.
// The FPGA harness (fpga/pynq-z1, lockstep mode) implements the same
// commands with the same meaning to the chip edge; the board regression
// compares the two transcripts byte for byte.
//
// It plays the demo board: clocks the chip, holds reset, bit-bangs the host
// SPI port on ui[0..2] (mode 0, SCLK half period 8 cycles = f_clk/16, the
// host.md 2.1 limit) and wires chip outputs back to chip inputs like the
// FPGA harness loopback (here with no extra register), and can join
// bidirectional pins into open-drain nets: a pin pulls its net low when its
// uio_oe is 1 and uio_out is 0, a net nobody pulls reads 1 (pull-up), and
// every pin of a net reads the net. Nothing here knows about lanes,
// programs or protocols.
//
// Stimulus (from the protocol study's harness, study/protocols/sim/study_tb.cpp):
// a waveform player and a CAN bus model.
//  * Waveform player: 8 channels, each mapped to one chip input pin (0-7 =
//    ui[0..7], 8-15 = uio[0..7]). A queue of runs (drive mask, value mask,
//    n cycles) is played one cycle per chip edge, during any command that
//    clocks the chip; an empty queue drives nothing. A ui pin takes its
//    channel's value while the channel drives, else its pull value if one is
//    set (U), else its normal source (host, wires). A uio pin with a channel
//    or a pull reads: the channel's value while it drives, else the chip's
//    own uio_out while uio_oe = 1, else the pull (default 1); a cycle in
//    which a channel and the chip both drive a pin counts as contention (K).
//    Channel 7 is also the CAN bus model's external node.
//  * CAN bus model (Y): up to 4 nodes, each a TXD net (a uio pin mask,
//    wired-AND of the pins whose uio_oe = 1 and uio_out = 0, pull-up) and an
//    RXD ui pin. bus = AND of every node's net delayed dtx cycles, and of
//    the external node (channel 7: dominant while it drives 0); each RXD
//    reads the bus delayed drx more cycles; each node's uio pins read their
//    own net (TXD read-back) with no delay. The delay lines record every
//    cycle from reset (initial value 1), whether or not the model is on.
//
// Per cycle, the inputs are formed in this order: host bits and wires
// (with the fault), nets, channels and pulls, the CAN bus model, then the
// per-pin skew of uio_in.
//
// Commands (one per line; every command answers one line):
//   R n          run n cycles                                  -> "ok"
//   W a d        host register write (a, d hex)                -> "ok"
//   X a          host register read                            -> "<hex>"
//   L f t        wire uo[f] -> ui[t] (both 0..7)               -> "ok"
//   N mask       join the uio pins in mask (hex) into one open-drain net  -> "ok"
//   S pin d      uio pin `pin` reads its net d cycles late (synchroniser skew) -> "ok"
//   F off len    one-shot fault: at the next falling edge of the first
//                wire's source, invert every wire during
//                [edge + off, edge + off + len)                 -> "ok"
//   M ch pin     map waveform channel ch (0-7) to pin (0-15; -1 unmaps) -> "ok"
//   U pin v      pull value of pin (0-15) when nothing drives it: 0, 1, or -1
//                for none (the pin's normal source)             -> "ok"
//   V d v n ...  append runs to the waveform queue: n cycles (1..65535 per
//                run) of drive mask d and value mask v (hex)    -> "ok"
//   P n          run until the waveform queue is empty, then n cycles -> "ok"
//   K            contention count (pin-cycles)                 -> "<dec>"
//   T 0|1        pin-change trace off / on (default on)        -> "ok"
//   Y dtx drx [m r]...  CAN bus model on: delays (0-255 cycles) and up to 4
//                nodes, each a TXD uio pin mask m (hex) and an RXD ui pin r
//                (0-7); "Y" with no nodes turns it off          -> "ok"
//   C            current cycle                                 -> "<dec>"
//   D            dump pin changes since the last D: lines
//                "cycle uo ui uio_out uio_oe uio_in" (hex), then "end"
//   Q            quit
//
// Cycle n is the n-th rising edge. "uo in cycle n" is uo_out after edge n;
// "ui in cycle n" is what the harness applies before edge n + 1.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Vtt_um_jesselevine727_rosetta.h"
#include "verilated.h"

static Vtt_um_jesselevine727_rosetta *top;
static uint64_t cyc = 0;
static uint8_t host_ui = 0x02;  // ui[0] SCLK low, ui[1] CS_n high, ui[2] MOSI low

struct Wire { int from, to; };
static std::vector<Wire> wires;
static bool fault_armed = false, fault_seen = false;
static uint64_t fault_lo = 0, fault_hi = 0, fault_off = 0, fault_len = 0;
static int fault_prev = 1;

static std::vector<uint8_t> nets;    // uio pin masks, wired-AND with pull-up
static int skew[8];                  // per uio pin input delay, cycles (< 64)
static uint8_t hist[64];             // resolved uio_in of the last 64 cycles

// waveform player
static int chpin[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
static int pullv[16];                // -1 = none
struct Run { uint8_t d, v; uint32_t n; };
static std::deque<Run> wq;
static uint8_t cur_d = 0, cur_v = 0;
static uint64_t contention = 0;

// CAN bus model
struct Node { uint8_t mask; int rx; };
static std::vector<Node> nodes;      // empty = off
static int can_dtx = 0, can_drx = 0;
static uint8_t node_h[4][256], bus_h[256];

struct Change { uint64_t c; uint8_t uo, ui, uout, uoe, uin; };
static std::vector<Change> trace;
static bool tracing = true;
static int last_uo = -1, last_ui = -1, last_uout = -1, last_uoe = -1, last_uin = -1;

static void pop_wave() {
  cur_d = 0;
  cur_v = 0;
  while (!wq.empty() && wq.front().n == 0) wq.pop_front();
  if (!wq.empty()) {
    cur_d = wq.front().d;
    cur_v = wq.front().v;
    if (--wq.front().n == 0) wq.pop_front();
  }
}

static uint8_t resolve_nets() {
  uint8_t uin = 0xFF;                    // unconnected pins read 1, as before
  uint8_t pull = top->uio_oe & ~top->uio_out;
  for (uint8_t m : nets)
    if (pull & m) uin &= (uint8_t)~m;
  return uin;
}

static uint8_t apply_skew(uint8_t uin) {
  hist[cyc % 64] = uin;
  for (int k = 0; k < 8; k++)
    if (skew[k] && cyc >= (uint64_t)skew[k]) {
      uint8_t old = hist[(cyc - skew[k]) % 64];
      uin = (uint8_t)((uin & ~(1u << k)) | (old & (1u << k)));
    }
  return uin;
}

static uint8_t apply_wires() {
  uint8_t u = host_ui & 0x07;
  bool inv = fault_seen && cyc >= fault_lo && cyc < fault_hi;
  for (const Wire &w : wires) {
    int v = (top->uo_out >> w.from) & 1;
    if (inv) v ^= 1;
    u = (uint8_t)((u & ~(1u << w.to)) | (v << w.to));
  }
  return u;
}

static void apply_channels(uint8_t &ui, uint8_t &uin) {
  for (int p = 0; p < 16; p++) {
    int ch = -1;
    for (int k = 0; k < 8; k++)
      if (chpin[k] == p) ch = k;                 // the highest channel wins
    bool wd = ch >= 0 && ((cur_d >> ch) & 1);
    int wv = ch >= 0 ? ((cur_v >> ch) & 1) : 0;
    if (p < 8) {
      if (wd) ui = (uint8_t)((ui & ~(1u << p)) | (wv << p));
      else if (pullv[p] >= 0) ui = (uint8_t)((ui & ~(1u << p)) | (pullv[p] << p));
    } else {
      int k = p - 8;
      if (ch < 0 && pullv[p] < 0) continue;
      bool cd = (top->uio_oe >> k) & 1;
      int cv = (top->uio_out >> k) & 1;
      int v = wd ? wv : cd ? cv : (pullv[p] >= 0 ? pullv[p] : 1);
      if (wd && cd) contention++;
      uin = (uint8_t)((uin & ~(1u << k)) | (v << k));
    }
  }
}

static void apply_canbus(uint8_t &ui, uint8_t &uin) {
  uint8_t pull = top->uio_oe & ~top->uio_out;
  int net[4];
  int bus = !(((cur_d >> 7) & 1) && !((cur_v >> 7) & 1));   // external node
  for (int k = 0; k < 4; k++) {
    uint8_t m = k < (int)nodes.size() ? nodes[k].mask : 0;
    net[k] = (pull & m) ? 0 : 1;
    node_h[k][cyc & 255] = (uint8_t)net[k];
    if (k < (int)nodes.size()) bus &= can_dtx ? node_h[k][(cyc - can_dtx) & 255] : net[k];
  }
  bus_h[cyc & 255] = (uint8_t)bus;
  int rx = can_drx ? bus_h[(cyc - can_drx) & 255] : bus;
  for (int k = 0; k < (int)nodes.size(); k++) {
    if (nodes[k].rx >= 0) ui = (uint8_t)((ui & ~(1u << nodes[k].rx)) | (rx << nodes[k].rx));
    uin = (uint8_t)((uin & ~nodes[k].mask) | (net[k] ? nodes[k].mask : 0));
  }
}

static void tick() {
  if (!wires.empty() && fault_armed) {
    int src = (top->uo_out >> wires[0].from) & 1;
    if (fault_prev == 1 && src == 0) {
      fault_armed = false;
      fault_seen = true;
      fault_lo = cyc + fault_off;
      fault_hi = fault_lo + fault_len;
    }
    fault_prev = src;
  }
  pop_wave();
  uint8_t ui = apply_wires();
  uint8_t uin = resolve_nets();
  apply_channels(ui, uin);
  apply_canbus(ui, uin);
  uin = apply_skew(uin);
  top->ui_in = ui;
  top->uio_in = uin;
  if (tracing && (top->uo_out != last_uo || ui != last_ui || top->uio_out != last_uout ||
                  top->uio_oe != last_uoe || uin != last_uin)) {
    trace.push_back({cyc, top->uo_out, ui, top->uio_out, top->uio_oe, uin});
    last_uo = top->uo_out;
    last_ui = ui;
    last_uout = top->uio_out;
    last_uoe = top->uio_oe;
    last_uin = uin;
  }
  top->clk = 0;
  top->eval();
  top->clk = 1;
  top->eval();
  cyc++;
}

static void run(uint64_t n) {
  for (uint64_t i = 0; i < n; i++) tick();
}

// One host SPI frame of nbits, MSB first; returns the bits the chip sent.
static uint64_t spi_frame(uint64_t out, int nbits) {
  const int H = 8;
  uint64_t in = 0;
  host_ui = 0x00;  // CS_n low, SCLK low
  run(H);
  for (int i = nbits - 1; i >= 0; i--) {
    int b = (out >> i) & 1;
    host_ui = (uint8_t)(b << 2);  // MOSI, SCLK low
    run(H);
    in = (in << 1) | (top->uo_out & 1);  // sample MISO at SCLK rising
    host_ui = (uint8_t)((b << 2) | 1);
    run(H);
  }
  host_ui = (uint8_t)(host_ui & ~1u);
  run(H);
  host_ui = 0x02;  // CS_n high
  run(H);
  return in;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  for (int i = 0; i < 16; i++) pullv[i] = -1;
  memset(node_h, 1, sizeof node_h);
  memset(bus_h, 1, sizeof bus_h);
  top = new Vtt_um_jesselevine727_rosetta;
  top->ena = 1;
  top->uio_in = 0xFF;
  top->ui_in = 0x02;
  top->rst_n = 0;
  top->clk = 0;
  top->eval();
  run(10);
  top->rst_n = 1;
  run(10);

  std::string line;
  std::ios::sync_with_stdio(false);
  setvbuf(stdout, nullptr, _IOLBF, 0);
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    char cmd = line[0];
    std::istringstream is(line.substr(1));
    if (cmd == 'Q') break;
    if (cmd == 'R') {
      unsigned long long n = 0;
      is >> n;
      run(n);
      printf("ok\n");
    } else if (cmd == 'W') {
      unsigned a = 0, d = 0;
      is >> std::hex >> a >> d;
      spi_frame(((uint64_t)(a & 0x7F) << 32) | d, 40);
      printf("ok\n");
    } else if (cmd == 'X') {
      unsigned a = 0;
      is >> std::hex >> a;
      uint64_t in = spi_frame((uint64_t)(0x80 | (a & 0x7F)) << 40, 48);
      printf("%08x\n", (unsigned)(in & 0xFFFFFFFFu));
    } else if (cmd == 'L') {
      int f = 0, t = 0;
      is >> f >> t;
      wires.push_back({f, t});
      printf("ok\n");
    } else if (cmd == 'N') {
      unsigned m = 0;
      is >> std::hex >> m;
      nets.push_back((uint8_t)m);
      printf("ok\n");
    } else if (cmd == 'S') {
      int k = 0, d = 0;
      is >> k >> d;
      if (k >= 0 && k < 8 && d >= 0 && d < 64) skew[k] = d;
      printf("ok\n");
    } else if (cmd == 'F') {
      unsigned long long off = 0, len = 0;
      is >> off >> len;
      fault_off = off;
      fault_len = len;
      fault_armed = true;
      fault_seen = false;
      fault_prev = wires.empty() ? 1 : (top->uo_out >> wires[0].from) & 1;
      printf("ok\n");
    } else if (cmd == 'M') {
      int ch = -1, p = -1;
      is >> ch >> p;
      if (ch >= 0 && ch < 8 && p >= -1 && p < 16) {
        chpin[ch] = p;
        printf("ok\n");
      } else {
        printf("error M ch pin\n");
      }
    } else if (cmd == 'U') {
      int p = -1, v = -1;
      is >> p >> v;
      if (p >= 0 && p < 16 && v >= -1 && v <= 1) {
        pullv[p] = v;
        printf("ok\n");
      } else {
        printf("error U pin v\n");
      }
    } else if (cmd == 'V') {
      std::string d, v;
      unsigned long long n;
      bool ok = true;
      while (is >> d >> v >> n) {
        if (n < 1 || n > 65535) ok = false;
        wq.push_back({(uint8_t)std::stoul(d, nullptr, 16), (uint8_t)std::stoul(v, nullptr, 16),
                      (uint32_t)n});
      }
      printf(ok ? "ok\n" : "error V run length 1..65535\n");
    } else if (cmd == 'P') {
      unsigned long long n = 0;
      is >> n;
      while (!wq.empty()) tick();
      run(n);
      printf("ok\n");
    } else if (cmd == 'K') {
      printf("%llu\n", (unsigned long long)contention);
    } else if (cmd == 'T') {
      int t = 1;
      is >> t;
      tracing = t != 0;
      printf("ok\n");
    } else if (cmd == 'Y') {
      int dtx = 0, drx = 0;
      is >> dtx >> drx;
      std::vector<Node> nn;
      std::string m;
      int r;
      while (is >> m >> r) nn.push_back({(uint8_t)std::stoul(m, nullptr, 16), r});
      if (dtx < 0 || dtx > 255 || drx < 0 || drx > 255 || nn.size() > 4) {
        printf("error Y dtx drx [mask rx]...\n");
      } else {
        can_dtx = dtx;
        can_drx = drx;
        nodes = nn;
        printf("ok\n");
      }
    } else if (cmd == 'C') {
      printf("%llu\n", (unsigned long long)cyc);
    } else if (cmd == 'D') {
      for (const Change &c : trace)
        printf("%llu %02x %02x %02x %02x %02x\n", (unsigned long long)c.c, c.uo, c.ui, c.uout,
               c.uoe, c.uin);
      trace.clear();
      printf("end\n");
    } else {
      printf("error unknown command\n");
    }
  }
  top->final();
  delete top;
  return 0;
}
