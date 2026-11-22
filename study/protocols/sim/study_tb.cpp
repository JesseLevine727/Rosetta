// Study harness: a copy of toolchain/tests/rtl/rosetta_tb.cpp (the cycle-level
// harness of the Rosetta chip top under Verilator) with three additions used by
// study/protocols only. The repository harness is not changed.
//
//   1. Waveform injection. Up to 8 "channels", each mapped to one chip input
//      pin (index 0-7 = ui[0..7], 8-15 = uio[0..7]). A queue of per-cycle
//      channel states (drive mask, value mask) is played one entry per cycle.
//      A channel that does not drive leaves its pin to the chip (uio pins the
//      chip drives read the chip's value) or to the pin's pull value.
//      Commands:
//        M ch pin        map channel ch to pin
//        U pin v         pull value of a pin nobody drives (default 1 for uio,
//                        for ui: the host/wire value)
//        V d v n [...]   append runs: n cycles of (drive mask d, value mask v), hex d v, dec n
//        P n             run until the queue is empty, then n more cycles -> "ok"
//        K               number of cycles in which a channel and the chip both drove a uio pin
//        T 0|1           pin-change tracing off / on (default on)
//   2. CAN bus model, "B dtx drx": node A's TXD net = wired-AND of uio[0] and
//      uio[1] (open-drain, pull-up), node B's = uio[2] and uio[3]; the bus is
//      the AND of both nets delayed dtx cycles and of channel 7 (an external
//      node: 0 = dominant, Z or 1 = recessive); ui[3] (node A RXD) and ui[4]
//      (node B RXD) read the bus delayed drx more cycles. uio[k] read their
//      own net (TXD read-back) with no delay.
//   3. "G" returns the cycle of the first change of uio_oe/uio_out after the
//      last "G" (not used by the scripts; kept for interactive debugging).
//
// Base commands as the original: R n, W a d, X a, L f t, N mask, S pin d,
// F off len, C, D, Q.
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
static uint8_t host_ui = 0x02;

struct Wire { int from, to; };
static std::vector<Wire> wires;
static bool fault_armed = false, fault_seen = false;
static uint64_t fault_lo = 0, fault_hi = 0, fault_off = 0, fault_len = 0;
static int fault_prev = 1;

static std::vector<uint8_t> nets;
static int skew[8];
static uint8_t hist[64];

struct Change { uint64_t c; uint8_t uo, ui, uout, uoe, uin; };
static std::vector<Change> trace;
static bool tracing = true;
static int last_uo = -1, last_ui = -1, last_uout = -1, last_uoe = -1, last_uin = -1;

// waveform channels
static int chpin[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
static int pull[16];          // -1 = default
struct Ent { uint8_t d, v; uint32_t n; };
static std::deque<Ent> wq;
static uint64_t contention = 0;
static uint8_t cur_d = 0, cur_v = 0;

// CAN bus
static bool canbus = false;
static int can_dtx = 0, can_drx = 0;
static uint8_t netA_h[256], netB_h[256], bus_h[256];

static void pop_wave() {
  cur_d = 0; cur_v = 0;
  while (!wq.empty() && wq.front().n == 0) wq.pop_front();
  if (!wq.empty()) {
    cur_d = wq.front().d; cur_v = wq.front().v;
    if (--wq.front().n == 0) wq.pop_front();
  }
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

static void tick() {
  if (!wires.empty() && fault_armed) {
    int src = (top->uo_out >> wires[0].from) & 1;
    if (fault_prev == 1 && src == 0) {
      fault_armed = false; fault_seen = true;
      fault_lo = cyc + fault_off; fault_hi = fault_lo + fault_len;
    }
    fault_prev = src;
  }
  pop_wave();
  uint8_t ui = apply_wires();
  // uio: open-drain nets first (original behaviour)
  uint8_t uin = 0xFF;
  uint8_t pullmask = top->uio_oe & ~top->uio_out;
  for (uint8_t m : nets)
    if (pullmask & m) uin &= (uint8_t)~m;
  // pins with channels or explicit pulls
  for (int p = 0; p < 16; p++) {
    int ch = -1;
    for (int k = 0; k < 8; k++) if (chpin[k] == p) ch = k;
    bool wd = ch >= 0 && ((cur_d >> ch) & 1);
    int wv = ch >= 0 ? ((cur_v >> ch) & 1) : 0;
    if (p < 8) {
      if (wd) ui = (uint8_t)((ui & ~(1u << p)) | (wv << p));
      else if (pull[p] >= 0) ui = (uint8_t)((ui & ~(1u << p)) | (pull[p] << p));
    } else {
      int k = p - 8;
      bool cd = (top->uio_oe >> k) & 1;
      int cv = (top->uio_out >> k) & 1;
      if (ch < 0 && pull[p] < 0) continue;     // original behaviour
      int v = wd ? wv : cd ? cv : (pull[p] >= 0 ? pull[p] : 1);
      if (wd && cd) contention++;
      uin = (uint8_t)((uin & ~(1u << k)) | (v << k));
    }
  }
  if (canbus) {
    int a = !((top->uio_oe & 1) && !(top->uio_out & 1)) && !((top->uio_oe & 2) && !(top->uio_out & 2));
    int b = !((top->uio_oe & 4) && !(top->uio_out & 4)) && !((top->uio_oe & 8) && !(top->uio_out & 8));
    netA_h[cyc & 255] = a; netB_h[cyc & 255] = b;
    int ext = !(((cur_d >> 7) & 1) && !((cur_v >> 7) & 1));
    int ad = cyc >= (uint64_t)can_dtx ? netA_h[(cyc - can_dtx) & 255] : 1;
    int bd = cyc >= (uint64_t)can_dtx ? netB_h[(cyc - can_dtx) & 255] : 1;
    int bus = ad & bd & ext;
    bus_h[cyc & 255] = bus;
    int rx = cyc >= (uint64_t)can_drx ? bus_h[(cyc - can_drx) & 255] : 1;
    ui = (uint8_t)((ui & ~0x18u) | (rx << 3) | (rx << 4));
    uin = (uint8_t)((uin & ~0x0Fu) | (a ? 3 : 0) | (b ? 12 : 0));
  }
  hist[cyc % 64] = uin;
  for (int k = 0; k < 8; k++)
    if (skew[k] && cyc >= (uint64_t)skew[k]) {
      uint8_t old = hist[(cyc - skew[k]) % 64];
      uin = (uint8_t)((uin & ~(1u << k)) | (old & (1u << k)));
    }
  top->ui_in = ui;
  top->uio_in = uin;
  if (tracing && (top->uo_out != last_uo || ui != last_ui || top->uio_out != last_uout ||
                  top->uio_oe != last_uoe || uin != last_uin)) {
    trace.push_back({cyc, top->uo_out, ui, top->uio_out, top->uio_oe, uin});
    last_uo = top->uo_out; last_ui = ui; last_uout = top->uio_out;
    last_uoe = top->uio_oe; last_uin = uin;
  }
  top->clk = 0; top->eval();
  top->clk = 1; top->eval();
  cyc++;
}

static void run(uint64_t n) { for (uint64_t i = 0; i < n; i++) tick(); }

static uint64_t spi_frame(uint64_t out, int nbits) {
  const int H = 8;
  uint64_t in = 0;
  host_ui = 0x00; run(H);
  for (int i = nbits - 1; i >= 0; i--) {
    int b = (out >> i) & 1;
    host_ui = (uint8_t)(b << 2); run(H);
    in = (in << 1) | (top->uo_out & 1);
    host_ui = (uint8_t)((b << 2) | 1); run(H);
  }
  host_ui = (uint8_t)(host_ui & ~1u); run(H);
  host_ui = 0x02; run(H);
  return in;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  for (int i = 0; i < 16; i++) pull[i] = -1;
  top = new Vtt_um_jesselevine727_rosetta;
  top->ena = 1; top->uio_in = 0xFF; top->ui_in = 0x02; top->rst_n = 0; top->clk = 0;
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
    if (cmd == 'R') { unsigned long long n; is >> n; run(n); printf("ok\n"); }
    else if (cmd == 'W') {
      unsigned a = 0, d = 0; is >> std::hex >> a >> d;
      spi_frame(((uint64_t)(a & 0x7F) << 32) | d, 40); printf("ok\n");
    } else if (cmd == 'X') {
      unsigned a = 0; is >> std::hex >> a;
      uint64_t in = spi_frame((uint64_t)(0x80 | (a & 0x7F)) << 40, 48);
      printf("%08x\n", (unsigned)(in & 0xFFFFFFFFu));
    } else if (cmd == 'L') { int f, t; is >> f >> t; wires.push_back({f, t}); printf("ok\n"); }
    else if (cmd == 'N') { unsigned m; is >> std::hex >> m; nets.push_back((uint8_t)m); printf("ok\n"); }
    else if (cmd == 'S') { int k, d; is >> k >> d; if (k >= 0 && k < 8 && d >= 0 && d < 64) skew[k] = d; printf("ok\n"); }
    else if (cmd == 'F') {
      unsigned long long off, len; is >> off >> len;
      fault_off = off; fault_len = len; fault_armed = true; fault_seen = false;
      fault_prev = wires.empty() ? 1 : (top->uo_out >> wires[0].from) & 1; printf("ok\n");
    } else if (cmd == 'C') printf("%llu\n", (unsigned long long)cyc);
    else if (cmd == 'D') {
      for (const Change &c : trace)
        printf("%llu %02x %02x %02x %02x %02x\n", (unsigned long long)c.c, c.uo, c.ui, c.uout, c.uoe, c.uin);
      trace.clear(); printf("end\n");
    } else if (cmd == 'M') { int ch, p; is >> ch >> p; if (ch >= 0 && ch < 8) chpin[ch] = p; printf("ok\n"); }
    else if (cmd == 'U') { int p, v; is >> p >> v; if (p >= 0 && p < 16) pull[p] = v; printf("ok\n"); }
    else if (cmd == 'V') {
      std::string d, v; unsigned long long n;
      while (is >> d >> v >> n)
        wq.push_back({(uint8_t)std::stoul(d, nullptr, 16), (uint8_t)std::stoul(v, nullptr, 16), (uint32_t)n});
      printf("ok\n");
    } else if (cmd == 'P') {
      unsigned long long n; is >> n;
      while (!wq.empty()) tick();
      run(n); printf("ok\n");
    } else if (cmd == 'K') printf("%llu\n", (unsigned long long)contention);
    else if (cmd == 'T') { int t; is >> t; tracing = t != 0; printf("ok\n"); }
    else if (cmd == 'B') {
      is >> can_dtx >> can_drx; canbus = true;
      for (int i = 0; i < 256; i++) netA_h[i] = netB_h[i] = bus_h[i] = 1;
      printf("ok\n");
    } else printf("error unknown command\n");
  }
  top->final();
  delete top;
  return 0;
}
