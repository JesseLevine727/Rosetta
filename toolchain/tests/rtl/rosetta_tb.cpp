// Cycle-level harness for the Rosetta chip top (src/rosetta_top.sv) under
// Verilator, driven line by line over stdin by toolchain/tests/rtlsim.py.
//
// It plays the demo board: clocks the chip, holds reset, bit-bangs the host
// SPI port on ui[0..2] (mode 0, SCLK half period 8 cycles = f_clk/16, the
// host.md 2.1 limit) and wires chip outputs back to chip inputs like the
// FPGA harness loopback (here with no extra register). Nothing here knows
// about lanes, programs or protocols.
//
// Commands (one per line; every command answers one line):
//   R n          run n cycles                                  -> "ok"
//   W a d        host register write (a, d hex)                -> "ok"
//   X a          host register read                            -> "<hex>"
//   L f t        wire uo[f] -> ui[t] (both 0..7)               -> "ok"
//   F off len    one-shot fault: at the next falling edge of the first
//                wire's source, invert every wire during
//                [edge + off, edge + off + len)                 -> "ok"
//   C            current cycle                                 -> "<dec>"
//   D            dump pin changes since the last D: lines "cycle uo ui",
//                then "end"
//   Q            quit
//
// Cycle n is the n-th rising edge. "uo in cycle n" is uo_out after edge n;
// "ui in cycle n" is what the harness applies before edge n + 1.
#include <cstdint>
#include <cstdio>
#include <cstring>
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

struct Change { uint64_t c; uint8_t uo, ui; };
static std::vector<Change> trace;
static int last_uo = -1, last_ui = -1;

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
      fault_armed = false;
      fault_seen = true;
      fault_lo = cyc + fault_off;
      fault_hi = fault_lo + fault_len;
    }
    fault_prev = src;
  }
  uint8_t ui = apply_wires();
  top->ui_in = ui;
  if (top->uo_out != last_uo || ui != last_ui) {
    trace.push_back({cyc, top->uo_out, ui});
    last_uo = top->uo_out;
    last_ui = ui;
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

  char line[256];
  setvbuf(stdout, nullptr, _IOLBF, 0);
  while (fgets(line, sizeof line, stdin)) {
    char cmd = line[0];
    if (cmd == 'Q') break;
    if (cmd == 'R') {
      unsigned long long n = strtoull(line + 1, nullptr, 10);
      run(n);
      printf("ok\n");
    } else if (cmd == 'W') {
      unsigned a = 0, d = 0;
      sscanf(line + 1, "%x %x", &a, &d);
      spi_frame(((uint64_t)(a & 0x7F) << 32) | d, 40);
      printf("ok\n");
    } else if (cmd == 'X') {
      unsigned a = 0;
      sscanf(line + 1, "%x", &a);
      uint64_t in = spi_frame((uint64_t)(0x80 | (a & 0x7F)) << 40, 48);
      printf("%08x\n", (unsigned)(in & 0xFFFFFFFFu));
    } else if (cmd == 'L') {
      int f = 0, t = 0;
      sscanf(line + 1, "%d %d", &f, &t);
      wires.push_back({f, t});
      printf("ok\n");
    } else if (cmd == 'F') {
      unsigned long long off = 0, len = 0;
      sscanf(line + 1, "%llu %llu", &off, &len);
      fault_off = off;
      fault_len = len;
      fault_armed = true;
      fault_seen = false;
      fault_prev = wires.empty() ? 1 : (top->uo_out >> wires[0].from) & 1;
      printf("ok\n");
    } else if (cmd == 'C') {
      printf("%llu\n", (unsigned long long)cyc);
    } else if (cmd == 'D') {
      for (const Change &c : trace)
        printf("%llu %02x %02x\n", (unsigned long long)c.c, c.uo, c.ui);
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
