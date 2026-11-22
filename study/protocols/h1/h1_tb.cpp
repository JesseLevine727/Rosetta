// Verilator harness for the H1 prototype (study). Commands on stdin:
//   P period        set cfg_period_i (default 6)
//   R               two halted cycles (clears the decoder)
//   F levels        run one cycle per character ('0'/'1' = dline); answers
//                   "<bits>" (decoded bits in order, '-' if none) and the
//                   cycle of the first and last bit
//   Q
#include <cstdio>
#include <iostream>
#include <string>
#include "Vrosetta_mdpll.h"
#include "verilated.h"

static Vrosetta_mdpll *m;
static int prev = 1;
static void tick() { m->clk = 0; m->eval(); m->clk = 1; m->eval(); }

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  m = new Vrosetta_mdpll;
  m->rst_n = 0; m->run_i = 1; m->abort_i = 0; m->wr_tick_i = 0; m->cfg_en_i = 1;
  m->cfg_period_i = 6; m->cfg_inv_i = 0; m->pt_i = 1; m->dline_i = 1; m->dedge_i = 0;
  tick(); tick(); m->rst_n = 1; tick();
  std::string line;
  std::ios::sync_with_stdio(false);
  setvbuf(stdout, nullptr, _IOLBF, 0);
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    char c = line[0];
    if (c == 'Q') break;
    if (c == 'P') { m->cfg_period_i = std::stoi(line.substr(1)); printf("ok\n"); }
    else if (c == 'R') { m->run_i = 0; tick(); tick(); m->run_i = 1; printf("ok\n"); }
    else if (c == 'F') {
      std::string bits;
      long first = -1, last = -1;
      for (size_t i = 2; i < line.size(); i++) {
        int lv = line[i] == '1';
        m->dline_i = lv; m->dedge_i = (lv != prev); prev = lv;
        m->eval();
        if (m->st_o) { bits.push_back(m->rxb_o ? '1' : '0'); if (first < 0) first = i - 2; last = i - 2; }
        tick();
      }
      printf("%s %ld %ld\n", bits.empty() ? "-" : bits.c_str(), first, last);
    } else printf("error\n");
  }
  delete m;
  return 0;
}
