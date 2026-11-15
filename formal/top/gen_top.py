#!/usr/bin/env python3
"""Generate the white-box plumbing and the component shells of the top proofs.

Run from formal/top/:  python3 gen_top.py

Inputs: the PORT LISTS of the components the chip top instantiates
(top.md section 1). For src/rosetta_lane.sv only the port list is read, with
exactly `sed -n '/^module rosetta_lane/,/);/p'` (the lane RTL body is not
read). src/rosetta_top.sv is not read at all.

Outputs (all committed, regenerate after a component port change):

  top_wb.svh     `define TOP_WBW and, inside `ifdef TOP_WB_WIRES, one named
                 wire per observed port: w_<inst>_<port> = wb[off +: width];
                 with `ifdef TOP_WB2_WIRES the same for a second copy:
                 v_<inst>_<port> = wb2[off +: width].
  top_wb.ys      yosys `connect -set` commands, run after `flatten` in the
                 proof top: each slice of wb is tied to the flattened port
                 of the design's instance (top.md section 1 instance names:
                 u_xbar, u_host, u_seq, u_mem_prog, u_mem_data, u_chan,
                 gen_lane[L].u_lane) under the DUT instance u_dut.
  top_wb_ab.ys   the same for two copies: wb <- u_a, wb2 <- u_b.
  top_wb_mac.ys / top_wb_mac_ab.ys
                 the SRAM macro pins inside the (real, proven) wrappers,
                 u_mem_prog.u_sram.u_macro and u_mem_data.u_sram.u_macro
                 (instance names taken from the wrapper RTL). Only usable
                 when the wrappers are real (not shells).
  shells.sv      one SHELL per component: same module name, parameters and
                 ports as the real block; every output is a free ($anyseq)
                 value; inputs are kept. A proof that reads shells.sv
                 instead of a component's RTL black-boxes that component
                 (a cut point at its outputs). The lane shell asserts
                 RTRIP == 4 (lane.md 1, ruling 18) and the sequencer shell
                 N_LANES == 4 (sequencer.md: everything is for 4 lanes), so
                 a wrong instance parameter is caught.
  shell_macro_free.sv / shell_macro_ext.sv
                 the SRAM macro RM_IHPSG13_1P_512x32_c2_bm_bist with A_DOUT
                 free (anyseq) or undriven (driven by the proof top through
                 connect -set, used by the memory models of the two-copy
                 proofs).
"""
import os
import re
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', '..', 'src')
PARAMS = {'N_LANES': 4, 'RTRIP': 4}

# (prefix, instance path inside the top, module)
INSTS = [
    ('xbar', 'u_xbar', 'rosetta_xbar'),
    ('host', 'u_host', 'rosetta_host'),
    ('seq', 'u_seq', 'rosetta_seq'),
    ('mp', 'u_mem_prog', 'rosetta_mem_prog'),
    ('md', 'u_mem_data', 'rosetta_mem_data'),
    ('chan', 'u_chan', 'rosetta_chan'),
] + [('l%d' % L, 'gen_lane[%d].u_lane' % L, 'rosetta_lane') for L in range(4)]

MACRO_PORTS = [('A_CLK', 'in', 1), ('A_MEN', 'in', 1), ('A_WEN', 'in', 1),
               ('A_REN', 'in', 1), ('A_ADDR', 'in', 9), ('A_DIN', 'in', 32),
               ('A_DLY', 'in', 1), ('A_DOUT', 'out', 32), ('A_BM', 'in', 32),
               ('A_BIST_CLK', 'in', 1), ('A_BIST_EN', 'in', 1),
               ('A_BIST_MEN', 'in', 1), ('A_BIST_WEN', 'in', 1),
               ('A_BIST_REN', 'in', 1), ('A_BIST_ADDR', 'in', 9),
               ('A_BIST_DIN', 'in', 32), ('A_BIST_BM', 'in', 32)]
MACROS = [('pmac', 'u_mem_prog.u_sram.u_macro'), ('dmac', 'u_mem_data.u_sram.u_macro')]


def header(mod):
    """Module header text 'module ... );' -- for the lane exactly the sed."""
    path = os.path.join(SRC, mod + '.sv')
    out = subprocess.run(['sed', '-n', '/^module %s/,/);/p' % mod, path],
                         capture_output=True, text=True, check=True).stdout
    # sed prints every range that starts with "module <mod>"; keep the first
    end = out.index(');') + 2
    return out[:end]


def ports(hdr):
    """[(dir, range_text_or_None, name, width)] from a header."""
    body = hdr
    m = re.match(r'\s*module\s+\w+\s*#\s*\((.*?)\)\s*\((.*)\);\s*$', body, re.S)
    if m:
        plist = m.group(2)
    else:
        m = re.match(r'\s*module\s+\w+\s*\((.*)\);\s*$', body, re.S)
        plist = m.group(1)
    plist = re.sub(r'//[^\n]*', '', plist)
    res = []
    cur_dir, cur_rng = None, None
    for d in plist.split(','):
        d = d.strip()
        if not d:
            continue
        mm = re.match(r'(input|output)\s+(?:wire|logic)?\s*(\[[^\]]*\])?\s*(\w+)$', d)
        if mm:
            cur_dir, cur_rng, name = mm.group(1), mm.group(2), mm.group(3)
        else:
            name = d.split()[-1]
        w = 1
        if cur_rng:
            hi, lo = cur_rng[1:-1].split(':')
            env = dict(PARAMS)
            w = eval(hi, {}, env) - eval(lo, {}, env) + 1
        res.append((cur_dir, cur_rng, name, w))
    return res


def main():
    hdrs = {mod: header(mod) for _, _, mod in INSTS}
    sigs = []                      # (prefix, inst, port, width)
    for pfx, inst, mod in INSTS:
        for _d, _r, name, w in ports(hdrs[mod]):
            sigs.append((pfx, inst, name, w))
    msigs = []
    for pfx, inst in MACROS:
        for name, _d, w in MACRO_PORTS:
            msigs.append((pfx, inst, name, w))

    lay, off = [], 0
    for s in sigs + msigs:
        lay.append(s + (off,))
        off += s[3]
    total = off
    nport = sum(s[3] for s in sigs)

    with open(os.path.join(HERE, 'top_wb.svh'), 'w') as f:
        f.write('// GENERATED by gen_top.py -- do not edit. White-box layout of the\n')
        f.write('// top checker buses wb (copy A) and wb2 (copy B): one slice per\n')
        f.write('// observed instance port (top.md 1 instance names) and SRAM macro pin.\n')
        f.write('`ifndef TOP_WB_LAYOUT\n`define TOP_WB_LAYOUT\n')
        f.write('`define TOP_WBW %d\n' % total)
        f.write('`define TOP_WBW_PORTS %d\n' % nport)
        f.write('`endif\n')
        for guard, bus, p in (('TOP_WB_WIRES', 'wb', 'w'), ('TOP_WB2_WIRES', 'wb2', 'v')):
            f.write('`ifdef %s\n' % guard)
            for pfx, inst, name, w, o in lay:
                f.write('wire [%d:0] %s_%s_%s = %s[%d +: %d];\n' % (w - 1, p, pfx, name, bus, o, w))
            f.write('`endif\n')

    def ys(fname, pairs, part):
        with open(os.path.join(HERE, fname), 'w') as f:
            f.write('# GENERATED by gen_top.py -- do not edit. Run after flatten in the\n')
            f.write('# proof top: ties each white-box slice to the flattened port.\n')
            for bus, dut in pairs:
                f.write('# ---- %s <- %s\n' % (bus, dut))
                for pfx, inst, name, w, o in lay:
                    is_mac = pfx in ('pmac', 'dmac')
                    if is_mac != (part == 'mac'):
                        continue
                    f.write('connect -nounset -set %s[%d:%d] \\%s.%s.%s \n' % (bus, o + w - 1, o, dut, inst, name))

    ys('top_wb.ys', [('wb', 'u_dut')], 'ports')
    ys('top_wb_ab.ys', [('wb', 'u_a'), ('wb2', 'u_b')], 'ports')
    ys('top_wb_mac.ys', [('wb', 'u_dut')], 'mac')
    ys('top_wb_mac_ab.ys', [('wb', 'u_a'), ('wb2', 'u_b')], 'mac')

    # ---------------------------------------------------------------- shells
    with open(os.path.join(HERE, 'shells.sv'), 'w') as f:
        f.write('// GENERATED by gen_top.py -- do not edit.\n')
        f.write('// Component SHELLS: same name, parameters and ports as the real\n')
        f.write('// blocks; every output free ($anyseq), inputs kept. Reading this\n')
        f.write('// file instead of a block\'s RTL cuts the block at its outputs.\n')
        done = set()
        for _p, _i, mod in INSTS:
            if mod in done:
                continue
            done.add(mod)
            f.write('\n`ifndef TOP_REAL_%s\n' % mod.upper())
            f.write('`default_nettype none\n')
            f.write(hdrs[mod] + '\n')
            for d, r, name, w in ports(hdrs[mod]):
                if d == 'output':
                    f.write('  (* anyseq *) logic %s f_%s;\n' % (r or '', name))
                    f.write('  assign %s = f_%s;\n' % (name, name))
            if mod == 'rosetta_lane':
                f.write('  // lane.md 1 / ruling 18: RTRIP = IN_LAT + 1 = 4\n')
                f.write('  always @* a_shell_rtrip: assert (RTRIP == 4);\n')
            if mod == 'rosetta_seq':
                f.write('  // sequencer.md: everything is specified for N_LANES = 4\n')
                f.write('  always @* a_shell_nlanes: assert (N_LANES == 4);\n')
            f.write('endmodule\n`endif\n')

    mhdr = ('module RM_IHPSG13_1P_512x32_c2_bm_bist (\n' +
            ',\n'.join('    %s wire %s %s' % ('input ' if d == 'in' else 'output',
                                               '[%d:0]' % (w - 1) if w > 1 else '     ', n)
                       for n, d, w in MACRO_PORTS) + '\n);\n')
    for fname, body in (
            ('shell_macro_free.sv',
             '  // A_DOUT free in every cycle: a proof that needs no memory contents\n'
             '  (* anyseq *) logic [31:0] f_dout;\n  assign A_DOUT = f_dout;\n'),
            ('shell_macro_ext.sv',
             '  // A_DOUT left undriven here: the proof top drives it with\n'
             '  // connect -set from its own memory model (top_wb_mac*.ys pins).\n'
             '  wire unused_ok = 1\'b0;\n')):
        with open(os.path.join(HERE, fname), 'w') as f:
            f.write('// GENERATED by gen_top.py -- do not edit. SRAM macro model for the\n')
            f.write('// top proofs (mem.md 4: a model with the macro\'s name and ports;\n')
            f.write('// ROSETTA_SRAM_BEHAV undefined so the wrapper instantiates it).\n')
            f.write('`default_nettype none\n')
            f.write(mhdr + body + 'endmodule\n')
    print('TOP_WBW =', total, ' ports part =', nport, ' signals =', len(lay))


if __name__ == '__main__':
    main()
