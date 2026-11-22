#!/usr/bin/env python3
"""Write the pre-amendment lane ("today's lane") for the zero-rule miter.

Extracts src/rosetta_{lane,cfg,tick,delay,coding,field,crc,stuff,events,pins}.sv
at revision BASE_REV (the last commit before the H3/H4 RTL, the same pinned
revision as gen_wb.py) with git show, renames every module rosetta_X to
rosetta_X_h so it can sit next to the amended lane in one design, and writes
them to ref/build/zero_old/ (git-ignored). The files are compiled only.

Usage: python3 gen_zero.py   (run from formal/lane/, before lane_zero.sby)
"""
import os, re, subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
BASE_REV = '3937e06'
MODS = ['lane', 'cfg', 'tick', 'delay', 'coding', 'field', 'crc', 'stuff', 'events', 'pins']
OUT = os.path.join(HERE, 'ref', 'build', 'zero_old')
os.makedirs(OUT, exist_ok=True)
pat = re.compile(r'\brosetta_(' + '|'.join(MODS) + r')\b')
for m in MODS:
    txt = subprocess.run(['git', 'show', '%s:src/rosetta_%s.sv' % (BASE_REV, m)], cwd=HERE,
                         check=True, capture_output=True, text=True).stdout
    with open(os.path.join(OUT, 'rosetta_%s_h.sv' % m), 'w') as f:
        f.write(pat.sub(lambda x: x.group(0) + '_h', txt))
print('wrote', len(MODS), 'files to', OUT)

# ---------------------------------------------------------------------------
# The PREVIOUS lane property set (formal/lane at OLDSET_REV = BASE_REV),
# for the zero-rule run "the previous properties still pass on the amended
# lane when the new bits are never written": written to
# ref/build/zero_oldset/ with exactly one change, the zero assumption
# (SETM TICK data[15:14] = 0, SETM CODE data[12] = 0) inserted next to
# u_afterwr. The white-box plumbing (lane_wb.svh, lane_wb_*.ys) is the
# current one (the sub-block port widths of the amended RTL); the previous
# properties read the low 13 TICK / 12 CODE bits as before. bmc_gold is not
# run there (the previous lane_model.sv has no connection for the new
# sub-block inputs).
OLDSET_REV = BASE_REV   # formal/lane at the same pinned pre-amendment revision
SET = os.path.join(HERE, 'ref', 'build', 'zero_oldset')
os.makedirs(SET, exist_ok=True)
def show(path):
    return subprocess.run(['git', 'show', '%s:formal/lane/%s' % (OLDSET_REV, path)], cwd=HERE,
                          check=True, capture_output=True, text=True).stdout
props = show('lane_props.sv')
anchor = "        if (f_pv && p_rstn) u_afterwr: assume (!p_wrtick || !data_wait_i);\n"
assert props.count(anchor) == 1
props = props.replace(anchor, anchor +
    "        // zero rule (gen_zero.py): the new mode bits are never written as 1\n"
    "        u_zero_tick: assume (!(wlo && cfg_reg_i == 5'd0) || cfg_data_i[15:14] == 2'd0);\n"
    "        u_zero_code: assume (!(wlo && cfg_reg_i == 5'd5) || !cfg_data_i[12]);\n")
open(os.path.join(SET, 'lane_props.sv'), 'w').write(props)
for f in ['lane_fv.sv', 'lane_struct.ys', 'lane_model.sv']:
    open(os.path.join(SET, f), 'w').write(show(f))
sby = show('lane.sby')
sby = sby.replace('../../src/', os.path.join(HERE, '..', '..', 'src') + '/')
for f in ['lane_wb.svh', 'lane_wb_dut.ys', 'lane_wb_r53.ys']:
    sby = re.sub(r'^%s$' % re.escape(f), '%s %s' % (f, os.path.join(HERE, f)), sby, flags=re.M)
open(os.path.join(SET, 'lane.sby'), 'w').write(sby)
print('wrote the previous property set (zero assumption added) to', SET)
