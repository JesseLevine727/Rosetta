#!/usr/bin/env python3
"""
Run the tile's vertical power stripes across hard macros (OpenROAD odb script,
executed by the Project.ExtendStripesOverMacros LibreLane step).

Why: the Tiny Tapeout IHP SG13CMOS5L tile has a single-layer Metal4 PDN.
pdngen always carves its core-grid stripes around a placed block (its pins and
OBS become obstructions), so a stripe that crosses an SRAM is cut in two.  That
leaves the macro unpowered (its VDD!/VDDARRAY!/VSS! pins are Metal4 columns),
and the cut stripe segments become VPWR/VGND pin rectangles that stop far from
the tile edge, which the Tiny Tapeout precheck rejects ("Port VPWR is too far
from top edge of module").

What: every carved stripe column whose gaps are explained by macros is
re-drawn as one stripe over the full height of the grid, and its pin is
replaced by one full-height pin rectangle.  Before that the column is checked
against the macros it crosses: it must keep --clearance from every Metal4 pin
of the other supply net (and from the macro's Metal4 OBS, unless the master is
listed with --pins-only-master: for the IHP RM_IHPSG13 1P/2P SRAMs the Metal4
OBS is only a keep-out, the GDS has no Metal4 besides the supply columns; this
was checked for every 1P/2P macro of IHP-Open-PDK 89d42ee6 except
RM_IHPSG13_1P_8192x32_c4, which has other Metal4 and is never trusted).  If
the column is illegal, or legal but touching none of the macro's columns of its
own net, it is moved sideways (with its vias and pin) by the smallest legal
offset up to --max-nudge, preferring one that lands on a column of its own net.
A column that cannot be made legal is removed completely (with a warning).
"""
import json
import re
import sys

import click
import odb
from reader import click_odb


def rect_of(box):
    return (box.xMin(), box.yMin(), box.xMax(), box.yMax())


def transformed(inst, box):
    """Master-frame rectangle -> placed rectangle (macros with LEF ORIGIN 0 0)."""
    bb = inst.getBBox()
    W = inst.getMaster().getWidth()
    H = inst.getMaster().getHeight()
    x1, y1, x2, y2 = box.xMin(), box.yMin(), box.xMax(), box.yMax()
    o = str(inst.getOrient())
    if o == "R0":       # N
        r = (x1, y1, x2, y2)
    elif o == "MX":     # FS: mirror about the x axis
        r = (x1, H - y2, x2, H - y1)
    elif o == "MY":     # FN: mirror about the y axis
        r = (W - x2, y1, W - x1, y2)
    elif o == "R180":   # S
        r = (W - x2, H - y2, W - x1, H - y1)
    else:
        raise ValueError(f"{inst.getName()}: orientation {o} not supported")
    return (bb.xMin() + r[0], bb.yMin() + r[1], bb.xMin() + r[2], bb.yMin() + r[3])


def overlap_1d(a1, a2, b1, b2):
    return min(a2, b2) - max(a1, b1)


@click.command()
@click.option("--layer", default="Metal4", show_default=True)
@click.option("--power-net", default="VPWR", show_default=True)
@click.option("--ground-net", default="VGND", show_default=True)
@click.option("--clearance", type=float, default=0.3, show_default=True,
              help="um kept between a stripe and other-net metal of a macro")
@click.option("--stripe-clearance", type=float, default=1.0, show_default=True,
              help="um kept between two stripes after a move")
@click.option("--max-nudge", type=float, default=15.0, show_default=True)
@click.option("--max-halo", type=float, default=15.0, show_default=True,
              help="a stripe gap may extend this far beyond the macro outline")
@click.option("--pins-only-master", "pins_only", multiple=True,
              default=["RM_IHPSG13_1P_", "RM_IHPSG13_2P_"], show_default=True,
              help="master-name prefixes whose OBS on --layer is ignored")
@click.option("--no-prefer-connect", is_flag=True, default=False)
@click.option("--report", default=None, help="JSON report path")
@click.option("--check-grid-dir", default=None,
              help="re-run check_power_grid and write <net>-grid-errors.rpt here (+ metrics)")
@click_odb
def main(reader, layer, power_net, ground_net, clearance, stripe_clearance,
         max_nudge, max_halo, pins_only, no_prefer_connect, report, check_grid_dir):
    block = reader.block
    dbu = block.getDbUnitsPerMicron()
    L = reader.tech.findLayer(layer)
    if L is None:
        print(f"[ERROR] layer {layer} not found", file=sys.stderr)
        sys.exit(1)
    um = lambda v: v / dbu
    to_dbu = lambda v: int(round(v * dbu))
    grid = reader.tech.getManufacturingGrid() or 1
    clr = to_dbu(clearance)
    sclr = to_dbu(stripe_clearance)
    nets = {power_net: block.findNet(power_net), ground_net: block.findNet(ground_net)}
    other = {power_net: ground_net, ground_net: power_net}
    for n, obj in nets.items():
        if obj is None:
            print(f"[ERROR] net {n} not found", file=sys.stderr)
            sys.exit(1)

    # ------------------------------------------------------------ macros
    macros = []
    for inst in block.getInsts():
        m = inst.getMaster()
        if not m.isBlock() or not inst.isFixed():
            continue
        pins = []  # (net name, rect) on L
        for it in inst.getITerms():
            mt = it.getMTerm()
            if mt.getSigType() not in ("POWER", "GROUND"):
                continue
            net = it.getNet()
            name = net.getName() if net is not None else (
                power_net if mt.getSigType() == "POWER" else ground_net)
            for mp in mt.getMPins():
                for g in mp.getGeometry():
                    if g.getTechLayer() is not None and g.getTechLayer().getName() == layer:
                        pins.append((name, transformed(inst, g)))
        obs = []
        # RM_IHPSG13_1P_8192x32_c4 is a composite with real Metal4 besides its
        # supply columns (checked against the GDS), so its OBS always counts.
        trusted = any(m.getName().startswith(p) for p in pins_only) and "8192x32" not in m.getName()
        if not trusted:
            for g in m.getObstructions():
                if g.getTechLayer() is not None and g.getTechLayer().getName() == layer:
                    obs.append(transformed(inst, g))
        macros.append({"inst": inst, "name": inst.getName(), "master": m.getName(),
                       "bbox": rect_of(inst.getBBox()), "pins": pins, "obs": obs})
    print(f"[INFO] {len(macros)} placed macro(s): " +
          ", ".join(f"{d['name']} ({d['master']})" for d in macros))
    if not macros:
        return

    # ------------------------------------------------------------ stripes
    columns = {}  # (net, 2*xc, w) -> {"segs": [sbox], "vias": [sbox], "swire": swire}
    for name, net in nets.items():
        for sw in net.getSWires():
            for sb in sw.getWires():
                if sb.isVia() or sb.getTechLayer() is None or sb.getTechLayer().getName() != layer:
                    continue
                if sb.getDY() <= sb.getDX():
                    continue  # vertical stripes only
                xc2 = sb.xMin() + sb.xMax()  # 2 * centre, exact
                key = (name, xc2, sb.getDX())
                columns.setdefault(key, {"segs": [], "vias": [], "swire": sw})
                columns[key]["segs"].append(sb)
    if not columns:
        print("[WARNING] no vertical stripes on the layer, nothing to do")
        return
    ref_lo = min(sb.yMin() for c in columns.values() for sb in c["segs"])
    ref_hi = max(sb.yMax() for c in columns.values() for sb in c["segs"])
    # vias of each column (any via whose centre is on the column centre line)
    for name, net in nets.items():
        for sw in net.getSWires():
            for sb in sw.getWires():
                if not sb.isVia():
                    continue
                xc2 = sb.xMin() + sb.xMax()
                for (n, cx2, w), col in columns.items():
                    if n == name and cx2 == xc2:
                        col["vias"].append(sb)
    # Pin rectangles.  pdngen makes ONE dbBPin per supply net holding a box per
    # stripe, so the pins are snapshotted here and rebuilt once at the end
    # (destroying that BPin per column would drop every other stripe's pin).
    pin_boxes = {}  # net -> list of (layer, x1, y1, x2, y2, status)
    for name in nets:
        bt = block.findBTerm(name)
        if bt is None:
            continue
        pin_boxes[name] = []
        for bp in bt.getBPins():
            st = str(bp.getPlacementStatus())
            for b in bp.getBoxes():
                pin_boxes[name].append((b.getTechLayer(), b.xMin(), b.yMin(), b.xMax(), b.yMax(), st))
    changed = {}  # (net, old xc2) -> new (x1, x2) or None (removed)

    # ------------------------------------------------------------ analysis
    def gaps(col):
        segs = sorted((sb.yMin(), sb.yMax()) for sb in col["segs"])
        out, cur = [], ref_lo
        for lo, hi in segs:
            if lo > cur:
                out.append((cur, lo))
            cur = max(cur, hi)
        if cur < ref_hi:
            out.append((cur, ref_hi))
        return out

    halo = to_dbu(max_halo)

    def crossing_macros(x1, x2):
        return [d for d in macros if overlap_1d(x1, x2, d["bbox"][0], d["bbox"][2]) > 0]

    def check(net, x1, x2, colkey=None, positions=None, full=False):
        """(legal, n_connected, reason) for a stripe [x1,x2] over the full span.
        n_connected counts own-net macro pin rectangles overlapped by at least
        half the narrower width (full=True: the stripe lies entirely inside)."""
        n_conn = 0
        for d in crossing_macros(x1 - clr, x2 + clr):
            for (pn, (a, b, c, e)) in d["pins"]:
                if pn == net:
                    need = (x2 - x1) if full else min(x2 - x1, c - a) / 2
                    if overlap_1d(x1, x2, a, c) >= need:
                        n_conn += 1
                    continue
                if overlap_1d(x1 - clr, x2 + clr, a, c) > 0:
                    return False, 0, f"{um(clr)} um clearance to {pn} pin of {d['name']} at x {um(a):.3f}..{um(c):.3f}"
            for (a, b, c, e) in d["obs"]:
                if overlap_1d(x1 - clr, x2 + clr, a, c) > 0 and overlap_1d(ref_lo, ref_hi, b, e) > 0:
                    return False, 0, f"OBS of {d['name']}"
        if positions is not None:
            for k, (p1, p2) in positions.items():
                if k == colkey:
                    continue
                if overlap_1d(x1 - sclr, x2 + sclr, p1, p2) > 0:
                    return False, 0, f"too close to stripe {k[0]} at x {um((p1 + p2) / 2):.3f}"
        return True, n_conn, ""

    positions = {k: (k[1] // 2 - k[2] // 2, k[1] // 2 - k[2] // 2 + k[2]) for k in columns}
    core = block.getCoreArea()
    report_rows = []
    n_ext = n_moved = n_removed = 0
    for key in sorted(columns, key=lambda k: k[1]):
        net, xc2, w = key
        col = columns[key]
        g = gaps(col)
        if not g:
            continue
        # every gap must be explained by a macro this column crosses
        x1, x2 = positions[key]
        crossed = crossing_macros(x1, x2)
        explained = all(any(gl >= d["bbox"][1] - halo and gh <= d["bbox"][3] + halo
                            for d in crossed) for gl, gh in g)
        if not crossed or not explained:
            print(f"[WARNING] {net} stripe at x={um(xc2 / 2):.3f} has gaps not explained by a macro, left as is")
            continue
        legal, nconn, why = check(net, x1, x2, key, positions)
        dx = 0
        inside = lambda d: check(net, x1 + d, x2 + d, key, positions, full=True)[1] > 0
        if not legal or (not no_prefer_connect and not inside(0)):
            # candidates by increasing |dx|: first one fully inside a column of
            # its own net, else first one overlapping one by half, else first legal
            best = {"full": None, "half": None, "any": None}
            for i in range(0, int(to_dbu(max_nudge) / grid) + 1):
                for d in ((0,) if i == 0 else (i * grid, -i * grid)):
                    if x1 + d < core.xMin() or x2 + d > core.xMax():
                        continue
                    ok, nc, _ = check(net, x1 + d, x2 + d, key, positions)
                    if not ok:
                        continue
                    if best["any"] is None:
                        best["any"] = d
                    if nc > 0 and best["half"] is None:
                        best["half"] = d
                    if nc > 0 and inside(d):
                        best["full"] = d
                        break
                if best["full"] is not None:
                    break
            if no_prefer_connect:
                dx = 0 if legal else best["any"]
            else:
                dx = next((best[k] for k in ("full", "half", "any") if best[k] is not None), None)
        if dx is None:
            print(f"[WARNING] {net} stripe at x={um(xc2 / 2):.3f} cannot cross the macros ({why}); removing the whole stripe")
            for sb in col["segs"] + col["vias"]:
                odb.dbSBox_destroy(sb)
            changed[(net, xc2)] = None
            del positions[key]
            n_removed += 1
            report_rows.append({"net": net, "x": um(xc2 / 2), "action": "removed", "reason": why})
            continue
        if dx:
            for sb in col["vias"]:
                # via sboxes are re-created at the shifted location
                vx, vy = sb.getViaXY()
                sw = sb.getSWire()
                tv, bv = sb.getTechVia(), sb.getBlockVia()
                shape = sb.getWireShapeType()
                odb.dbSBox_destroy(sb)
                if tv is not None:
                    odb.dbSBox_create(sw, tv, vx + dx, vy, shape)
                else:
                    odb.dbSBox_create(sw, bv, vx + dx, vy, shape)
            n_moved += 1
        nx1, nx2 = x1 + dx, x2 + dx
        positions[key] = (nx1, nx2)
        sw = col["swire"]
        for sb in col["segs"]:
            odb.dbSBox_destroy(sb)
        odb.dbSBox_create(sw, L, nx1, ref_lo, nx2, ref_hi, "STRIPE")
        changed[(net, xc2)] = (nx1, nx2)
        _, nconn, _ = check(net, nx1, nx2)
        n_ext += 1
        names = ",".join(d["name"] for d in crossing_macros(nx1, nx2))
        msg = (f"[INFO] {net} stripe x={um(xc2 / 2):.3f}" +
               (f" moved {um(dx):+.3f} um to x={um((nx1 + nx2) / 2):.3f}" if dx else "") +
               f": extended over {names}, on {nconn} {net} macro column(s)")
        print(msg)
        report_rows.append({"net": net, "x": um(xc2 / 2), "dx": um(dx), "x_new": um((nx1 + nx2) / 2),
                            "macros": names, "connected_columns": nconn, "action": "extended"})

    # rebuild the supply pins: unchanged boxes are kept, the boxes of a changed
    # stripe are replaced by one full-height box (none if it was removed)
    for name, boxes in pin_boxes.items():
        if not any(k[0] == name for k in changed):
            continue
        bt = block.findBTerm(name)
        keep, status = [], "FIRM"
        for (lay, a, b, c, e, st) in boxes:
            status = st
            if lay is not None and lay.getName() == layer and (name, a + c) in changed:
                continue
            keep.append((lay, a, b, c, e))
        for (n, xc2), new in changed.items():
            if n == name and new is not None:
                keep.append((L, new[0], ref_lo, new[1], ref_hi))
        for bp in list(bt.getBPins()):
            odb.dbBPin_destroy(bp)
        bp = odb.dbBPin_create(bt)
        for (lay, a, b, c, e) in keep:
            odb.dbBox_create(bp, lay, a, b, c, e)
        bp.setPlacementStatus(status)
        print(f"[INFO] {name}: pin rebuilt with {len(keep)} box(es)")

    # per-macro supply summary
    summary = {}
    for d in macros:
        fed = {}
        for (pn, (a, b, c, e)) in d["pins"]:
            hit = any(k[0] == pn and overlap_1d(p1, p2, a, c) > 0 for k, (p1, p2) in positions.items())
            fed.setdefault(pn, [0, 0])
            fed[pn][1] += 1
            if hit:
                fed[pn][0] += 1
        summary[d["name"]] = {n: f"{v[0]}/{v[1]} pin rectangles under a stripe" for n, v in fed.items()}
        for n, v in fed.items():
            print(f"[INFO] {d['name']}: {n}: {v[0]} of {v[1]} Metal4 supply pin rectangles have a stripe")
            if v[0] == 0:
                print(f"[WARNING] {d['name']}: no {n} supply column is fed by a stripe")
    print(f"[INFO] stripes extended: {n_ext} (moved: {n_moved}), removed: {n_removed}")
    if report:
        with open(report, "w") as f:
            json.dump({"stripes": report_rows, "macros": summary}, f, indent=2)
    if check_grid_dir:
        check_grid(reader, [power_net, ground_net], check_grid_dir)


def check_grid(reader, net_names, out_dir):
    """Re-run OpenROAD's power grid connectivity check on the modified grid and
    publish the same metrics as OpenROAD.GeneratePDN (pdngen's own check ran
    before the stripes were extended)."""
    import os
    total = 0
    for net in net_names:
        rpt = os.path.join(out_dir, f"{net}-grid-errors.rpt")
        with open(rpt, "w") as f:
            f.write("")
        try:
            reader.design.evalTclString(f"check_power_grid -net {net} -error_file {rpt}")
        except Exception as e:  # check_power_grid raises on violations
            print(f"[WARNING] Grid check for {net} failed: {e}")
        count = sum(1 for line in open(rpt) if line.startswith("violation type:"))
        print(f"[INFO] power grid check {net}: {count} violation(s) after stripe extension")
        print(f"%OL_METRIC_I design__power_grid_violation__count__net:{net} {count}")
        total += count
    print(f"%OL_METRIC_I design__power_grid_violation__count {total}")


if __name__ == "__main__":
    main()
