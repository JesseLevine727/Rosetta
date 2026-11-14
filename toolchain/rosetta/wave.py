"""ASCII waveforms of model traces, for documentation and expect-style tests.

    print(render({"tx": trace.out(0), "sck": trace.out(3)}, scale=10))

Each signal is a sequence of per-cycle values: 0/1 (ints or '0'/'1'),
'z' (released) or 'x' (unknown), or for a bus any other int/str value.
One text column covers `scale` cycles and shows the value at the first
cycle of the column. Characters:

    _  low      -  high      ~  released (z)      x  unknown
    a bus prints its value at each change: |48......|69......

`elide` (optional) shortens every stretch where no signal changes for more
than `elide` columns to `elide` columns with a `//` mark, and the ruler
line shows the cycle number at each change.
"""
from __future__ import annotations

from typing import Sequence, Union

Value = Union[int, str]
_CHARS = {"0": "_", "1": "-", "z": "~", "x": "x"}


def _norm(v: Value) -> str:
    return str(v)


def _is_bit(vals: Sequence[str]) -> bool:
    return all(v in _CHARS for v in vals)


def _row(vals: Sequence[str]) -> str:
    if _is_bit(vals):
        return "".join(_CHARS[v] for v in vals)
    out, prev, label, k = [], None, "", 0
    for v in vals:                          # bus: "|value" at each change, one char per column
        if v != prev:
            label, k, prev = "|" + v, 0, v
        out.append(label[k] if k < len(label) else ".")
        k += 1
    return "".join(out)


def columns(signals: dict[str, Sequence[Value]], start: int = 0, end: int | None = None,
            scale: int = 1) -> tuple[list[int], dict[str, list[str]]]:
    n = max((len(s) for s in signals.values()), default=0)
    end = n if end is None else min(end, n)
    cycles = list(range(start, end, max(1, scale)))
    cols = {name: [_norm(s[c]) if c < len(s) else "x" for c in cycles]
            for name, s in signals.items()}
    return cycles, cols


def render(signals: dict[str, Sequence[Value]], start: int = 0, end: int | None = None,
           scale: int = 1, elide: int | None = None, ruler: bool = True) -> str:
    cycles, cols = columns(signals, start, end, scale)
    if elide is not None and cycles:
        keep: list[int] = []
        run = 0
        for i in range(len(cycles)):
            same = i > 0 and all(c[i] == c[i - 1] for c in cols.values())
            run = run + 1 if same else 0
            if run < elide:
                keep.append(i)
            elif run == elide:
                keep.append(-1)              # elision mark
        cycles = [cycles[i] if i >= 0 else -1 for i in keep]
        cols = {k: [v[i] if i >= 0 else "//" for i in keep] for k, v in cols.items()}
    width = max((len(k) for k in signals), default=0)
    lines = []
    if ruler:
        lines.append(" " * width + "  " + _ruler(cycles, cols))
    for name, vals in cols.items():
        if any(v == "//" for v in vals):
            parts = [vals[i] for i in range(len(vals))]
            row = "".join("/" if p == "//" else _row([p]) if _is_bit([p]) else "." for p in parts) \
                if _is_bit([p for p in parts if p != "//"]) else _bus_with_marks(parts)
        else:
            row = _row(vals)
        lines.append(f"{name:<{width}}  {row}".rstrip())
    return "\n".join(lines)


def _bus_with_marks(parts: list[str]) -> str:
    row = list(_row([p if p != "//" else "\0" for p in parts]))
    return "".join("/" if p == "//" else c for p, c in zip(parts, row))


def _ruler(cycles: list[int], cols: dict[str, list[str]]) -> str:
    """Cycle numbers at the columns where some signal changes."""
    line = [" "] * len(cycles)
    pos = 0
    for i, c in enumerate(cycles):
        if c < 0:
            line[i] = "/"
            continue
        changed = i == 0 or cycles[i - 1] < 0 or any(v[i] != v[i - 1] for v in cols.values())
        if changed and i >= pos:
            label = str(c)
            for j, ch in enumerate(label):
                if i + j < len(line):
                    line[i + j] = ch
            pos = i + len(label) + 1
    return "".join(line).rstrip()


def edges(values: Sequence[Value]) -> list[tuple[int, str, str]]:
    """(cycle, old, new) for every change of a per-cycle signal."""
    out = []
    for i in range(1, len(values)):
        a, b = _norm(values[i - 1]), _norm(values[i])
        if a != b:
            out.append((i, a, b))
    return out
