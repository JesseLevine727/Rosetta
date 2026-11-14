"""Imported first by every test module: puts toolchain/ on sys.path so that
`python3 -m unittest discover -s toolchain/tests` finds the `rosetta` package."""
import os
import sys

TOOLCHAIN = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if TOOLCHAIN not in sys.path:
    sys.path.insert(0, TOOLCHAIN)
FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")

README = os.path.join(TOOLCHAIN, "README.md")


def readme_programs() -> dict[str, str]:
    """The ```asm blocks of toolchain/README.md marked <!-- program: NAME -->."""
    import re
    with open(README, encoding="utf-8") as f:
        text = f.read()
    return {m.group(1): m.group(2) for m in re.finditer(
        r"<!-- program: (\w+) -->\s*```asm\n(.*?)```", text, re.S)}
