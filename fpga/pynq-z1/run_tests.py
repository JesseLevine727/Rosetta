"""Run the toolchain tests on one backend and write per-test results as JSON.

    ROSETTA_BACKEND=board ROSETTA_TRANSCRIPT=dir python3 run_tests.py <repo> <pattern> <out.json>

Used by regress.py; the backend and transcript directory come from the
environment exactly as for `python3 -m unittest discover -s toolchain/tests`.
"""
import json
import os
import sys
import time
import traceback
import unittest


class _Result(unittest.TestResult):
    def __init__(self):
        super().__init__()
        self.rows = []
        self._t0 = {}

    def startTest(self, test):
        self._t0[test.id()] = time.time()
        super().startTest(test)

    def _row(self, test, status, detail=""):
        self.rows.append({"id": test.id(), "status": status,
                          "seconds": round(time.time() - self._t0.get(test.id(), time.time()), 3),
                          "detail": detail[-6000:]})
        print(f"{status:7s} {test.id()}", flush=True)

    def addSuccess(self, test):
        super().addSuccess(test)
        self._row(test, "pass")

    def addFailure(self, test, err):
        super().addFailure(test, err)
        self._row(test, "fail", "".join(traceback.format_exception(*err)))

    def addError(self, test, err):
        super().addError(test, err)
        self._row(test, "error", "".join(traceback.format_exception(*err)))

    def addSkip(self, test, reason):
        super().addSkip(test, reason)
        self._row(test, "skip", reason)

    def addExpectedFailure(self, test, err):
        super().addExpectedFailure(test, err)
        self._row(test, "xfail")

    def addUnexpectedSuccess(self, test):
        super().addUnexpectedSuccess(test)
        self._row(test, "fail", "unexpected success")


def main():
    repo, pattern, out = sys.argv[1:4]
    tests = os.path.join(repo, "toolchain", "tests")
    sys.path.insert(0, tests)
    t0 = time.time()
    suite = unittest.TestLoader().discover(tests, pattern=pattern, top_level_dir=tests)
    res = _Result()
    suite.run(res)
    doc = {"backend": os.environ.get("ROSETTA_BACKEND", "verilator"), "pattern": pattern,
           "seconds": round(time.time() - t0, 1), "tests": res.rows,
           "counts": {s: sum(1 for r in res.rows if r["status"] == s)
                      for s in ("pass", "fail", "error", "skip")}}
    with open(out, "w") as f:
        json.dump(doc, f, indent=1)
    bad = doc["counts"]["fail"] + doc["counts"]["error"]
    print(f"{len(res.rows)} tests: {doc['counts']}  ({doc['seconds']} s)")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
