# Formal verification

Every RTL block gets formal proofs before it is integrated, and every
integration step gets its own proofs. Simulation exists only where the Tiny
Tapeout flow requires it (`test/`, gate-level smoke test).

Layout (one directory per block, mirroring `src/`):

    formal/<block>/<block>.sby      SymbiYosys job: prove (unbounded) + cover
    formal/<block>/props_<block>.sv properties, bound to the RTL
    formal/eqy/                     equivalence checks (RTL vs synthesized netlist)

Rules:

- **Prove, don't just bound.** Each job has a `prove` task (k-induction or
  PDR) for unbounded proofs. A bounded `bmc` result counts only when its
  depth is justified in the job file.
- **Every assumption has a cover.** For each block, cover properties show the
  interesting states are reachable under the assumptions, so a proof can't
  pass by having nothing to check.
- **Proof status is tracked in this README** once blocks exist.
