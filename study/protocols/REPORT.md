# Protocol feasibility and gap study: low-speed USB, 10 Mbit Ethernet, CAN

Study of 21 Nov 2026, against the RTL in `src/` as of commit `4590881` (RTL content hash `09fd49a09d1f3782`, unchanged during the study). Scope after the owner's change of the same day: low-speed USB, 10BASE-T Ethernet and CAN. JTAG, SWD and PS/2 are dropped (see the appendix).

Every timing claim marked **[RTL]** was measured by simulating the whole chip top (`tt_um_jesselevine727_rosetta`: all four lanes, sequencer, host port, crossbar) under Verilator. The programs were written by hand and loaded through the host SPI port. **[model]** means a Python model of the arithmetic, validated against the RTL where noted. **[spec]** means read from the block specs and not simulated.

Standards figures carry the confidence a separate research pass gave them:
- P = read in the primary specification;
- S = a secondary source citing the clause;
- Part = partly verified;
- NV = not verified.

Sources are listed in section 7.

## 1. Summary

| Protocol | Role | Verdict | Rate / limit | Hardware change needed | External parts (3.3 V Pmod) |
|---|---|---|---|---|---|
| Low-speed USB | device | **WORKS AS IS** [RTL] | 1.5 Mbit/s, host clock ±1.5 %; turnaround 2.05–3.45 bit times measured (limit 2 to 7.5); 112 program words | no | D− 1.5 kΩ pull-up to 3.3 V, 27–68 Ω series resistors, ESD diodes; edge slowing optional (fast edges violate the 75 ns minimum rise time) |
| Low-speed USB | listener | **WORKS WITH LIMITS** [RTL] | every packet of both directions captured with timestamps; sustained about 38 k records/s to the host; bursts up to the 64-word queue | no | same pins, inputs only |
| 10BASE-T | transmit (with link pulses) | **WORKS WITH LIMITS** [RTL] | exact 10 Mbit/s, CRC-32, TP_IDL, NLP every 15.97 ms; frame at most 480 bytes pre-staged (about 670 with host streaming); never 1518 | no | RJ45 with 1:1 magnetics; a line driver (pad current into 100 Ω not verified); series and termination resistors; no waveshaping, so not template-compliant |
| 10BASE-T | receive / listener | **WORKS WITH LIMITS** [RTL] | low-jitter partners only: DCD up to 16.5 ns, alternating per-edge jitter up to 8.2 ns, Gaussian σ up to 3 ns. **Cannot meet the 802.3 receiver tolerance (±13.5 ns) at any sampling rate with the present decoder.** Frame at most 448 bytes buffered (about 590 with concurrent drain); sustained load at most about 24 % | **yes, for compliance**: Manchester phase-tracking decoder (H1), about 2.5–3.5 k µm² per lane equipped *(estimate)* | magnetics; fast differential comparator with about 300–585 mV threshold (squelch) and hysteresis; 100 Ω termination |
| CAN | full node: transmit with arbitration, receive, ACK, same-frame role switch | **WORKS WITH LIMITS** [RTL] | two lanes per node; two Rosetta nodes arbitrated correctly at 250 k, 500 k and 1 Mbit/s with 133–250 ns loop delay; against standard controllers, SOF alignment must be tuned in software (fine to 500 kbit/s; at 1 Mbit/s, starts after a long idle are marginal) | no; H3 (tiny) makes 1 Mbit/s idle-start joins robust | 3.3 V transceiver (SN65HVD230, TCAN334, MCP2562 with VIO 3.3 V); two chip pins tied to TXD (wired-AND with the transceiver's TXD pull-up); 120 Ω termination |
| CAN | full node in **one** lane | **CANNOT** today | | single-lane CAN mode (H5), not recommended | |
| CAN | listen-only node | **WORKS AS IS** [RTL] | one lane (receiver without the ACK drive) | no | transceiver |
| CAN | ACK-only node | **WORKS AS IS** [RTL] | one lane; acknowledged an external node's frame in simulation | no | transceiver |

**The three key findings**

1. **Ethernet receive margin** [RTL, 7,605 frames].
 - The interval classifier tolerates an interval error of ±16.7 ns, one sampling cycle: shorts must measure at most 4 cycles and longs at least 5.
 - Measured limits:
   - duty-cycle distortion passes at 16.5 ns and fails at 17.0 ns;
   - alternating per-edge jitter passes at 8.2 ns and fails at 8.5 ns;
   - independent uniform jitter passes at 8 ns and fails 5 of 200 frames at 9 ns.
 - 802.3 requires a receiver to accept ±13.5 ns of zero-crossing jitter, that is narrow pulses of 23–77 ns and wide pulses of 73–127 ns (S). Those windows overlap (77 > 73), so **no interval classifier at any resolution can be compliant**; only a phase-tracking decoder can.
 - Modelled: a duty-cycle-compensated DPLL at 60 MHz tolerates 33 ns of DCD and 16.5 ns of alternating jitter, but only 11–14.5 ns of random jitter.
 - Adding sampling on both clock edges brings random jitter to 17 ns. Both edges alone, keeping the classifier, gain nothing.
2. **USB turnaround** [RTL; 12 runs × 3 transactions; host clock at 0 and ±1.5 %].
 - Measured:
   - IN → NAK: 132–138 cycles (3.30–3.45 bit times);
   - IN → DATA1: 132–137 cycles (3.30–3.43);
   - DATA0 after SETUP → ACK: 82–86 cycles (2.05–2.15).
 - Limits (USB 2.0 7.1.18.1, Table 7-14, P): at least 2 bit times; at most 7.5 bit times at the A plug, because low-speed devices have captive cables (6.5 at a B receptacle).
 - Spare: about 4 bit times on the slowest path (160 cycles, 32 instruction slots).
 - The 2-bit minimum is met by a programmed `PIN` wait: 2–6 cycles spare measured, 1 in the best-case slot alignment by analysis. One more wait unit would widen it.
3. **CAN arbitration: yes, with two lanes per node** [RTL].
 - The transmitter lane runs `BAUD` timing with an event: "bus dominant while my TXD is recessive, for K cycles". The event aborts the lane inside the losing bit. Outputs keep their level on abort, and that level is recessive.
 - The receiver lane receives every frame all the time in `RECOVER`, so it simply keeps the frame and acknowledges it.
 - Two Rosetta nodes started together:
   - the loser's TXD was dominant for exactly its SOF and the two equal identifier bits (3 bits), and never again in that frame;
   - its receiver lane took and acknowledged the winner's frame;
   - it then resent its own frame and was acknowledged.
 - **One lane cannot do it**:
   - receive de-stuffing is not available in `BAUD` (ruling 24);
   - `EXCHANGE` stuffs the transmitted stream;
   - `RECOVER` would resynchronise on the node's own echoed edges.

What could not be determined is listed in section 8: electrical compliance of the transmit waveforms, pad drive, behaviour against real equipment, and a few 802.3 numbers.

## 2. Ranked hardware changes

Area headroom is about 22,000 µm² (503,000 used; routing fails above about 525,000). Sizes are estimates from the library cell areas in `docs/area-budget.md` (flop with enable 67 µm², combinational cell about 9 µm²). None was synthesised.

**H1. Manchester phase-tracking receive decoder** (rank 1)
- **Change:** a new decode for `CODE.rx = MANCHESTER` under `RECOVER` timing; today that combination decodes as plain level.
- The `RECOVER` counter gains about 6 fractional bits and becomes a phase estimate of the mid-bit instant (`PERIOD` = bit time).
- A data edge within ±P/4 of the estimate is a mid-bit edge: bit = new level, one `st` with `rxv`. Other edges are boundary edges.
- The phase is updated by err/64 after a short high-gain acquisition on the preamble.
- A separate rising/falling offset (about 6 bits) cancels duty-cycle distortion.
- Bits reach `rosetta_field` exactly as today's Manchester bits do.
- **Enables:** Ethernet receive against compliant partners: DCD 33 ns and deterministic jitter 16.5 ns per edge (requirement 13.5); random jitter 11–14.5 ns (marginal). Also any Manchester rate set by `PERIOD`.
- **Size:** about 20 flops + 150–250 cells, 2.5–3.5 k µm² per lane. One lane about 3 k µm²; all four lanes 10–14 k µm² (2–3 % of the die, half the headroom).
- **Blocks and proofs:** `rosetta_coding`, `rosetta_tick` (RECOVER), the lane mode-register decode. Rerun the coding, tick, lane and top proofs, and add a round-trip proof (encode, jitter bound, decode).
- **If not made:** Ethernet receive stays "works with low-jitter partners only"; no compliance claim is possible.

**H3. Edge-terminated `RAW DRIVE`** (rank 2)
- **Change:** one mode bit (for example `CODE[12]`) makes a `RAW DRIVE` also end at the first data-input edge. Used as "wait N cycles or until another node's SOF edge, whichever comes first", after which the SOF field launches.
- **Enables:** a CAN transmitter that joins a competing SOF within about 5 cycles plus the loop delay, as a standard controller's hard sync does. Robust arbitration after bus idle at 1 Mbit/s.
- **Size:** 1 flop + 3–5 cells, about 100 µm² per lane (0.4 k µm² for four).
- **Blocks and proofs:** `rosetta_field` (RAW DRIVE completion) and lane cfg; rerun the field, lane and top proofs.
- **If not made:** joining by an event handler costs 15–25 cycles. Idle-start collisions are fine to 500 kbit/s and marginal at 1 Mbit/s.

**H4. `RECOVER` options** (rank 3)
- **Change:** resynchronise on one edge direction only (for CAN, recessive-to-dominant), and capture `PERIOD2` cycles after the launch tick instead of `PERIOD/2` (a programmable sample point).
- **Enables:** CAN-correct receiver timing (R-to-D resync only, a 75–87.5 % sample point, immune to the slower D-to-R transceiver edge). Also helps UART and LIN receivers.
- **Size:** 2 flops + about 30 cells (a 16-bit mux on H), about 0.4 k µm² per lane.
- **Blocks and proofs:** `rosetta_tick`; rerun the tick, lane and top proofs.
- **If not made:** receivers resync on both edges at 50 %. That worked in simulation. On a real bus the asymmetric transceiver delay (about 50 ns, TJA1050 typical) shifts the sample point by about 3 cycles, harmless at 500 k.

**H2. Sampling the data input on both clock edges** (rank 4)
- **Change:** a negedge first flop and a re-timing flop on the protocol input pins, one extra crossbar plane for each lane's `in0`, and a half-cycle bit in the interval and phase arithmetic.
- **Enables:** useful only with H1, where random jitter goes from 11–14.5 ns to about 17 ns (requirement 13.5). With today's classifier it gains nothing for symmetric jitter.
- **Size:** 1.5 k µm² (ui[3..7] only) to 3 k µm² (all 16 pins).
- **Blocks and proofs:** `rosetta_xbar`, tick and coding. The proofs are single-clock today, so negedge flops need multi-edge formal handling. The result depends on the RP2350 clock's duty cycle.
- **If not made:** the DPLL alone is marginal on random jitter.

**Not recommended:** a faster chip clock; a ring-oscillator receive front end; a delay-tap time-to-digital front end; a single-lane CAN mode (H5). See 4.5 and 6.5.

A coherent package for the three protocols is **H1 in one or two lanes, with H3 and H4 in all lanes: about 4–8 k µm² (0.8–1.6 % of the die)**. H1 in all four lanes plus H3 and H4 is about 14–18 k µm², most of the headroom.

**Work that is not hardware (compiler, language, programs)**

- **USB:**
- transactions expressed as several frames with a party argument;
- runtime parameters (the device address) and per-endpoint mailboxes;
- the NAK-if-not-staged default;
- bus-reset detection by a `WAIT` timeout after the SE0 event (event windows stop at K ≤ 63 cycles);
- keep-alive and suspend;
- more endpoints (a HID keyboard needs EP1 IN; about 10 words).
- **Ethernet:**
- `net` byte order and the trailing FCS (residue check, done here with `CRCCHK`);
- link pulses as a timed idle loop;
- the length of the last partial word (solved here with a marker bit, section 4.3);
- half-duplex carrier sense and collision handling (an event on the receive input in the transmit lane; not simulated);
- double buffering and the host drain.
- **CAN:**
- one role compiled into two cooperating lane programs joined by channels;
- arbitration, ACK-by-receiver and error-frame semantics in the language;
- error counters, error passive and bus-off (host or program);
- SOF timing tuned to the end of intermission.
- **Schedule rules the compiler must know**, all found while writing these programs:
- (a) `Q PUT` and `Q PUSH` do not wait for the current field. "After field X completes" needs a sync instruction first (`BR M/NM`, `MOVSR`).
- (b) An `EVT` placed after a parked field executes when that field is promoted, not when it completes.
- (c) A level-triggered event fires once per episode. Clearing its pending bit while the condition still holds loses it for good; this hung the first USB program.
- (d) `CRCCHK` needs the residue in `{R1, R0}`, so `EXPECT src=CRC` is the cheaper check when the CRC position is known.
- (e) `PIN` wait units are `PERIOD`, which is half a bit in `BAUD`.

## 3. Method and evidence

All files are in `study/protocols/sim/`; results are in `study/protocols/sim/results/`.

| File | What |
|---|---|
| `study_tb.cpp`, `studysim.py` | A copy of the repository Verilator harness and driver with three additions: per-cycle waveform injection on any input pin (tri-state, pulls, contention counting); a CAN bus model (wired-AND of two nodes' TXD nets and an external node, with transceiver delays); trace on/off. The repository harness and `rtlsim.py` are not used or changed. |
| `eth_rx.asm`, `eth_rx_margin.py`, `ethlib.py` | Ethernet receive program and margin campaign. The continuous-time Manchester generator applies a random phase per frame, a frequency offset, per-edge jitter and DCD, then samples at each clock edge exactly as the first synchroniser flop sees it. |
| `eth_options.py`, `eth_isi.py` | Model of decoder and front-end options; model of inter-symbol interference. |
| `eth_tx.asm`, `eth_tx_check.py` | Ethernet transmit with link pulses, and loopback into the receive lane. |
| `usb_dev.asm`, `usb_listen.asm`, `usb_turnaround.py` | USB device (lane 0) and listener (lane 1). The host model does NRZI, stuffing, CRC5/16 and EOP, with ±1.5 % clock and random phase. |
| `can_tx.asm`, `can_rx.asm`, `can_arbitration.py` | CAN transmitter and receiver lanes; two complete nodes (4 lanes) plus an external node on one bus. |

To rerun: `source /home/elfo/tools/oss-cad-suite-dl/oss-cad-suite/environment`, then `python3 <script>` in `study/protocols/sim`. Set `ROSETTA_STUDY_BUILD` to choose the build directory. Each script states its own usage. Wall time: about 6 minutes for the Ethernet margin campaign, under a minute for each other script.

**Simulation fidelity.**
- One sample per clock cycle. An input edge is seen at the first rising edge after it; a metastable sample resolves to one side, which the random phase covers.
- Pad delays (about 10 ns round trip) are constant offsets and are not modelled.
- Pad rise/fall asymmetry, comparator offset and transformer effects are not modelled, except through the DCD parameter.

## 4. 10 Mbit Ethernet (10BASE-T)

### 4.1 Roles

Transmit and receive: a station needs both, one lane each, half duplex. The listener is identical to receive. Link pulses belong to transmit.

### 4.2 Requirements (IEEE 802.3 clause 14; clause numbers as cited by the sources)

| Item | Value | Conf. |
|---|---|---|
| Bit rate | 10 Mbit/s, tolerance ±0.01 % (100 ppm) | NV (widely quoted, not found in a source read) |
| Coding | Manchester: first half is the complement, second half the value, so a 1 has a rising mid-bit edge; bytes go LSB first | S |
| Framing | preamble 7 × 0x55, SFD 0xD5, destination .. payload, FCS (CRC-32, polynomial 0x04C11DB7, initialised to ones, complemented, x^31 first); IFG 96 bit times = 9.6 µs | S |
| Receiver jitter tolerance | ±13.5 ns on zero crossings (14.3.1.3.1); as pulse widths: narrow 23–77 ns, wide 73–127 ns, at 585 mV–3.1 V | S |
| Transmitter jitter | ±5.5 ns through the twisted-pair model, ±8 ns directly into 100 Ω (14.3.1.2.3). Sources disagree; older ones give ±3.5 ns with the model | Part |
| TP_IDL | positive for at least 250 ns from the last positive transition, at most about 450 ns; back within 50 mV in 4.5 µs (14.3.1.2.1, figure 14-10) | Part |
| End-of-packet detection by a receiver | within 2.3 bit times (14.3.1.3.3) | S |
| Link pulses (NLP) | 100 ns nominal (60–200 ns), every 16 ± 8 ms, the first 8–24 ms after the last packet (14.2.1.1). Receiver link test: link_test_min 2–7 ms, link_test_max 25–150 ms, lc_max 2–10 pulses | S |
| Auto-negotiating PHYs | accept NLPs by parallel detection and bring up **10BASE-T half duplex** after lc_max pulses plus 0.5–1 s; NLPs must be sent continuously | S |
| Squelch | reject below 300 mV peak after the test filter; accept 585 mV–3.1 V | S |
| Transmit amplitude | 2.2–2.8 V peak differential into 100 Ω | S |
| Preamble bits a receiver may lose | no normative figure found (PHYs lose 1–3) | NV |

### 4.3 Receive: mapping and program (existing hardware)

Settings: `PULSE` timing, `CODE.rx = MANCHESTER`, `THR0 = 5`. Every edge of `in0` (the comparator output) is a sample. An interval of 5 cycles or more marks a mid-bit edge; two short ones toggle the phase (`coding.md` 3.2).

`eth_rx.asm` is 27 words. Abridged:

```
      SETM  TICK, mode=PULSE
      SETM  CODE, rx=MANCHESTER, data_in=in0
      SETM  THR01, thr0=5
      SETM  CRCPOLY, 0x1DB7 / CRCPOLY.hi, 0x04C1 / CRCCFG, width=32, init_ones=1, xorout_ones=1
      SETM  EVC0, a=in0, trigger=LEVEL, level=HIGH, k=12   ; TP_IDL ends the frame
      LDI   R2, 0xD555                                      ; last 16 bits of preamble + SFD
frame:  LDI   R3, 8 / LDI R0, 0xDD7B / LDI R1, 0xC704         ; buffer pointer, CRC residue
      EVT   0, eof, on, clear
      CRCINIT
      FIELD EXPECT 16, lsb, seek, src=R2                    ; hardware SFD seek
data:   FIELD SAMPLE 32, crc, src=IMM 1, dst=MEM              ; marker bit in SR
      BR    always, data
eof:    CRCCHK ; push R3, SR halves (partial word), CRC verdict ; BR frame
```

The marker `src=IMM 1` leaves `SR = (1 << k) | bits` after an abandoned field, so the host knows the exact length of the last partial word. Checked with frames of 64, 65, 67, 68 and 69 bytes [RTL].

**Cycle budget** [spec, confirmed on the RTL]:
- A field's successor is current one cycle after its completion, and mid-bit edges are at least 4–5 cycles apart.
- Each 32-bit field lasts 192 cycles and needs a delivery, a `BR` and a `FIELD`: 3 slots, 15 cycles.
- The TP_IDL event fires 12 + 3 cycles after the last rise (250 ns). K = 10 would meet the 2.3 bit-time end-of-packet figure; the longest data high time is 6–8 cycles.

### 4.4 Receive margin [RTL]

Test conditions: 64-byte frames (14 header + 46 payload + FCS), 200 frames per point. Each frame has a random phase against the clock and a frequency offset uniform in ±100 ppm. A frame counts only if every byte and the CRC verdict are right. (`results/eth_rx_margin.log`)

| Impairment | Last point with 0/200 failures | First failing point | Theory |
|---|---|---|---|
| Duty-cycle distortion (high pulses longer by D, low pulses shorter) | 16.5 ns | 17.0 ns: 42/200 | 16.67 ns |
| Alternating per-edge jitter +J/−J (every interval ±2J; the worst case for the classifier; equal to a DCD of 2J) | 8.2 ns | 8.5 ns: 33/200 | 8.33 ns |
| Independent uniform jitter in [−J, J] per edge | 8.0 ns | 9.0 ns: 5/200; 10 ns: 43/200 | 8.33 ns worst case |
| Gaussian jitter, σ per edge | 3.0 ns | 3.5 ns: 3/200; 4.0 ns: 13/200 | |

**Why these limits.** Shorts (50 ns) must measure at most 4 cycles and longs (100 ns) at least 5. With single-edge sampling, an interval I measures as floor or ceil of I/16.67 ns. So shorts are safe up to 66.7 ns and longs down to 83.3 ns: ±16.7 ns of interval error, or ±8.3 ns per edge when two edges move apart. The specs' note of "about half a cycle of margin" is exactly this.

**Against the requirement.** Narrow pulses up to 77 ns and wide pulses down to 73 ns must both decode. No threshold separates them, so no sampling resolution can make the interval classifier compliant. Against a partner transmitter at its own jitter limit (±8 ns into 100 Ω), the margin is about 0.3 ns.

**Band-limited channels** [model, `eth_isi.py`]. Through a single-pole low-pass, pattern-dependent shifts mostly push intervals away from the threshold: a short after a long gets shorter, a long after a short gets longer. At a time constant that gives ±10 ns of edge deviation from the ideal grid, the worst short grows only to 59.5 ns and the worst long shrinks only to 97.7 ns. So the classifier tolerates cable ISI far better than its per-edge figure suggests. What breaks it is DCD (comparator offset, rise/fall asymmetry, transformer baseline wander) and random jitter. In practice it works on a short cable to a normal NIC, with no compliance claim.

### 4.5 Receive options [model]

`eth_options.py` runs the same random frames through other front ends and decoders. Its model of today's 60 MHz single-edge classifier reproduces the RTL limits exactly (8.0 / 16.5 / 9.0 ns), which validates both the generator and the model.

Limits were found by scanning in 0.5 ns steps with 40 frames per step. One failure ends the scan, so the random-jitter figures carry about ±1 ns of noise. (`results/eth_options.log`, `results/eth_options_ro_calibrated.log`)

| Front end | Decoder | Alternating jitter per edge | DCD | Independent uniform jitter per edge |
|---|---|---|---|---|
| 60 MHz, single edge (today) | classifier | 8.0 | 16.5 | 9.0 |
| 60 MHz, single edge | DPLL, no DCD compensation | 8.0 | 16.5 | 14.5 |
| 60 MHz, single edge | **DPLL with DCD compensation (H1)** | **16.5** | **33.0** | **11.0–11.5** |
| 60 MHz, both edges, 50 % duty | classifier | 8.0 | 16.5 | 8.5 |
| 60 MHz, both edges, 45 % duty | classifier | 8.0 | 16.5 | 8.5 |
| 60 MHz, both edges, 50 % / 45 % duty | **DPLL with DCD compensation (H1 + H2)** | 16.5 | 33.5 / 33.0 | **17.0** |
| 66.7 MHz, single edge | classifier | 5.0 | 10.0 | 5.0 |
| 66.7 MHz, single edge | DPLL with DCD compensation | 17.5 | 35.0 | 13.5 |
| 3 equal phases (taps) | classifier | 11.0 | 22.0 | 11.5 |
| 4 equal phases (taps) | DPLL with DCD compensation | 18.5 | 37.5 | 18.5 |
| ring oscillator 200–400 MHz, calibrated | classifier | 10.5 | 21.0 | 11.0 |
| ring oscillator, calibrated | DPLL with DCD compensation | 18.5 | 36.0 | 19.0 |

The requirement is 13.5 ns per edge. The DPLL's 16.5 ns alternating limit is where 33 ns of DCD leaves a 17 ns pulse, about one cycle: the line itself sets that limit, not the decoder.

Notes on each option the coordinator asked about:

**(a) A faster chip clock.**
- At 66.7 MHz the classifier gets *worse* (5 / 10 ns), because the 75 ns decision point no longer falls midway between sample instants.
- It also makes 10 Mbit transmit impossible: 6.67 cycles per bit, and BAUD periods are integers.
- Timing closes at 60 MHz with under 3 ns to spare. Not a lever.

**(b) Both clock edges, and delay taps.**
- With the classifier, both edges give no gain for symmetric impairments. Half-cycle resolution moves only one side of the uncertain zone, because 75 ns = 9 half-cycles lies on the sampling grid. The zone becomes 66.7–75 or 75–83.3 ns, and the short side stays at 66.7 ns.
- With a DPLL, both edges add 3–6 ns on random jitter, because the quantisation term drops from 8.3 to 4.2 ns.
- A 45 % clock duty cycle cost nothing measurable in the model. But the half-cycle sample instant moves with the RP2350 clock's duty cycle and with pad asymmetry.
- Delay taps (4- or 8-phase): the equal-phase models above are optimistic, because cell delays vary about 2× with process, voltage and temperature. Taps would have to be either:
- calibrated: a chain longer than one clock period, with the clock itself sampled through it to count taps per period, so that phase = tap / taps-per-period (a DPLL can absorb that as its fractional unit); or
- used only to order an edge within its cycle.
- Synthesis, placement and hold fixing alter delay chains unless constrained. Each tap is an asynchronous sample, so the thermometer code gets bubbles. The formal flow cannot model the delays.
- Gain over H1+H2: about 1.5 ns. Not recommended.

**(c) Ring-oscillator front end.**
- Contents: a 7–15 stage ring at a few hundred MHz, clocking only:
- an edge detector;
- an interval counter or DPLL (about 10–20 flops);
- a calibration counter against the 60 MHz clock (about 10 flops);
- a toggle handshake or a 2–4 entry FIFO into the 60 MHz domain (about 6–10 flops).
- Size: about 40 flops, 3–5 k µm².
- Gain over H1 at 60 MHz: about +2 ns on alternating jitter and +5–8 ns on random jitter.
- Risks:
- a second, uncontrolled clock domain in a design whose proofs are single-clock (it needs CDC verification and an abstracted front end in formal);
- a combinational loop that STA must be told about (a generated clock on an internal pin, with the loop broken by `set_disable_timing`);
- frequency varies about 2× over corners;
- precheck acceptance of the loop is unknown (the competition allows mixed-signal circuits inside the design if precheck passes).
- Not recommended for this tape-out.

**(d) A PHY chip on the Pmod.**
- RMII (2 bits at 50 MHz on a 50 MHz reference clock) cannot be expressed: a lane cannot follow a 50 MHz clock, since the minimum feature is 3 cycles.
- MII (4 bits at 2.5 MHz, plus DV and ER) needs one lane per data bit, because a field samples one input. Receive alone would take all four lanes, with the host interleaving the bits. Not practical.
- A PHY with a 1-bit 10 Mbit serial mode (SNI, 10 MHz clock; DP83848 is an example, from memory, not verified) would fit one lane per direction in `EDGE` mode:
- receive at 3+3 cycles is plausible;
- transmit is marginal: the output follows the clock edge by 4–5 cycles plus pad delay, against a 100 ns period.
- In every case the chip would no longer do line coding or clock recovery, which the competition scores lower.

### 4.6 Transmit [RTL]

**Settings:** `BAUD`, `PERIOD = PERIOD2 = 3` (exactly 100 ns per bit at 60 MHz), `CODE.tx = MANCHESTER`, differential output (out0 = TD+, out1 = TD−), idle with both low.

**Program** (`eth_tx.asm`, 30 words):
- preamble and SFD as four 16-bit `DRIVE`s from registers;
- payload as `DRIVE 32, lsb, crc, src=MEM`;
- FCS as `DRIVE 32, src=CRC` (msb first);
- TP_IDL as `PIN out0=1, out1=0, wait=6`, then a `PIN` back to idle;
- between frames, an NLP (`PIN ... wait=2`, 100 ns) followed by 78 loops of `BR QNE` + `WAIT timeout=4095` (204.75 µs each).

**Measured** (`results/eth_tx_check.json`):
- edge intervals are only 3 and 6 cycles (50 / 100 ns exactly);
- 64 preamble+SFD bits;
- FCS equal to zlib's CRC-32 for frames of 64, 68 and 120 bytes;
- TD+ stays high 21 cycles (350 ns) after the last transition when the last bit is 1, and 18 cycles when it is 0;
- TD− is the exact complement;
- NLP width 6 cycles (100 ns). The period is 15.97 ms with the full loop count; simulated with a shortened loop, 2 iterations took 24,630 cycles, i.e. 12,285 cycles each plus overhead;
- loopback into the receive lane: every frame received with a good CRC.

**Cycle budget:** back-to-back `DRIVE`s are gap-free (`P2d = 3`, against the 2 cycles needed with source `CRC`). Each 32-bit field needs 2 slots of its 192 cycles.

**Not verified:**
- the 802.3 transmit template: the digital step has no waveshaping, and a transformer gives a backswing at the end of TP_IDL;
- whether the pads can source about 25 mA into 100 Ω; otherwise a line driver goes on the Pmod;
- pad rise/fall asymmetry, which adds a small DCD (about 1–2 ns).

The 50 ns minimum feature is well within the pads' roughly 66 MHz toggle ceiling.

### 4.7 Frame size and host bandwidth [spec + arithmetic]

The lane's data region is 128 words. The host SPI port gives about 90 k word writes/s and 76 k reads/s (`host.md` 2.1), against 312 k words/s on the wire.

| Case | Limit |
|---|---|
| Transmit, frame fully pre-staged | 120 words (an 8-word in-queue reserved) = 480 bytes + FCS |
| Transmit, host refilling behind the read pointer | about 120 / (1 − 0.29) = 169 words, about 670 bytes; needs the host to poll R3; not demonstrated |
| Receive, buffered | 112 words (8-word queues at both ends) = 448 bytes including FCS |
| Receive, host draining during the frame | about 147 words, about 590 bytes; not demonstrated |
| Full 1518-byte frame | impossible both ways: 380 words, and the host port runs at about 25–30 % of line rate |
| Back-to-back receive (9.6 µs gap) | only as far as the buffer holds; sustained receive at most about 300 kbyte/s (24 % load) |
| ARP (64 bytes), ICMP echo up to about 400 bytes of payload, small UDP | yes |

The receive program must also stop at the end of the buffer, because R3 wraps over the queues. A word counter costs 2 instructions.

### 4.8 Verdicts

- **Transmit: WORKS WITH LIMITS.**
- Exact timing, CRC, TP_IDL and link pulses are shown on the RTL.
- Frames up to about 480 bytes.
- The waveform template and the pad drive are unverified.
- Half-duplex collision handling is program work.
- **Receive / listener: WORKS WITH LIMITS** with the existing hardware.
- Low-jitter partners only: DCD ≤ 16.5 ns, alternating jitter ≤ 8.2 ns, Gaussian σ ≤ 3 ns.
- Frames up to 448 bytes.
- **NEEDS HARDWARE CHANGE (H1, ideally with H2) for any claim of 802.3 receiver tolerance.**

## 5. Low-speed USB

### 5.1 Roles

Device (the Rosetta chip as a keyboard-like function) and listener. A host role was not required and was not studied.

### 5.2 Requirements (USB 2.0, read in the primary text)

| Item | Value | Where |
|---|---|---|
| Bit rate | 1.5 Mbit/s ± 1.5 % for functions | 7.1.11, Table 7-10 |
| Device response delay | measured from the SE0-to-J transition at the end of the EOP to the J-to-K that starts the response: ≤ 6.5 bit times at a B receptacle, < 7.5 at the A plug of a captive cable. Low-speed devices must have captive cables, so the limit is **7.5** | 7.1.18.1, Table 7-14 |
| Minimum inter-packet delay | 2 bit times | Table 7-14 |
| Host timeout | between 16 and 18 bit times | 7.1.19.1 |
| Transmitted EOP | SE0 for 1.25–1.50 µs, then J for 1 bit | Table 7-10 |
| Received EOP | accept SE0 ≥ 670 ns; an SE0 shorter than 210 ns during a transition must not count | 7.1.4.1, Table 7-10 |
| Reset | SE0 ≥ 10 ms from the host; a device may act after 2.5 µs | 7.1.7.5 |
| Keep-alive / suspend | a low-speed EOP every 1 ms; suspend after 3 ms idle | 7.1.7.6 |
| Device receiver jitter | ±75 ns to the next transition, ±45 ns paired (T_DJR1/2) | Table 7-10 |
| Framing | SYNC KJKJKJKK (NRZI 00000001); PID plus its complement; CRC5 (x^5 + x^2 + 1) and CRC16 (0x8005), both seeded with ones, inverted, MSb first; residues 01100b and 0x800D; a 0 stuffed after six 1s, including at the end of the CRC | 7.1.10, 8.3 |
| Electrical | VOH 2.8–3.6 V; VIH 2.0 V; 1.5 kΩ ± 5 % pull-up on D− to 3.0–3.6 V; edges 75–300 ns; 200–450 pF at the device | Table 7-7, 7.1.2.1 |

### 5.3 Mapping and programs

**Pins:** D− on `uio[0]` (out0 and in0), D+ on `uio[1]` (out1 and in1). J = D− high.

**Receive:**
- `RECOVER` timing, `PERIOD = 40` (one bit), NRZI, differential input (an SE0 makes no edge and no bit, ruling 20).
- De-stuffing: ONES, N = 6.
- SYNC is found by `EXPECT 8, seek` with pattern 0x80.
- CRC5 is checked by `EXPECT 5, src=CRC` straight after the 11 address/endpoint bits.
- A data packet's CRC16 is checked by residue (`CRCCHK` against 0x800D) after the SE0 event aborts the open byte loop.

**Events:**
- E0 = both lines low for 20 cycles (SE0).
- E1 = J (D− high, D+ low) for 4 cycles. It is cleared during SE0 and then waited for, to find the end of the EOP.

**Transmit:**
- `BAUD` at 20 + 20 cycles, NRZI, differential output, stuffing from SYNC to CRC16.
- EOP by `PIN`: SE0 for 4 × 20 cycles, J for 2 × 20, then release to the pull-up.

`usb_dev.asm` is 112 words; `usb_listen.asm` is 15.

**Device flow.**
1. Token fields: SYNC, PID into R0, 11 bits into R1, CRC5 check.
2. `WAIT E0`, clear E1, then check the CRC, the address and the PID. The address is data memory word 16, which the host writes after SET_ADDRESS.
3. IN token:
 - read the EP0 mailbox: word 20 holds the staged bit, length and PID; the bytes are in words 21–28;
 - switch to transmit modes while the EOP is still on the wire;
 - `WAIT E1`, then `PIN wait=3` (at least 2 bit times);
 - send DATAx, the bytes, CRC16 and EOP, or **NAK automatically if nothing is staged**;
 - then receive the host's handshake; on ACK, clear the mailbox and push a record.
4. SETUP or OUT token: receive the DATA packet bytes into memory until the SE0 event, check the residue, then ACK.

**What the host must pre-stage.**
- The address word, and the IN mailbox before the host polls. A control transfer's data stage may be NAKed for up to 5 s, so the RP2350 has ample time.
- It must read each received packet (words 32 on) before the next one arrives. Otherwise the program needs a "buffer full, NAK" flag (3 instructions).

**Cycle budget.**
- Worst path, the IN token: the token's last CRC bit is captured about 100 cycles before the EOP ends, and the program needs about 25 instructions before it can wait for J. The response starts 132–138 cycles after the EOP ends, against a 300-cycle limit.
- Handshake receive after the device's own EOP: the receive search is current about 28 cycles after release, and the host's SYNC can start no earlier than 40 cycles after release.

### 5.4 Results [RTL] (`results/usb_turnaround.json`, `results/usb_turnaround.log`)

Twelve runs, with host clock offsets near 0, +1.5 % and −1.5 % and a random sub-cycle phase. Each run does four transactions. Turnaround is measured on the resolved bus, from the SE0-to-J of the host's packet to the device's first K.

| Transaction | Device reply | Turnaround (cycles / bit times) | Checked |
|---|---|---|---|
| IN, nothing staged | NAK (0x5A) | 132–138 / 3.30–3.45 | reply PID |
| IN, 8 bytes staged as DATA1 | DATA1 + 8 bytes + CRC16, then the host's ACK | 132–137 / 3.30–3.43 | data and CRC16 correct, ACK recorded, mailbox cleared |
| SETUP + DATA0 (8 bytes) | ACK (0xD2) | 82–86 / 2.05–2.15 | received bytes and PID correct, records pushed |
| IN to another address | silence | | |

**Other checks:**
- The device's EOP is SE0 for 80 cycles (1.33 µs; limit 1.25–1.50 µs), then J for one bit.
- In no cycle did the host and the device drive the bus together.
- The listener lane recorded every packet of every transaction in both directions with timestamps: token, NAK, DATA1 with payload and CRC, ACK, SETUP, DATA0, ACK.

**Margins that were not simulated but are large.**
- Receive sampling is mid-bit after every edge, with 333 ns either side.
- With at most 7 bits between edges, 1.5 % drift is 70 ns; host source jitter is 25 ns; quantisation is 17 ns.
- Pad delay (about 10 ns) is 0.015 bit.

### 5.5 Verdicts

- **Device: WORKS AS IS** for the link layer shown: control endpoint 0, IN, SETUP/OUT, the NAK default, the address filter, CRC5/16, stuffing, and EOP in both directions.
- Still program work: bus-reset detection (a `WAIT` timeout after E0, since event windows stop at 63 cycles), suspend, a second endpoint, STALL.
- Electrical: 3.3 V levels fit. Edges are faster than the 75 ns minimum; that works in practice, as bit-banged devices show, but is not compliant without slowing.
- **Listener: WORKS WITH LIMITS.** Each byte is a two-word capture record. The host drains about 38 k records/s, ample for normal low-speed traffic; bursts are limited by the 64-word queue.

## 6. CAN

### 6.1 Roles

Full node (transmit with arbitration, receive, ACK, error signalling), plus listen-only and ACK-only nodes as fallbacks.

### 6.2 Requirements (Bosch CAN 2.0, read; ISO 11898-1 not read)

| Item | Value |
|---|---|
| Bit time | 8–25 time quanta: SYNC 1, PROP 1–8, PS1 1–8, PS2 = max(PS1, 2); sample point at the end of PS1; SJW from 1 to min(4, PS1) |
| Synchronisation | hard sync on the recessive-to-dominant edge of SOF; resync only on recessive-to-dominant edges, at most once per bit; a node sending dominant does not resync on an edge with positive phase error |
| Stuffing | the complement after 5 equal bits, from SOF to the end of the CRC; 6 equal bits is a stuff error |
| CRC | x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1 (0x4599), init 0, over the de-stuffed SOF .. data |
| Tail | CRC delimiter; ACK slot (receivers with a correct frame drive dominant); ACK delimiter; EOF of 7 recessive bits; intermission of 3 |
| Arbitration | bitwise over the identifier and RTR; a node that sends recessive and samples dominant withdraws without sending another bit |
| Errors | active error flag = 6 dominant bits; error passive when TEC or REC ≥ 128; bus-off when TEC ≥ 256 |
| Propagation | PROP ≥ 2 × (bus delay + node output delay + node input delay); transceiver loop delay: TJA1050 ≤ 250 ns (typical 105 ns on the dominant edge, 155 ns on the recessive edge), SN65HVD230 70–135 ns |

### 6.3 Mapping (two lanes per node)

**Pins per node:**
- RXD: one input pin, read by both lanes.
- The TXD net: two `uio` pins tied together on the board, both open-drain. The transceiver's TXD pull-up makes recessive.
- The transmitter lane also reads the TXD net back on `in1`.

**Transmitter lane** (`can_tx.asm`, 51 words).
- `BAUD` timing, with `PERIOD` at the sample point (75 %) and `PERIOD2` for the rest of the bit.
- `DRIVE` fields with CRC and EQUAL-5 stuffing: SOF+ID+RTR (13 bits), IDE/r0/DLC (6), data bytes from memory, CRC (`src=CRC`, 15).
- Then the CRC delimiter; the ACK slot by `EXCHANGE 1` (release and sample); 8 recessive bits.
- Two level events are armed during the frame:
- E0, "RXD dominant while TXD recessive for K cycles": arbitration loss, or a bit error after arbitration;
- E1, "RXD recessive while TXD dominant for K cycles": a bit error.
- On E0 during arbitration the lane is aborted inside the losing bit. Outputs keep their level on abort, and that level is recessive, so the lane never sends another bit.
- Constraints on K:
- K must exceed the loop delay plus the offset to the other nodes;
- the loss must abort the lane before the next launch: launch + 4 + K + 7 < bit time.
- K = 40 at 500 k gives 51 < 120. K = 30–40 at 1 M gives 41–51 < 60.

**Receiver lane** (`can_rx.asm`, 42 words).
- `RECOVER` timing: mid-bit sampling after every edge.
- De-stuffing with the stuff-error event (E3).
- SOF is found by a 1-bit `EXPECT` loop; then the ID and control fields, and a byte loop driven by DLC.
- CRC checked by `EXPECT 15, src=CRC`.
- ACK decision: if the CRC is good and the transmitter lane has not flagged the frame as its own (channel `OWN`, peeked with `BR Cn`), one `DRIVE 3` of 1,0,1 is launched at the CRC delimiter. That puts a dominant ACK slot on the TXD net in the receiver's own bit timing.
- Then EOF and intermission are checked, and a "bus free" channel signal tells the transmitter lane when it may start.
- Error flag on a stuff error, by `PIN`.

**Cycle budgets** [confirmed by the RTL runs]:
- From the DLC decision to the first data field: about 45 cycles (one bit is 60 at 1 M).
- From the CRC check to the ACK field being current: at most 23 cycles. It is needed by the delimiter's launch tick, 30 cycles (1 M) or 60 cycles (500 k) after the last CRC capture.

### 6.4 Results [RTL] (`results/can_arbitration_*.json`)

**Setup:**
- node A = lanes 0+1, ID 0x123, 2 bytes;
- node B = lanes 2+3, ID 0x0F0, 1 byte;
- an external scripted node, ID 0x700, 3 bytes, all on one bus.

The external frame is sent first. Both nodes start when their receiver lanes see the end of its intermission.

The outcome was identical in every configuration tested:

| Bit rate | Loop delay | K |
|---|---|---|
| 500 k | 8 cycles (133 ns) | 40 |
| 500 k | 15 cycles (250 ns) | 40 |
| 1 M | 8 cycles | 30 |
| 1 M | 15 cycles | 30 and 40 |
| 250 k | 15 cycles | 60 |

In each:
- **Bus:** frames 0x700, 0x0F0 and 0x123, each acknowledged.
- **A's transmitter:** lost at ID bit 8 (record C1), then sent its frame and was acknowledged (C0).
- **B's transmitter:** sent and was acknowledged.
- **Receivers:** both took all three frames with a good CRC, and each acknowledged the other node's frames but not its own.

**Detail of the contested frame:**
- Node A's TXD net was dominant for exactly 360 cycles at the start of the contested frame: SOF and two identifier bits, 3 × 120.
- It was not dominant again until A's receiver lane drove the ACK slot of B's frame. That ACK bit lasted 132 cycles: the loop delay plus the synchroniser stretch it, because `RECOVER` restarts its timer on the node's own ACK edge. This is harmless; it ends well before the transmitter samples the ACK delimiter.

**Fallback nodes:** both receiver lanes acknowledged the external frame, so the **ACK-only node** works. The **listen-only node** is the same receiver without the `DRIVE 3`.

### 6.5 What limits it

**Two lanes per node.** One lane cannot do the job, because:
- receive de-stuffing needs `RECOVER` or `EDGE` timing (ruling 24);
- `EXCHANGE` stuffs and CRCs the transmitted stream;
- a `RECOVER` transmitter would restart its bit timer on its own echoed edges, and each restart lengthens a bit by the loop delay.

A single-lane CAN mode (H5: `EXCHANGE` feeding stuffing and CRC from the sampled bit, and forcing recessive after a lost bit) would cost about 1 k µm² per lane. It rewires the field/stuffing contract, so it is not recommended while two lanes work.

**SOF alignment against standard controllers** [analysis, not simulated].
- The transmitter lane times its bits from its own SOF and never resynchronises; crystal drift over one frame is negligible.
- For correct arbitration, its SOF must fall within about (the other nodes' sample point − loop delay) of the earliest competing SOF.
- After a frame, the start time is deterministic, because the receiver lane's timer free-runs through EOF and intermission. So it can be tuned in software:
- as written, the SOF lands 0.29 bit late at 500 k, which is fine;
- it lands 1.08 bits late at 1 M, which would fail; signalling one bit earlier fixes it.
- After a long idle, a competitor may start at any moment. Joining it through an event handler costs 15–25 cycles plus the loop delay: fine at 500 k, marginal at 1 M. H3 cuts this to about 5 cycles plus the loop delay.

**Receiver timing.**
- The receiver resyncs on both edge directions at a fixed 50 % sample point. CAN's rule is recessive-to-dominant edges only, limited by SJW, sampling at 75–87.5 %.
- It worked in simulation. On a real bus, the slower dominant-to-recessive transceiver edge shifts sampling by about 3 cycles (5 % of a bit at 500 k). H4 makes the timing standard.

**Program and host work.** Error confinement (counters, error passive, bus-off), overload frames, extended identifiers, RTR frames and DLC > 8 are left to program or host. These programs signal only bit errors and stuff errors.

### 6.6 Verdicts

- **Full node: WORKS WITH LIMITS.** Two lanes per node. Rosetta-to-Rosetta shown to 1 Mbit/s. Against standard controllers, to 500 kbit/s by software tuning; joins at 1 Mbit/s after bus idle need H3.
- **Single-lane full node: CANNOT** without H5.
- **Listen-only and ACK-only nodes: WORK AS IS**, one lane each.

## 7. Sources

- USB 2.0 specification: https://www.pjrc.com/tmp/usb20.pdf
- UNH-IOL Clause 14 MAU Test Suite v5.2, and the UNH-IOL 10BASE-T MAU slides (iol.unh.edu)
- National Semiconductor AN-783
- Cirrus Logic CS8900A, TI TLK111 and DP83848 datasheets; TI SNLA266B
- Tektronix 10BASE-T compliance report
- UNH-IOL Clause 28 State Machine Base Page Test Suite
- Bosch CAN Specification 2.0, and Bosch "The Configuration of the CAN Bit Timing"
- TJA1050, MCP2551 and SN65HVD230 datasheets
- V-USB product page

The IEEE 802.3 text itself was not reachable, so every 802.3 figure here comes from sources that cite its clauses.

## 8. Could not determine

- **802.3 details:**
- the ±0.01 % rate tolerance (not found in a source read);
- the exact TP_IDL template corners;
- how a receiver jitter test distributes the ±13.5 ns (the deterministic pulse-width windows were read; the random content was not);
- the transmit jitter figures, which disagree between sources.
- **Electrical:** whether the pads can drive a 10BASE-T transmitter directly; the effect of pad rise/fall asymmetry on transmit DCD; the RP2350 clock's duty cycle (it matters only for H2).
- **Real equipment:** a NIC's actual jitter as seen after our comparator; a PC USB host's real inter-packet timing; standard CAN controllers' SOF timing (section 6.5 is analysis).
- **Areas:** H1–H5 are estimates; nothing was synthesised.
- **Precheck:** its treatment of a ring oscillator or delay chains.

## Appendix: dropped protocols (notes only)

- **JTAG controller:** `BAUD` clock on out3, TDI on out0, TDO on in0 (`EXCHANGE`, as in the SPI controller); TMS on out1/out2 by `PIN` between fields (the clock stops, and JTAG is static); the last scan bit as a separate 1-bit field.
- **SWD controller:** one bidirectional pin; drive phases capture on the rising edge, target phases on the falling edge, switched by `SETM TICK` during turnaround while the clock idles; parity by a 1-bit CRC; the ACK checked between fields.
- **PS/2:** open-drain clock and data with 5 V pull-ups need level shifting. Device by `BAUD` with clock output and `stretch_honour`; host by `EDGE`; odd parity by a 1-bit CRC seeded with ones.
