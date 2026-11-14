# Block spec: `rosetta_stuff` (bit stuffing)

Watches the bits on the wire and says when the next wire bit must be a
stuff bit. The same block serves transmit (the lane inserts the bit) and
receive (the lane drops the bit and checks it).

## Ports

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `cfg_en_i` | 1 | stuffing enabled |
| `cfg_n_i` | 3 | run length `N`, 1-7. `N = 0` behaves as disabled |
| `cfg_mode_i` | 1 | 0 = `ONES`: after `N` consecutive 1s insert a 0 (USB: N=6, HDLC: N=5). 1 = `EQUAL`: after `N` consecutive equal bits insert the complement (CAN: N=5) |
| `clear_i` | 1 | start of a frame: forget history (wins over `bit_valid_i`) |
| `bit_valid_i` | 1 | a bit is on the wire this cycle |
| `bit_i` | 1 | that wire bit (data bit or stuff bit) |
| `stuff_next_o` | 1 | the next wire bit must be a stuff bit |
| `stuff_bit_o` | 1 | its required value (meaningful when `stuff_next_o`) |
| `violation_o` | 1 | pulses for one cycle, the cycle after a wire bit arrived while `stuff_next_o` was 1 and differed from `stuff_bit_o` |

## Behaviour

State: `run` (0-7), `last` (1 bit). All outputs are registered functions of
state. On each cycle with `bit_valid_i` (and not `clear_i`), where
`was_stuff = stuff_next_o` before the edge:

- `ONES` mode: if `bit_i` is 1, `run = run + 1` (saturating at 7), else
  `run = 0`. A correct stuff bit is 0, so it resets the run.
- `EQUAL` mode: if there is history and `bit_i == last`, `run = run + 1`
  (saturating), else `run = 1`. `last = bit_i`. A stuff bit counts as the
  first bit of the next run.
- `stuff_next_o = cfg_en_i && N != 0 && run == N` after the update.
- `stuff_bit_o` = 0 in `ONES` mode, `~last` in `EQUAL` mode.
- `violation_o = was_stuff && bit_i != stuff_bit_o` (value before the edge).

`clear_i` or reset: `run = 0`, no history, `stuff_next_o = 0`,
`violation_o = 0`. When disabled, `stuff_next_o` and `violation_o` stay 0.

## Required properties

1. With stuffing enabled and the transmitter obeying `stuff_next_o`, the
   wire never carries more than `N` consecutive 1s (`ONES`) or more than
   `N` consecutive equal bits (`EQUAL`).
2. **Round trip**: one instance watching a transmitter that inserts stuff
   bits and a second instance used by a receiver that drops them recover
   exactly the original data bits, for every `N`, both modes, every data
   sequence; and `violation_o` never fires on such a stream.
3. `violation_o` fires exactly when a stuff position carries the wrong bit.
4. `clear_i` priority; disabled behaviour; reset values.
