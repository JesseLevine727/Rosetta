# Block spec: `rosetta_stuff` (bit stuffing)

Spec **v2** (13 Nov). Changes from v1, all found by the designer and the
property writer: `EQUAL` mode with N = 1 is treated as disabled (it could
never stop stuffing); a violating bit now restarts the run, so behaviour
after a violation is the same for every N and `violation_o` is always a
single pulse; configuration timing and the hidden "history" state are
stated.

Watches the bits on the wire and says when the next wire bit must be a
stuff bit. The same block serves transmit (the lane inserts the bit) and
receive (the lane drops the bit and checks it).

## Ports

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `cfg_en_i` | 1 | stuffing enabled |
| `cfg_n_i` | 3 | run length `N`, 1-7. `N = 0` behaves as disabled. In `EQUAL` mode `N = 1` also behaves as disabled |
| `cfg_mode_i` | 1 | 0 = `ONES`: after `N` consecutive 1s insert a 0 (USB: N=6, HDLC: N=5). 1 = `EQUAL`: after `N` consecutive equal bits insert the complement (CAN: N=5) |
| `clear_i` | 1 | start of a frame: forget history (wins over `bit_valid_i`) |
| `bit_valid_i` | 1 | a bit is on the wire this cycle |
| `bit_i` | 1 | that wire bit (data bit or stuff bit) |
| `stuff_next_o` | 1 | the next wire bit must be a stuff bit |
| `stuff_bit_o` | 1 | its required value (meaningful when `stuff_next_o`) |
| `violation_o` | 1 | pulses for one cycle, the cycle after a wire bit arrived while `stuff_next_o` was 1 and differed from `stuff_bit_o` |

## Behaviour

State: `run` (0-7), `last` (1 bit), `viol` (1 bit). "No history" means
`run == 0`. Define `active = cfg_en_i && cfg_n_i != 0 && !(cfg_mode_i == EQUAL && cfg_n_i == 1)`.

Outputs are functions of the state and the `cfg_` inputs only:

- `stuff_next_o = active && run == cfg_n_i`
- `stuff_bit_o` = 0 in `ONES` mode, `~last` in `EQUAL` mode. Meaningful
  only while `stuff_next_o` is 1.
- `violation_o = viol && active`

On each rising clock edge, in priority order:

1. Reset (asynchronous) or `clear_i`: `run = 0`, `viol = 0`. `last` is 0
   after reset and unchanged by `clear_i`.
2. `bit_valid_i`: let `bad = stuff_next_o && bit_i != stuff_bit_o` (values
   before the edge).
   - `viol = bad`.
   - If `bad`: `run = 1` (the violating bit is treated as the first bit
     after a clear). In `EQUAL` mode `last = bit_i`.
   - Else, `ONES` mode: `run = bit_i ? min(run + 1, 7) : 0`.
   - Else, `EQUAL` mode: `run = (run != 0 && bit_i == last) ? min(run + 1, 7) : 1`;
     `last = bit_i`.
3. Otherwise: `run` and `last` hold, `viol = 0`.

The run and `last` update whether or not stuffing is active. `cfg_` inputs
change only in reset or in a cycle with `clear_i`.

Consequences worth stating:

- A correct stuff bit gives `run = 0` (`ONES`) or `run = 1` (`EQUAL`).
- `violation_o` is high for exactly one cycle per violating bit, for every
  N. After a violation the stream is no longer a valid frame; the block
  keeps counting from the violating bit, and the lane is expected to
  abandon the frame and assert `clear_i`.
- The lane selects which fields are stuffed by gating `bit_valid_i`, not by
  toggling `cfg_en_i`. A stuff bit still pending after the last stuffed
  data bit must be sent and checked by the lane (USB, CAN and HDLC all
  require it).

## Required properties

1. With stuffing enabled and the transmitter obeying `stuff_next_o`, the
   wire never carries more than `N` consecutive 1s (`ONES`) or more than
   `N` consecutive equal bits (`EQUAL`).
2. **Round trip**: one instance watching a transmitter that inserts stuff
   bits and a second instance used by a receiver that drops them recover
   exactly the original data bits, for every active configuration and every
   data sequence, and a stuff bit is always followed by a data bit (the
   stream makes progress); and `violation_o` never fires on such a stream.
3. `violation_o` fires exactly when a stuff position carries the wrong bit.
4. `clear_i` priority; disabled behaviour (including `EQUAL` with N = 1);
   reset values.
5. `violation_o` is never high in two consecutive cycles unless two
   consecutive bits each violate, and after a violation the run is 1.
