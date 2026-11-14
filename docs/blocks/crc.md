# Block spec: `rosetta_crc` (check unit)

Computes any CRC of width 1-32, one wire bit per update.

## Ports

| Port | Width | Meaning |
|---|---|---|
| `clk`, `rst_n` | 1 | clock; async active-low reset |
| `cfg_poly_i` | 32 | generator polynomial without its top term, right-aligned in the low `W` bits (CRC-32: `32'h04C11DB7`; CRC-5/USB: `5'h05`) |
| `cfg_init_i` | 32 | value loaded by `init_i`, low `W` bits used |
| `cfg_width_m1_i` | 5 | `W - 1`, so `W` is 1-32 |
| `cfg_xorout_i` | 1 | when 1, `value_o` is the complement of the state |
| `init_i` | 1 | load `cfg_init_i` into the state (wins over `en_i`) |
| `en_i` | 1 | consume `bit_i` this cycle |
| `bit_i` | 1 | next bit, in wire order |
| `state_o` | 32 | raw state, low `W` bits; bits `W` and above are 0 |
| `value_o` | 32 | state, complemented in the low `W` bits if `cfg_xorout_i`; bits `W` and above are 0 |

## Behaviour

With state `s` (`W` bits), on a rising clock edge:

- `rst_n` low: `s = 0`.
- `init_i`: `s = cfg_init_i[W-1:0]`.
- else `en_i`: `fb = s[W-1] ^ bit_i`; `s = (s << 1)[W-1:0] ^ (fb ? cfg_poly_i[W-1:0] : 0)`.
- else: `s` holds.

`state_o` and `value_o` are functions of the registered state only (no
combinational path from `bit_i`, `en_i` or `init_i`).

No bit reflection exists in the hardware. "Reflected" CRCs are the same
shift register fed in wire order; the lane sends the check value starting
from bit `W-1`.

## Known answers (data = ASCII "123456789")

Bytes are fed in wire order as stated.

| CRC | W | poly | init | xorout | byte bit order on the wire | `value_o` after 72 bits |
|---|---|---|---|---|---|---|
| CRC-32 (Ethernet) | 32 | `04C11DB7` | `FFFFFFFF` | 1 | LSB first | bit-reverse of `CBF43926` = `649C2FD3` |
| CRC-16/USB | 16 | `8005` | `FFFF` | 1 | LSB first | bit-reverse of `B4C8` = `132D` |
| CRC-16/CCITT-FALSE | 16 | `1021` | `FFFF` | 0 | MSB first | `29B1` |
| CRC-15/CAN | 15 | `4599` | `0000` | 0 | MSB first | `059E` |
| CRC-8 | 8 | `07` | `00` | 0 | MSB first | `F4` |
| CRC-5/USB | 5 | `05` | `1F` | 1 | LSB first | bit-reverse (5 bits) of `19` = `13` |
| Parity (even) | 1 | `1` | `0` | 0 | any | `1` (odd number of ones in the data) |

The property writer must independently confirm these constants from a
software CRC before relying on them.

## Required properties

1. The update rule above, for every `W`, polynomial and input.
2. Bits `W` and above of both outputs are always 0.
3. `init_i` priority; hold when idle; reset value.
4. Each known answer, as a proof over the fixed 72-bit input.
5. Linearity: for a fixed configuration and equal-length inputs from init 0,
   `crc(a ^ b) = crc(a) ^ crc(b)` (two-copy proof).
