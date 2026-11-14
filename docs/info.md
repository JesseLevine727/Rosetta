## How it works

Rosetta is a protocol tool that is told what a protocol looks like instead of
having each protocol built in. A short description of a protocol (its wires,
its timing, and the fields of a frame) is compiled on a computer into small
programs, which are loaded into the chip over an SPI host port.

The chip has four identical lanes. Each lane does the bit-speed work of one
side of one protocol: timing (fixed rate, an external clock edge, clock
recovery, or pulse widths), line coding, bit stuffing, CRC, field shifting,
event detection (for example I2C start and stop), and pin drive. A shared
sequencer runs one small program per lane from a 512 x 32 program memory, and
a 512 x 32 data memory holds the queues and timestamped capture records for
each lane. A crossbar connects any lane to any pin, or to another lane for
loopback self-test.

From one description the chip can speak a protocol in any role, listen to a
bus and timestamp what it sees, inject faults, and pass data from one
protocol to another. The design is verified block by block with formal
proofs.

This repository is work in progress; the design is not yet complete.

## How to test

The host (the RP2350 on the demo board) talks to the chip over the SPI host
port on `ui[0]` (clock), `ui[1]` (chip select, active low), `ui[2]` (data
in) and `uo[0]` (data out). `uo[1]` is an interrupt output.

1. Hold reset, then release it with the 60 MHz clock running.
2. Load a compiled program image and the pin routing over the host port.
3. Start the lanes and exchange data through the host port queues.

A self-test that needs no external parts: route one lane's output to another
lane's input through the crossbar, load a UART sender on the first and a
UART receiver on the second, send bytes and read them back.

## External hardware

None for UART, SPI and the loopback self-test. I2C and other open-drain
buses need pull-up resistors on the bidirectional pins.
