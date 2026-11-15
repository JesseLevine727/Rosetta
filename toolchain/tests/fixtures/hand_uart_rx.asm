; Copied verbatim from rosetta-fpga results/loopback_20261114/uart_rx.asm: hardware-proven
; on the PYNQ-Z1 at 60 MHz, 14 Nov 2026 (assembles to that run's image.json words).
; lane 1, origin 16: UART receive, receiver role (toolchain/README.md "UART receive").
; The EVC0 line is replaced by a hand-built word (see EVC_FIX below):
; asm.py encodes EVC with the v0.2 layout (trigger [4:3], k [15:9]); the RTL
; (rosetta_events.sv, ruling 10) uses trigger [5:3] (1 level, 2 edge, 3 any,
; 4 both-low, 0 off) and K [15:10].
        SETM  TICK, mode=RECOVER
        SETM  PERIOD, 521          ; one bit time
        SETM  EVC0, a=in0, trigger=EDGE, level=LOW    ; EVC_FIX -> .word 0x35000010
idle:   EVT   0, idle, off, clear  ; forget edges of the last frame
        WAIT  E0
        FIELD EXPECT 1, IMM 0      ; start bit, sampled mid-bit
        BR    NM, idle
        FIELD SAMPLE 8, lsb, HOSTQ, tag=1
        FIELD EXPECT 1, IMM 1      ; stop bit
        BR    M, idle
        Q     PUSHI 0, tag=0xFE    ; framing error record
        BR    always, idle
