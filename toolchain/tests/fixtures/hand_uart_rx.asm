; Copied from rosetta-fpga results/loopback_20261114/uart_rx.asm: hardware-proven on the
; PYNQ-Z1 at 60 MHz, 14 Nov 2026. Only comments changed: the notes about the old
; event-register layout are gone (asm.py now encodes ruling 10 directly); the
; instructions assemble to that run's image.json words.
; lane 1, origin 16: UART receive, receiver role (toolchain/README.md "UART receive").
        SETM  TICK, mode=RECOVER
        SETM  PERIOD, 521          ; one bit time
        SETM  EVC0, a=in0, trigger=EDGE, level=LOW
idle:   EVT   0, idle, off, clear  ; forget edges of the last frame
        WAIT  E0
        FIELD EXPECT 1, IMM 0      ; start bit, sampled mid-bit
        BR    NM, idle
        FIELD SAMPLE 8, lsb, HOSTQ, tag=1
        FIELD EXPECT 1, IMM 1      ; stop bit
        BR    M, idle
        Q     PUSHI 0, tag=0xFE    ; framing error record
        BR    always, idle
