; Copied from rosetta-fpga results/loopback_20261114/uart_listener.asm: hardware-proven on the
; PYNQ-Z1 at 60 MHz, 14 Nov 2026. Only comments changed: the notes about the old
; event-register layout are gone (asm.py now encodes ruling 10 directly); the
; instructions assemble to that run's image.json words.
; lane 1, origin 48: UART receive as a listener (toolchain/README.md "UART receive as a listener").
        SETM  TICK, mode=RECOVER
        SETM  PERIOD, 521
        SETM  EVC0, a=in0, trigger=EDGE, level=LOW
idle:   EVT   0, idle, off, clear
        WAIT  E0
        FIELD EXPECT 1, IMM 0
        BR    NM, idle             ; a glitch: dropped silently
        FIELD SAMPLE 8, lsb, CAP, tag=1
        FIELD EXPECT 1, IMM 1
        BR    M, idle
        Q     PUSHTS
        Q     PUSHI 0, tag=0xFE    ; framing error, with a timestamp
        BR    always, idle
