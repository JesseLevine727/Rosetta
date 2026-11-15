; Copied verbatim from rosetta-fpga results/loopback_20261114/uart_tx.asm: hardware-proven
; on the PYNQ-Z1 at 60 MHz, 14 Nov 2026 (assembles to that run's image.json words).
; lane 0, origin 0: UART transmit 8N1 (toolchain/README.md "UART transmit").
; PERIOD2 is patched by the host script for the rate-tolerance runs.
        SETM  TICK, mode=BAUD
        SETM  PERIOD, 260
        SETM  PERIOD2, 260
        PIN   out0=1               ; idle high
next:   Q     POP R0               ; waits for the host
        FIELD DRIVE 1, IMM 0       ; start bit
        FIELD DRIVE 8, lsb, R0
        FIELD DRIVE 1, IMM 1       ; stop bit
        BR    always, next
