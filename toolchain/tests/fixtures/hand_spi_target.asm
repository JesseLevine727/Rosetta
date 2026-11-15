; Copied from rosetta-fpga results/loopback_20261114/spi_target.asm: hardware-proven on the
; PYNQ-Z1 at 60 MHz, 14 Nov 2026. Only comments changed: the notes about the old
; event-register layout are gone (asm.py now encodes ruling 10 directly); the
; instructions assemble to that run's image.json words.
; lane 3, origin 80: SPI target, mode 0, 8-bit words.
; in0 = MOSI, in1 = SCK (looped back), in2 = CS_n; out0 = MISO (push-pull).
        SETM  TICK, mode=EDGE, clock_in=in1, capture=RISING
        SETM  CODE, data_in=in0
        SETM  EVC0, a=in2, trigger=EDGE, level=LOW
        PIN   out0=0               ; drive MISO
wait:   EVT   0, wait, off, clear  ; forget CS edges seen so far
        Q     POP R0               ; the word to answer with (host supplies it first)
        WAIT  E0                   ; chip select
        FIELD EXCHANGE 8, early, R0, HOSTQ   ; MSB out at once (mode 0), sample on SCK rising
        BR    always, wait
