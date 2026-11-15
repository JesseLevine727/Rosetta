; Copied from rosetta-fpga results/loopback_20261114/spi_controller.asm: hardware-proven on the
; PYNQ-Z1 at 60 MHz, 14 Nov 2026. Only comments changed: the notes about the old
; event-register layout are gone (asm.py now encodes ruling 10 directly); the
; instructions assemble to that run's image.json words.
; lane 2, origin 64: SPI controller, mode 0, 8-bit words, 1 MHz (toolchain/README.md "SPI controller").
; out0 = MOSI, in0 = MISO, out2 = CS_n, out3 = SCK. PERIOD = PERIOD2 = 30 -> 60 cycles/bit.
        SETM  TICK, mode=BAUD, clock_out        ; SCK idle low, capture rising
        SETM  CODE, data_in=in0
        SETM  PERIOD, 30
        SETM  PERIOD2, 30
        PIN   out2=1, out0=0       ; deselect before the first word
word:   Q     POP R0
        PIN   out2=0               ; chip select low   (CSWAIT: variant B adds ", wait=1")
        FIELD EXCHANGE 8, early, R0, HOSTQ
        PIN   out2=1               ; at the end of the last bit
        BR    always, word
