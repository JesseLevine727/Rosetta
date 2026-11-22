; 10BASE-T transmit, lane 2 (study). out0 = TD+, out1 = TD- (differential
; output, CODE[8]); idle = both low (no differential voltage). BAUD 3 + 3
; cycles = 100 ns per bit, Manchester (first half ~bit, second half bit).
; Host: frame bytes (destination .. payload, no FCS) as 32-bit words at data
; memory words 16.., then pushes the word count to the host-in queue.
; Between frames a 100 ns normal link pulse every 78 x 204.75 us = 15.97 ms.
.equ NLPN, 78
        SETM  TICK, mode=BAUD
        SETM  PERIOD, 3
        SETM  PERIOD2, 3
        SETM  CODE, tx=MANCHESTER, diff_out=1
        SETM  CRCPOLY, 0x1DB7
        SETM  CRCPOLY.hi, 0x04C1
        SETM  CRCCFG, width=32, init_ones=1, xorout_ones=1
        LDI   R0, 0x5555
        LDI   R1, 0xD555
        PIN   out0=0, out1=0
nlp:    PIN   out0=1, out1=0, wait=2        ; link pulse, 2 x 50 ns
        PIN   out0=0, out1=0
        LDI   R2, NLPN
w:      BR    QNE, send
        WAIT  timeout=4095                  ; 4095 x 3 cycles
        LOOP  R2, w
        BR    always, nlp
send:   Q     POP R2                        ; words to send
        LDI   R3, 16
        CRCINIT
        FIELD DRIVE 16, lsb, src=R0         ; preamble
        FIELD DRIVE 16, lsb, src=R0
        FIELD DRIVE 16, lsb, src=R0
        FIELD DRIVE 16, lsb, src=R1         ; preamble end, SFD
dloop:  FIELD DRIVE 32, lsb, crc, src=MEM
        LOOP  R2, dloop
        FIELD DRIVE 32, src=CRC             ; FCS, x^31 first
        PIN   out0=1, out1=0, wait=6        ; TP_IDL: positive for 300 ns after the last bit
        PIN   out0=0, out1=0
        BR    always, nlp
