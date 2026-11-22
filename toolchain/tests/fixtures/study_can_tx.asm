; CAN transmitter lane (study; one of the two lanes of a node). 500 kbit/s:
; BAUD 90 + 30 cycles (sample point 75 %). out0 = TXD (open-drain) on the
; node's TXD net, in0 = RXD, in1 = read-back of the node's TXD net.
; Host: queue word 1 = SOF+ID+RTR (13 bits, msb first), word 2 = IDE, r0,
; DLC (6 bits); data memory word 7 = number of data bytes, words 8.. = bytes.
; Channels: OWN (put while this node transmits, for the receiver lane's ACK
; decision), FREE (put by the receiver lane when intermission is over).
; Records: 0xC0 sent and acknowledged, 0xC1 arbitration lost, 0xC2 no ACK,
; 0xC3 bit error.
        SETM  TICK, mode=BAUD
        SETM  PERIOD, 90
        SETM  PERIOD2, 30
        SETM  CODE, data_in=in0, data_od=1
        SETM  STUFF, n=5, mode=EQUAL, enable=1
        SETM  CRCPOLY, 0x4599
        SETM  CRCCFG, width=15
        SETM  EVC0, a=in0, level=LOW, trigger=LEVEL, b=in1, b_must=HIGH, k=KD
        SETM  EVC1, a=in0, level=HIGH, trigger=LEVEL, b=in1, b_must=LOW, k=KD
new:    Q     POP R1
        Q     POP R2
retry:  Q     GET R0, FREE                  ; wait for intermission to end
        LDI   R3, 8
        LD    R0, [7]
        Q     PUT OWN, R1
        STUFFCLR
        CRCINIT
        EVT   0, lost, on, clear            ; sent recessive, read dominant
        EVT   1, berr, on, clear            ; sent dominant, read recessive
        FIELD DRIVE 13, crc, stuff, src=R1  ; SOF, identifier, RTR
        FIELD DRIVE 6, crc, stuff, src=R2   ; IDE, r0, DLC
        EVT   0, berr, on                   ; arbitration over: now a bit error
        ALU   CMP R0, 0
        BR    Z, crc
dl:     FIELD DRIVE 8, crc, stuff, src=MEM
        LOOP  R0, dl
crc:    FIELD DRIVE 15, stuff, src=CRC
        FIELD DRIVE 1, src=IMM 1            ; CRC delimiter
        EVT   0, lost, off                  ; runs when the delimiter starts
        EVT   1, berr, off
        FIELD EXCHANGE 1, src=IMM 1, dst=R0 ; ACK slot: release and sample
        FIELD DRIVE 8, src=IMM 0xFF         ; ACK delimiter, EOF
        ALU   CMP R0, 0
        Q     GET R3, OWN
        BR    Z, acked
        Q     PUSHI 2, tag=0xC2
        BR    always, new
acked:  Q     PUSHI 0, tag=0xC0
        BR    always, new
lost:   EVT   0, lost, off
        EVT   1, berr, off
        Q     GET R0, OWN                   ; withdraw: the receiver lane must ACK
        Q     PUSHI 1, tag=0xC1
        BR    always, retry
berr:   EVT   0, lost, off
        EVT   1, berr, off
        PIN   out0=0, wait=8                ; active error flag (6 bits = 8 x 90 cycles)
        PIN   out0=z
        Q     GET R0, OWN
        Q     PUSHI 3, tag=0xC3
        BR    always, retry
