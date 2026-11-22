; 10BASE-T receive, lane 0, hand-written for the study (not compiled).
; in0 = output of an external differential comparator on RD+/RD- (after the
; magnetics). PULSE timing, Manchester decode with THR0 = 5 (tick.md 6,
; coding.md 3.2): an edge interval of 5 cycles or more is a mid-bit edge.
; Each 32-bit field goes to data memory (dst MEM, R3 post-incremented by
; the delivery) msb first with a marker 1 preloaded (src IMM 1), so after an
; abandoned field SR = (1 << k) | the k bits received: the host knows the
; exact length of the last partial word. TP_IDL (line high for 12 cycles,
; 200 ns, longer than any data interval) ends the frame through event 0.
        SETM  TICK, mode=PULSE
        SETM  CODE, rx=MANCHESTER, data_in=in0
        SETM  THR01, thr0=5
        SETM  CRCPOLY, 0x1DB7
        SETM  CRCPOLY.hi, 0x04C1
        SETM  CRCCFG, width=32, init_ones=1, xorout_ones=1
        SETM  EVC0, a=in0, trigger=LEVEL, level=HIGH, k=12
        LDI   R2, 0xD555           ; last 16 bits of preamble + SFD, lsb window
frame:  LDI   R3, 8                ; frame buffer from word 8 (after an 8-word in-queue)
        LDI   R0, 0xDD7B           ; CRC-32 residue, raw state 0xC704DD7B
        LDI   R1, 0xC704
        EVT   0, eof, on, clear
        CRCINIT
        FIELD EXPECT 16, lsb, seek, src=R2
data:   FIELD SAMPLE 32, crc, src=IMM 1, dst=MEM
        BR    always, data
eof:    CRCCHK                     ; M = residue good
        Q     PUSH R3, tag=0xE1    ; next free word = 8 + full words received
        ALU   MOVSRH R0
        Q     PUSH R0, tag=0xE2    ; partial word, high half
        ALU   MOVSR R0
        Q     PUSH R0, tag=0xE3    ; partial word, low half
        BR    M, good
        Q     PUSHI 0, tag=0xEE
        BR    always, frame
good:   Q     PUSHI 1, tag=0xEE
        BR    always, frame
