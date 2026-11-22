; Low-speed USB listener, lane 1 (study). Same pins as the device, inputs only.
; Records (CAP: timestamp word, then value word with the tag in [31:24]):
;   0xA0  SYNC matched (packet start); 0xB1 one byte (PID, payload or CRC);
; plain record 0xEE: end of packet (SE0 for 20 cycles).
        SETM  TICK, mode=RECOVER
        SETM  PERIOD, 40
        SETM  CODE, rx=NRZI, data_in=in0, diff_in=1
        SETM  STUFF, n=6, mode=ONES, enable=1
        SETM  EVC0, a=in0, b=in1, trigger=BOTHLOW, k=20
        LDI   R2, 0x80
lsn:    STUFFCLR
        FIELD EXPECT 8, lsb, seek, src=R2, dst=CAP, tag=0xA0
        FIELD SAMPLE 8, lsb, stuff, dst=CAP, tag=0xB1
        EVT   0, eop, on, clear              ; runs once the SYNC has matched
lloop:  FIELD SAMPLE 8, lsb, stuff, dst=CAP, tag=0xB1
        BR    always, lloop
eop:    EVT   0, eop, off
        Q     PUSHI 0, tag=0xEE
        BR    always, lsn
