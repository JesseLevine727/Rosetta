; CAN receiver lane (study; the other lane of a node). RECOVER 120 cycles
; (500 kbit/s, sample at mid-bit after every edge). in0 = RXD, out0 = ACK
; drive (open-drain) on the node's TXD net. De-stuffing EQUAL 5 with the
; stuff-error event (STUFF[5] -> E3). Records: 0xD1 identifier, 0xD2 control
; bits (RTR IDE r0 DLC), 0xD3 data byte, 0xD4 end {1 = CRC good, 2 = own
; frame, 4 = ACK driven}, 0xD5 stuff error, 0xD6 form error after the ACK.
        SETM  TICK, mode=RECOVER
        SETM  PERIOD, 120
        SETM  CODE, data_in=in0, data_od=1
        SETM  STUFF, n=5, mode=EQUAL, enable=1, violation_event=1
        SETM  CRCPOLY, 0x4599
        SETM  CRCCFG, width=15
        LDI   R3, 0x3FF
        EVT   3, serr, on, clear
sofw:   STUFFCLR
        CRCINIT
        FIELD EXPECT 1, crc, stuff, src=IMM 0          ; SOF (dominant)
        BR    NM, sofw
        FIELD SAMPLE 11, crc, stuff, dst=HOSTQ, tag=0xD1
        FIELD SAMPLE 7, crc, stuff, dst=R0             ; RTR IDE r0 DLC
        ALU   MOV R1, R0
        ALU   AND R1, 0xF                              ; data bytes (DLC <= 8 assumed)
        Q     PUSH R0, tag=0xD2
        BR    Z, crc
dlp:    FIELD SAMPLE 8, crc, stuff, dst=HOSTQ, tag=0xD3
        LOOP  R1, dlp
crc:    FIELD EXPECT 15, stuff, src=CRC                ; M = CRC good
        LDI   R2, 0
        BR    NM, noack
        LDI   R2, 1
        BR    OWN, own
        FIELD DRIVE 3, src=IMM 5                       ; delimiter, ACK (dominant), ACK delimiter
        LDI   R2, 5
        BR    always, eof
own:    LDI   R2, 3
noack:  FIELD SAMPLE 3, dst=NONE
eof:    FIELD EXPECT 10, src=R3                        ; EOF (7) and intermission (3)
        BR    NM, ferr                                 ; sync: waits for the 10 bits
        BR    FREE, full
        Q     PUT FREE, R2
full:   Q     PUSH R2, tag=0xD4
        BR    always, sofw
ferr:   Q     PUSHI 0, tag=0xD6                        ; form error in EOF / intermission
        BR    always, sofw
serr:   PIN   out0=0, wait=6                           ; error flag, 6 bits
        PIN   out0=z
        Q     PUSHI 0, tag=0xD5
        BR    always, sofw
