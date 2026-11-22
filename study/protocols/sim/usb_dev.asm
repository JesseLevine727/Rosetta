; Low-speed USB device, lane 0, hand-written for the study (not compiled).
; D- = uio[0] (out0, in0), D+ = uio[1] (out1, in1); J = D- high. External:
; 1.5 k pull-up D- to 3.3 V, series resistors. Receive: RECOVER, NRZI,
; differential input, de-stuffing (ONES, N = 6). Transmit: BAUD 20 + 20 cycles
; (1.5 Mbit/s), NRZI, differential output, stuffing.
; Data memory (lane 0 region, word offsets), written by the host:
;   16       device address (low 7 bits)
;   20       EP0 IN mailbox: [15] staged, [11:8] length (0-8), [7:0] PID byte
;   21..28   EP0 IN data, one byte per word
; Written by the device:
;   32..     bytes of the last DATA packet received (with its 2 CRC bytes)
;   40       PID of the last DATA packet received
; Host-out queue records (tag in [31:24]): 0xA1 token {pid, addr+endp},
; 0xA2 received data packet {pid}, 0xA3 data count, 0xA4 IN data acknowledged.
.equ ADDR, 16
.equ EP0ST, 20
.equ EP0D, 21
.equ RXBUF, 32
.equ RXPID, 40
        SETM  PERIOD2, 20
        SETM  CODE, rx=NRZI, tx=NRZI, data_in=in0, diff_in=1, diff_out=1
        SETM  STUFF, n=6, mode=ONES, enable=1
        SETM  EVC0, a=in0, b=in1, trigger=BOTHLOW, k=20     ; SE0 for 20 cycles (0.5 bit)
        SETM  EVC1, a=in0, level=HIGH, trigger=LEVEL, b=in1, b_must=LOW, k=4   ; J
; ---- wait for a token ------------------------------------------------------
idle:   SETM  TICK, mode=RECOVER
        SETM  PERIOD, 40
        SETM  CRCPOLY, 0x05
        SETM  CRCCFG, width=5, init_ones=1, xorout_ones=1
        EVT   0, idle, on, clear                       ; a keep-alive EOP restarts the search
        STUFFCLR
        LDI   R2, 0x80
        FIELD EXPECT 8, lsb, seek, src=R2              ; SYNC (NRZI 0000_0001)
        FIELD SAMPLE 8, lsb, stuff, dst=R0             ; PID
        EVT   0, idle, off, clear                      ; runs once SYNC has matched
        FIELD SAMPLE 11, lsb, crc, stuff, dst=R1       ; ADDR, ENDP
        FIELD EXPECT 5, stuff, src=CRC                 ; CRC5, msb first: M = good
        WAIT  E0                                       ; the token's EOP has begun (SE0)
        EVT   1, idle, off, clear                      ; forget J seen during the packet
        BR    NM, skip
        LD    R2, [ADDR]
        ALU   MOV R3, R1
        ALU   AND R3, 0x7F
        ALU   CMP R3, R2
        BR    NZ, skip                                 ; not our address
        ALU   CMP R0, 0x69
        BR    Z, tok_in
        ALU   CMP R0, 0x2D
        BR    Z, tok_out                               ; SETUP
        ALU   CMP R0, 0xE1
        BR    Z, tok_out                               ; OUT
skip:   BR    always, idle
; ---- IN token: send the staged packet or NAK -------------------------------
tok_in: LD    R2, [EP0ST]
        SETM  TICK, mode=BAUD                          ; prepare transmit during the EOP
        SETM  PERIOD, 20
        SETM  CRCPOLY, 0x8005
        SETM  CRCCFG, width=16, init_ones=1, xorout_ones=1
        STUFFCLR
        ALU   MOV R1, R2
        ALU   SHR R1, 8
        ALU   AND R1, 0xF                              ; R1 = length
        LDI   R3, EP0D
        ALU   MOV R0, R2
        ALU   AND R2, 0xFF                             ; R2 = PID byte
        ALU   AND R0, 0x8000                           ; Z = nothing staged (kept to BR Z)
        WAIT  E1                                       ; J: EOP over
        PIN   wait=3                                   ; >= 2 bit times before driving
        BR    Z, nak
        PIN   out0=1, out1=0                           ; drive J
        FIELD DRIVE 8, lsb, stuff, src=IMM 0x80        ; SYNC
        FIELD DRIVE 8, lsb, stuff, src=R2              ; DATA0 / DATA1
        ALU   CMP R1, 0
        BR    Z, dcrc
dloop:  FIELD DRIVE 8, lsb, crc, stuff, src=MEM
        LOOP  R1, dloop
dcrc:   FIELD DRIVE 16, stuff, src=CRC                 ; CRC16, msb first
        PIN   out0=0, out1=0, wait=4                   ; EOP: SE0 2 bits
        PIN   out0=1, out1=0, wait=2                   ; J 1 bit
        PIN   out0=z, out1=z
        ; handshake from the host
        SETM  TICK, mode=RECOVER
        SETM  PERIOD, 40
        STUFFCLR
        LDI   R2, 0x80
        FIELD EXPECT 8, lsb, seek, src=R2
        FIELD SAMPLE 8, lsb, stuff, dst=R0
        ALU   CMP R0, 0xD2                             ; ACK
        BR    NZ, skip
        LDI   R2, 0
        ST    R2, [EP0ST]                              ; mailbox consumed
        Q     PUSH R0, tag=0xA4
        BR    always, skip
nak:    PIN   out0=1, out1=0
        FIELD DRIVE 8, lsb, stuff, src=IMM 0x80
        FIELD DRIVE 8, lsb, stuff, src=IMM 0x5A        ; NAK
        PIN   out0=0, out1=0, wait=4
        PIN   out0=1, out1=0, wait=2
        PIN   out0=z, out1=z
        BR    always, idle
; ---- SETUP / OUT token: receive DATA, answer ACK ----------------------------
tok_out: SETM CRCPOLY, 0x8005
        SETM  CRCCFG, width=16, init_ones=1, xorout_ones=1
        Q     PUSH R0, tag=0xA1
        LDI   R3, RXBUF
        WAIT  E1                                       ; token EOP over (J)
        EVT   0, rxend, on, clear                      ; SE0 ends the data packet
        STUFFCLR
        LDI   R2, 0x80
        FIELD EXPECT 8, lsb, seek, src=R2
        FIELD SAMPLE 8, lsb, stuff, dst=R0             ; DATA0 / DATA1
rloop:  FIELD SAMPLE 8, lsb, crc, stuff, dst=MEM
        BR    always, rloop
rxend:  EVT   0, idle, off
        EVT   1, idle, off, clear
        ST    R0, [RXPID]
        LDI   R0, 0x800D                               ; CRC16 residue (raw state)
        LDI   R1, 0
        CRCCHK
        BR    NM, idle                                 ; bad CRC: no handshake
        SETM  TICK, mode=BAUD
        SETM  PERIOD, 20
        STUFFCLR
        WAIT  E1                                       ; J: EOP over
        PIN   wait=3
        PIN   out0=1, out1=0
        FIELD DRIVE 8, lsb, stuff, src=IMM 0x80
        FIELD DRIVE 8, lsb, stuff, src=IMM 0xD2        ; ACK
        PIN   out0=0, out1=0, wait=4
        PIN   out0=1, out1=0, wait=2
        PIN   out0=z, out1=z
        LD    R0, [RXPID]
        Q     PUSH R0, tag=0xA2
        Q     PUSH R3, tag=0xA3
        BR    always, idle
