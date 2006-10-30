;; ---------------------------------------------------------------------------
;;
;;  MODBUS
;;
;;  Copyright © 2006  Peter Heinrich
;;  All Rights Reserved
;;
;;  $URL$
;;  $Revision$
;;
;; ---------------------------------------------------------------------------
;;  $Author$
;;  $Date$
;; ---------------------------------------------------------------------------



; Everyone should agree on the CPU clock speed.
kFrequency  equ   20000000    ; 20 MHz

; Base pointer to beginning of message buffer.
kMsgBuffer  equ   0x0080

; Enumerate the baud rates we support.  Trick: each value is a valid Baud Rate
; Generator (BRG) for the clock speed, above.
kBaud_9600   equ  (kFrequency / (  9600 << 4)) - 1    ; 129 (0x81) @ 20 MHz
kBaud_19200  equ  (kFrequency / ( 19200 << 4)) - 1    ; 64 (0x40) @ 20 MHz
kBaud_57600  equ  (kFrequency / ( 57600 << 4)) - 1    ; 20 (0x14) @ 20 MHz
kBaud_115200 equ  (kFrequency / (115200 << 4)) - 1    ; 9 (0x9) @ 20 MHz

; Enumerate the error checking methods we support.
kParity_Even equ  0
kParity_Odd  equ  1
kParity_None equ  2

; Enumerate our transmission modes.
kMode_RTU   equ   0           ; 8-bit character + start/stop/parity = 11 bits total
kMode_ASCII equ   1           ; 7-bit character + start/stop/parity = 10 bits total

; State machine constants.
kState_Init       equ   0
kState_Idle       equ   1
kState_EmitStart  equ   2
kState_Emission   equ   3
kState_EmitEnd    equ   4
kState_Reception  equ   5
kState_Waiting    equ   6
kState_MsgQueued  equ   7



;; ----------------------------------------------
;;  Macro LDADDR from, to
;;
;;  The LDADDR macro transfers an address word between file registers.  That
;;  is, it copies the copies two registers starting at 'from' to the two
;;  registers starting at 'to'.
;;
LDADDR      macro from, to
   movff    from, to
   movff    from + 1, to + 1
   endm
