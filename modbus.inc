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
kFrequency  equ   24000000    ; 24 MHz

; Enumerate the baud rates we support.  Trick: each value is a valid Baud Rate
; Generator (BRG) for the clock speed, above.
kBaud_9600   equ  (kFrequency / (  9600 << 4)) - 1    ; 155 (0x9b) @ 24 MHz
kBaud_19200  equ  (kFrequency / ( 19200 << 4)) - 1    ; 77 (0x4d) @ 24 MHz
kBaud_57600  equ  (kFrequency / ( 57600 << 4)) - 1    ; 25 (0x19) @ 24 MHz
kBaud_115200 equ  (kFrequency / (115200 << 4)) - 1    ; 12 (0xc) @ 24 MHz

; Enumerate the error checking methods we support.
kParity_Even equ  0
kParity_Odd  equ  1
kParity_None equ  2

; Enumerate our transmission modes.
kMode_RTU   equ   0           ; 8-bit character + start/stop/parity = 11 bits total
kMode_ASCII equ   1           ; 7-bit character + start/stop/parity = 10 bits total

; State machine constants.
kState_Init          equ   0
kState_Idle          equ   1
kState_EmitStart     equ   2
kState_Emission      equ   3
kState_EmitEnd       equ   4
kState_EmitDone      equ   5
kState_Reception     equ   6
kState_Waiting       equ   7
kState_MsgQueued     equ   8

; Bit fields for receive events in the event log.
kRxEvt_CommErr       equ   1
kRxEvt_SlaveMsg      equ   2  ; extended field
kRxEvt_NoResponse    equ   3  ; extended field
kRxEvt_Overrun       equ   4
kRxEvt_ListenOnly    equ   5
kRxEvt_Broadcast     equ   6

; Bit fields for transmit events in the event log.
kTxEvt_ReadEx        equ   0
kTxEvt_AbortEx       equ   1
kTxEvt_BusyEx        equ   2
kTxEvt_NAKEx         equ   3
kTxEvt_Timeout       equ   4
kTxEvt_ListenOnly    equ   5

; Dedicated event types in the event log.
kCmdEvt_ListenOnly   equ   0x04
kCmdEvt_Restart      equ   0x00

; Option flags
kDiag_RetQuery    equ   7
kDiag_ListenOnly  equ   6
kDiag_Busy        equ   5

; Base pointers to special buffers.  ASCII mode requires special buffer
; handling, since we don't have enough memory for two independent buffers.
; However, since we convert ASCII messages into RTU mode for processing any-
; way, we don't actually need separate buffers.  We do a reverse conversion
; just before transmission, back into the receive buffer.
kLogBuffer     equ   0x0060
kLogBufLen     equ   0x0040

kRxBuffer      equ   kLogBuffer + kLogBufLen
kRxBufLen      equ   0x100

kTxBuffer      equ   kRxBuffer + kRxBufLen
kTxBufLen      equ   kRxBufLen

kASCIIBuffer   equ   kRxBuffer
kASCIIBufLen   equ   0x201



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



;; ----------------------------------------------
;;  Macro INCREG register
;;
;;  This macro increments the register argument by 1, assuming it refers to a
;;  16-bit little-endian word composed of two file registers in the access
;;  bank.
;;
INCREG      macro register
   infsnz   register
     incf   register + 1
   endm
