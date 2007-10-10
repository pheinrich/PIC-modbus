;; ---------------------------------------------------------------------------
;;
;;  Modbus
;;
;;  Copyright © 2006,7  Peter Heinrich
;;  All Rights Reserved
;;
;;  $URL$
;;  $Revision$
;;
;; ---------------------------------------------------------------------------
;;  $Author$
;;  $Date$
;; ---------------------------------------------------------------------------



; State machine constants.
Modbus.kState_Init          equ   0
Modbus.kState_Idle          equ   1
Modbus.kState_EmitStart     equ   2
Modbus.kState_Emission      equ   3
Modbus.kState_EmitEnd       equ   4
Modbus.kState_EmitDone      equ   5
Modbus.kState_Reception     equ   6
Modbus.kState_Waiting       equ   7
Modbus.kState_MsgQueued     equ   8

; Bit fields for receive events in the event log.
Modbus.kRxEvt_CommErr       equ   1
Modbus.kRxEvt_SlaveMsg      equ   2  ; extended field
Modbus.kRxEvt_NoResponse    equ   3  ; extended field
Modbus.kRxEvt_Overrun       equ   4
Modbus.kRxEvt_ListenOnly    equ   5
Modbus.kRxEvt_Broadcast     equ   6

; Bit fields for transmit events in the event log.
Modbus.kTxEvt_ReadEx        equ   0
Modbus.kTxEvt_AbortEx       equ   1
Modbus.kTxEvt_BusyEx        equ   2
Modbus.kTxEvt_NAKEx         equ   3
Modbus.kTxEvt_Timeout       equ   4
Modbus.kTxEvt_ListenOnly    equ   5

; Dedicated event types in the event log.
Modbus.kCmdEvt_ListenOnly   equ   0x04
Modbus.kCmdEvt_Restart      equ   0x00

; Option flags
Modbus.kDiag_RetQuery    equ   7
Modbus.kDiag_ListenOnly  equ   6
Modbus.kDiag_Busy        equ   5

; Base pointers to special buffers.  ASCII mode requires special buffer
; handling, since we don't have enough memory for two independent buffers.
; However, since we convert ASCII messages into RTU mode for processing any-
; way, we don't actually need separate buffers.  We do a reverse conversion
; just before transmission, back into the receive buffer.
Modbus.kLogBuffer     equ   0x0060
Modbus.kLogBufLen     equ   0x0040

Modbus.kRxBuffer      equ   Modbus.kLogBuffer + Modbus.kLogBufLen
Modbus.kRxBufLen      equ   0x100

Modbus.kTxBuffer      equ   Modbus.kRxBuffer + Modbus.kRxBufLen
Modbus.kTxBufLen      equ   Modbus.kRxBufLen

Modbus.kASCIIBuffer   equ   Modbus.kRxBuffer
Modbus.kASCIIBufLen   equ   0x201
