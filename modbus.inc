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



; Protocol function codes.
Modbus.kReadCoils             equ   1     ; read/write bits
Modbus.kReadDiscretes         equ   2     ; read-only bits
Modbus.kReadRegisters         equ   3     ; read/write words
Modbus.kReadInputs            equ   4     ; read-only words
Modbus.kWriteCoil             equ   5     ; single bit
Modbus.kWriteRegister         equ   6     ; single word
Modbus.kGetExceptions         equ   7
Modbus.kDiagnostics           equ   8     ; sub-functions below
Modbus.kGetEventCount         equ   11
Modbus.kGetEventLog           equ   12
Modbus.kWriteCoils            equ   15    ; multiple bits
Modbus.kWriteRegisters        equ   16    ; multiple words
Modbus.kGetSlaveId            equ   17
Modbus.kReadFileRecord        equ   20    ; sub-function always 6
Modbus.kWriteFileRecord       equ   21    ; sub-function always 6
Modbus.kWriteRegMask          equ   22
Modbus.kReadWriteRegs         equ   23
Modbus.kReadFIFOQueue         equ   24
Modbus.kMEITransport          equ   43    ; sub-functions below

; Subfunction codes for the Modbus.kDiagnostics function.
Modbus.kDiagReturnQuery       equ   0
Modbus.kDiagRestartComm       equ   1
Modbus.kDiagGetRegister       equ   2
Modbus.kDiagSetDelim          equ   3
Modbus.kDiagSetListenOnly     equ   4
Modbus.kDiagClear             equ   10
Modbus.kDiagGetMsgCount       equ   11
Modbus.kDiagGetErrorCount     equ   12
Modbus.kDiagGetExceptCount    equ   13
Modbus.kDiagGetSlaveMsgCount  equ   14
Modbus.kDiagGetNoRespCount    equ   15
Modbus.kDiagGetNAKCount       equ   16
Modbus.kDiagGetBusyCount      equ   17
Modbus.kDiagGetOverrunCount   equ   18
Modbus.kDiagClearOverrun      equ   20

; Subfunction codes for the Modbus.kEncapsulated function.
Modbus.kEncapCANopen          equ   13
Modbus.kEncapGetDeviceId      equ   14

; Device id codes.
Modbus.kDevIdBasic            equ   1
Modbus.kDevIdRegular          equ   2
Modbus.kDevIdExtended         equ   3
Modbus.kDevIdIndividual       equ   4

; Device id objects.
Modbus.kDevIdBasic_First      equ   0
Modbus.kDevIdBasic_Last       equ   2
Modbus.kDevIdRegular_First    equ   3
Modbus.kDevIdRegular_Last     equ   127
Modbus.kDevIdExtended_First   equ   128
Modbus.kDevIdExtended_Last    equ   255
Modbus.kDevIdReserved_First   equ   7
Modbus.kDevIdReserved_Last    equ   127

; Possible exception codes.
Modbus.kErrorBadFunction      equ   1
Modbus.kErrorBadAddress       equ   2
Modbus.kErrorBadData          equ   3
Modbus.kErrorFailure          equ   4
Modbus.kErrorAcknowledge      equ   5
Modbus.kErrorBusy             equ   6
Modbus.kErrorNAKSent          equ   7
Modbus.kErrorMemoryParity     equ   8
Modbus.kErrorGateway          equ   10

; State machine constants.
Modbus.kState_Init            equ   0
Modbus.kState_Idle            equ   1
Modbus.kState_EmitStart       equ   2
Modbus.kState_Emission        equ   3
Modbus.kState_EmitEnd         equ   4
Modbus.kState_EmitDone        equ   5
Modbus.kState_Reception       equ   6
Modbus.kState_Waiting         equ   7
Modbus.kState_MsgQueued       equ   8

; Bit fields for receive events in the event log.
Modbus.kRxEvt_CommErr         equ   1
Modbus.kRxEvt_SlaveMsg        equ   2     ; extended field
Modbus.kRxEvt_NoResponse      equ   3     ; extended field
Modbus.kRxEvt_Overrun         equ   4
Modbus.kRxEvt_ListenOnly      equ   5
Modbus.kRxEvt_Broadcast       equ   6

; Bit fields for transmit events in the event log.
Modbus.kTxEvt_ReadEx          equ   0
Modbus.kTxEvt_AbortEx         equ   1
Modbus.kTxEvt_BusyEx          equ   2
Modbus.kTxEvt_NAKEx           equ   3
Modbus.kTxEvt_Timeout         equ   4
Modbus.kTxEvt_ListenOnly      equ   5

; Dedicated event types in the event log.
Modbus.kCmdEvt_ListenOnly     equ   0x04
Modbus.kCmdEvt_Restart        equ   0x00

; Option flags
Modbus.kDiag_ListenOnly       equ   7
Modbus.kDiag_Busy             equ   6
Modbus.kDiag_DontCountEvent   equ   5

; Base pointers to special buffers.  ASCII mode requires special buffer
; handling, since we don't have enough memory for two independent buffers.
; However, since we convert ASCII messages into RTU mode for processing any-
; way, we don't actually need separate buffers.  We do a reverse conversion
; just before transmission, back into the receive buffer.
Modbus.kLogBuffer             equ   0x00be
Modbus.kLogBufLen             equ   0x0041

Modbus.kRxBuffer              equ   Modbus.kLogBuffer + Modbus.kLogBufLen
Modbus.kRxBufLen              equ   0x100

Modbus.kTxBuffer              equ   Modbus.kRxBuffer + Modbus.kRxBufLen
Modbus.kTxBufLen              equ   Modbus.kRxBufLen

Modbus.kASCIIBuffer           equ   Modbus.kRxBuffer
Modbus.kASCIIBufLen           equ   0x201

; Significant offsets within the receive/transmit buffers.
Modbus.kRxSlave               equ   Modbus.kRxBuffer
Modbus.kRxFunction            equ   Modbus.kRxBuffer + 1
Modbus.kRxSubFunction         equ   Modbus.kRxBuffer + 2
Modbus.kRxDelimiter           equ   Modbus.kRxBuffer + 4
Modbus.kRxReadDeviceIdCode    equ   Modbus.kRxBuffer + 4
Modbus.kRxObjectId            equ   Modbus.kRxBuffer + 5

Modbus.kTxSlave               equ   Modbus.kTxBuffer
Modbus.kTxFunction            equ   Modbus.kTxBuffer + 1
Modbus.kTxErrorCode           equ   Modbus.kTxBuffer + 2
Modbus.kTxSubFunction         equ   Modbus.kTxBuffer + 2
Modbus.kTxStatus              equ   Modbus.kTxBuffer + 2
Modbus.kTxByteCount           equ   Modbus.kTxBuffer + 2
Modbus.kTxEventCount          equ   Modbus.kTxBuffer + 4
