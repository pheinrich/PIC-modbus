;; ---------------------------------------------------------------------------
;;
;;  Modbus
;;
;;  Copyright � 2006,7  Peter Heinrich
;;  All Rights Reserved
;;
;;  $URL$
;;  $Revision$
;;
;; ---------------------------------------------------------------------------
;;  $Author$
;;  $Date$
;; ---------------------------------------------------------------------------



   #include "private.inc"

   ; Global Variables
   global   Diag.Options

   ; Public Methods
   global   Diag.diagnostics
   global   Diag.getEventCount
   global   Diag.getEventLog
   global   Diag.init
   global   Diag.logListenOnly
   global   Diag.logRestart
   global   Diag.logRxEvt
   global   Diag.logTxEvt

   ; Dependencies
   extern   Modbus.Event
   extern   Modbus.unsupported
   extern   Util.Frame
   extern   VTable.dispatch



;; ---------------------------------------------------------------------------
                        udata_acs
;; ---------------------------------------------------------------------------

Diag.ExceptStatus       res   1
Diag.NumEvents          res   2     ; not cleared on comm restart
Diag.Options            res   1
                        ; 1-------  return query data
                        ; -1------  listen-only mode
                        ; --1-----  busy
                        ; ---XXXXX  reserved

Diag.LogHead            res   1     ; pointer to next write position
Diag.LogTail            res   1     ; pointer to next read position

Diag.NumCommErrs        res   2
Diag.NumExceptErrs      res   2
Diag.NumMsgs            res   2
Diag.NumNoResponse      res   2
Diag.NumOverruns        res   2
Diag.NumSlaveBusy       res   2
Diag.NumSlaveMsgs       res   2
Diag.NumSlaveNAKs       res   2
Diag.Register           res   2



;; ---------------------------------------------------------------------------
.diag                   code
;; ---------------------------------------------------------------------------

DiagnosticsVTbl:
   data     Modbus.kDiagReturnQuery, returnQuery
   data     Modbus.kDiagRestartComm, restartComm
   data     Modbus.kDiagGetRegister, getRegister
   data     Modbus.kDiagSetDelim, setDelim
   data     Modbus.kDiagSetListenOnly, setListenOnly
   data     Modbus.kDiagClear, clear
   data     Modbus.kDiagGetMsgCount, getMsgCount
   data     Modbus.kDiagGetErrorCount, getErrorCount
   data     Modbus.kDiagGetExceptCount, getExceptCount
   data     Modbus.kDiagGetSlaveMsgCount, getSlaveMsgCount
   data     Modbus.kDiagGetNoRespCount, getNoRespCount
   data     Modbus.kDiagGetNAKCount, getNAKCount
   data     Modbus.kDiagGetBusyCount, getBusyCount
   data     Modbus.kDiagGetOverrunCount, getOverrunCount
   data     Modbus.kDiagClearOverrun, clearOverrun
   data     -1, Modbus.unsupported



;; ----------------------------------------------
;;  void Diag.diagnostics()
;;
Diag.diagnostics:
   movff    Modbus.kRxSubFunction, Util.Frame
   movff    Modbus.kRxSubFunction + 1, Util.Frame + 1
   SetTableBase DiagnosticsVTbl
   goto     VTable.dispatch



;; ----------------------------------------------
;;  void Diag.getEventCount()
;;
Diag.getEventCount:
   return



;; ----------------------------------------------
;;  void Diag.getEventLog()
;;
Diag.getEventLog:
   return



;; ----------------------------------------------
;;  void Diag.init()
;;
Diag.init:
   ; Point to our block of local variables, with total byte length in W.
   lfsr     FSR0, Diag.ExceptStatus
   movlw    Diag.Register - Diag.ExceptStatus + 2

   ; Clear the block.
   clrf     POSTINC0
   decfsz   WREG, F
     bra    $-4

   return



;; ----------------------------------------------
;;  void Diag.logListenOnly()
;;
Diag.logListenOnly:
   movlw    Modbus.kCmdEvt_ListenOnly
   bra      Diag.storeLogByte



;; ----------------------------------------------
;;  void Diag.logRestart()
;;
Diag.logRestart:
   movlw    Modbus.kCmdEvt_Restart
   bra      Diag.storeLogByte



;; ----------------------------------------------
;;  void Diag.logRxEvt()
;;
;;  Adds an entry to the event log corresponding to a receive event.  The
;;  entry is an 8-bit bitfield:
;;
;;    1-------    ; always 1
;;    -1------    ; broadcast received (address = 0)
;;    --1-----    ; currently in listen-only mode
;;    ---1----    ; character overrun (buffer overflow)
;;    ----XX--    ; not used
;;    ------1-    ; communication error (bad checksum)
;;    -------X    ; not used
;;
Diag.logRxEvt:
   ; Count every message we see on the bus, even if it's not addressed to us.
   IncrementWord Diag.NumMsgs

   ; Count checksum failures.
   btfss    Modbus.Event, Modbus.kRxEvt_CommErr
     bra    rxSlave
   IncrementWord Diag.NumCommErrs

rxSlave:
   ; Count messages this device has processed, which means all broadcast messages
   ; and messages addressed to it specifically.
   btfss    Modbus.Event, Modbus.kRxEvt_SlaveMsg
     bra    rxNoResponse
   IncrementWord Diag.NumSlaveMsgs
   bcf      Modbus.Event, Modbus.kRxEvt_SlaveMsg

rxNoResponse:
   ; Count the messages for which this device returned no response, either normal
   ; or exception.
   btfss    Modbus.Event, Modbus.kRxEvt_NoResponse
     bra    rxOverrun
   IncrementWord Diag.NumNoResponse
   bcf      Modbus.Event, Modbus.kRxEvt_NoResponse

rxOverrun:
   ; Count buffer overruns.  This will also include framing errors.
   btfss    Modbus.Event, Modbus.kRxEvt_Overrun
     bra    rxWrite
   IncrementWord Diag.NumOverruns

rxWrite:
   ; Set the event type and copy the state of the listen-only mode indicator bit.
   bsf      Modbus.Event, 7
   btfsc    Diag.Options, Modbus.kDiag_ListenOnly
     bsf    Modbus.Event, Modbus.kRxEvt_ListenOnly

   ; Store the event byte in the log.
   bra      Diag.storeLogByte



;; ----------------------------------------------
;;  void Diag.logTxEvt()
;;
;;  Adds an entry to the event log corresponding to a transmit event.  The
;;  entry is an 8-bit bitfield:
;;
;;    01------    ; always 01
;;    --1-----    ; currently in listen-only mode
;;    ---1----    ; a write timeout occurred
;;    ----1---    ; slave program NAK exception sent (exception code 7)
;;    -----1--    ; slave busy exception sent (exception codes 5-6)
;;    ------1-    ; slave abort exception sent (exception code 4)
;;    -------1    ; read exception sent (exception codes 1-3)
;;
Diag.logTxEvt:
   ; Count all exception responses transmitted by this device.
   movlw    (1 << Modbus.kTxEvt_ReadEx) | (1 << Modbus.kTxEvt_AbortEx) | (1 << Modbus.kTxEvt_BusyEx) | (1 << Modbus.kTxEvt_NAKEx)
   andwf    Modbus.Event, W
   bz       txBusy
   IncrementWord Diag.NumExceptErrs

txBusy:
   ; Count the messages for which this device returned a "slave busy" exception.
   btfss    Modbus.Event, Modbus.kTxEvt_BusyEx
     bra    txNAK
   IncrementWord Diag.NumSlaveBusy

txNAK:
   ; Count the messages for which this device returned a negative acknowledgement
   ; exception.  This seems pathological, since this exception (7) isn't even
   ; documented in the Modbus Application Protocol V1.1a document and it's unclear
   ; how or when a NAK is generated.
   btfss    Modbus.Event, Modbus.kTxEvt_NAKEx
     bra    txWrite
   IncrementWord Diag.NumSlaveNAKs

txWrite:
   ; Set the event type and copy the state of the listen-only mode indicator bit.
   bcf      Modbus.Event, 7
   bsf      Modbus.Event, 6
   btfsc    Diag.Options, Modbus.kDiag_ListenOnly
     bsf    Modbus.Event, Modbus.kTxEvt_ListenOnly

   ; Store the event byte in the log.
   bra      Diag.storeLogByte



;; ----------------------------------------------
;;  void Diag.storeLogByte()
;;
;;  Stores the current Modbus.Event byte in the circular log buffer.  The tail
;;  pointer is advanced, but never more than the maximum buffer length, 64
;;  (the head pointer may move to compensate).
;;
Diag.storeLogByte:
   ; Keep track of overall event count, even across comm restarts.
   IncrementWord Diag.NumEvents

   ; Store the event byte at the tail of the buffer.
   lfsr     FSR0, Modbus.kLogBuffer
   movf     Diag.LogHead, W
   movff    Modbus.Event, PLUSW0

   ; Increment the head pointer, making sure it never exceeds the maximum buffer
   ; length of 65 (one more than we need, to avoid circular buffer difficulties).
   incf     Diag.LogHead, F         ; add 1 to the head pointer
   movlw    Modbus.kLogBufLen
   cpfslt   Diag.LogHead            ; is the new head >= max buffer length?
     clrf   Diag.LogHead            ; yes, reset to 0

   ; The tail pointer may need to be adjusted as well.
   movf     Diag.LogTail, W
   cpfseq   Diag.LogHead            ; are the head and tail pointers equal?
     return                         ; no, we're done

   incf     Diag.LogTail, F         ; yes, add 1 to the tail pointer, too
   movlw    Modbus.kLogBufLen
   cpfslt   Diag.LogTail            ; is the new tail >= max buffer length?
     clrf   Diag.LogTail            ; yes, reset to 0

   return



;; ----------------------------------------------
;;
;;
clear:
   return



;; ----------------------------------------------
;;
;;
clearOverrun:
   return



;; ----------------------------------------------
;;
;;
getBusyCount:
   return



;; ----------------------------------------------
;;
;;
getErrorCount:
   return



;; ----------------------------------------------
;;  WREG getEventCount()
;;
;;  Returns the number of events currently stored in the circular event log
;;  buffer, computed as the difference between the head and tail pointers.
;;
getEventCount:
   movf     Diag.LogHead, W
   subwf    Diag.LogTail, W
   btfsc    STATUS, N
     addlw  Modbus.kLogBufLen
   return



;; ----------------------------------------------
;;
;;
getExceptCount:
   return



;; ----------------------------------------------
;;
;;
getMsgCount:
   return



;; ----------------------------------------------
;;
;;
getNAKCount:
   return



;; ----------------------------------------------
;;
;;
getNoRespCount:
   return



;; ----------------------------------------------
;;
;;
getOverrunCount:
   return



;; ----------------------------------------------
;;
;;
getRegister:
   return



;; ----------------------------------------------
;;
;;
getSlaveMsgCount:
   return



;; ----------------------------------------------
;;
;;
restartComm:
   return



;; ----------------------------------------------
;;
;;
returnQuery:
   return



;; ----------------------------------------------
;;
;;
setDelim:
   return



;; ----------------------------------------------
;;
;;
setListenOnly:
   return



   end
