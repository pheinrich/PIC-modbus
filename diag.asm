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



   #include "private.inc"

   ; Variables
   extern   MODBUS.Event

   global   DIAG.Options

   ; Methods
   global   DIAG.init
   global   DIAG.logListenOnly
   global   DIAG.logRestart
   global   DIAG.logRxEvt
   global   DIAG.logTxEvt



; Option flags
kDiag_RetQuery    equ   7
kDiag_ListenOnly  equ   6
kDiag_Busy        equ   5



;; ---------------------------------------------------------------------------
            udata_acs
;; ---------------------------------------------------------------------------

DIAG.ExceptStatus    res   1
DIAG.NumEvents       res   2     ; not cleared on comm restart
DIAG.Options         res   1
                     ; 1------   ; return query data
                     ; -1-----   ; listen-only mode
                     ; --1----   ; busy
                     ; ---XXXX   ; reserved

DIAG.LogHead         res   1     ; pointer to the oldest event
DIAG.LogTail         res   1     ; pointer to most recent event

DIAG.NumCommErrs     res   2
DIAG.NumExceptErrs   res   2
DIAG.NumMsgs         res   2
DIAG.NumNoResponse   res   2
DIAG.NumOverruns     res   2
DIAG.NumSlaveBusy    res   2
DIAG.NumSlaveMsgs    res   2
DIAG.NumSlaveNAKs    res   2
DIAG.Register        res   2



;; ---------------------------------------------------------------------------
.diag       code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  void DIAG.init()
;;
DIAG.init:
   ; Point to our block of local variables, with total byte length in W.
   lfsr     FSR0, DIAG.ExceptStatus
   movlw    DIAG.Register - DIAG.ExceptStatus + 2

   ; Clear the block.
   clrf     POSTINC0
   decfsz   WREG
     bra    $-4

   return



;; ----------------------------------------------
;;  void DIAG.logListenOnly()
;;
DIAG.logListenOnly:
   movlw    kCmdEvt_ListenOnly
   bra      DIAG.storeLogByte



;; ----------------------------------------------
;;  void DIAG.logRestart()
;;
DIAG.logRestart:
   movlw    kCmdEvt_Restart
   bra      DIAG.storeLogByte



;; ----------------------------------------------
;;  void DIAG.logRxEvt()
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
DIAG.logRxEvt:
   ; Count every message we see on the bus, even if it's not addressed to us.
   INCREG   DIAG.NumMsgs

   ; Count checksum failures.
   btfss    MODBUS.Event, kRxEvt_CommErr
     bra    rxSlave
   INCREG   DIAG.NumCommErrs

rxSlave:
   ; Count messages this device has processed, which means all broadcast messages
   ; and messages addressed to it specifically.
   btfss    MODBUS.Event, kRxEvt_SlaveMsg
     bra    rxNoResponse
   INCREG   DIAG.NumSlaveMsgs
   bcf      MODBUS.Event, kRxEvt_SlaveMsg

rxNoResponse:
   ; Count the messages for which this device returned no response, either normal
   ; or exception.
   btfss    MODBUS.Event, kRxEvt_NoResponse
     bra    rxOverrun
   INCREG   DIAG.NumNoResponse
   bcf      MODBUS.Event, kRxEvt_NoResponse

rxOverrun:
   ; Count buffer overruns.  This will also include framing errors.
   btfss    MODBUS.Event, kRxEvt_Overrun
     bra    rxWrite
   INCREG   DIAG.NumOverruns

rxWrite:
   ; Set the event type and copy the state of the listen-only mode indicator bit.
   bsf      MODBUS.Event, 7
   btfsc    DIAG.Options, kDiag_ListenOnly
     bsf    MODBUS.Event, kRxEvt_ListenOnly

   ; Store the event byte in the log.
   bra      DIAG.storeLogByte



;; ----------------------------------------------
;;  void DIAG.logTxEvt()
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
DIAG.logTxEvt:
   ; Count all exception responses transmitted by this device.
   movlw    (1 << kTxEvt_ReadEx) | (1 << kTxEvt_AbortEx) | (1 << kTxEvt_BusyEx) | (1 << kTxEvt_NAKEx)
   andwf    MODBUS.Event, W
   bz       txBusy
   INCREG   DIAG.NumExceptErrs

txBusy:
   ; Count the messages for which this device returned a "slave busy" exception.
   btfss    MODBUS.Event, kTxEvt_BusyEx
     bra    txNAK
   INCREG   DIAG.NumSlaveBusy

txNAK:
   ; Count the messages for which this device returned a negative acknowledgement
   ; exception.  This seems pathological, since this exception (7) isn't even
   ; documented in the Modbus Application Protocol V1.1a document and it's unclear
   ; how or when a NAK is generated.
   btfss    MODBUS.Event, kTxEvt_NAKEx
     bra    txWrite
   INCREG   DIAG.NumSlaveNAKs

txWrite:
   ; Set the event type and copy the state of the listen-only mode indicator bit.
   bcf      MODBUS.Event, 7
   bsf      MODBUS.Event, 6
   btfsc    DIAG.Options, kDiag_ListenOnly
     bsf    MODBUS.Event, kTxEvt_ListenOnly

   ; Store the event byte in the log.
   bra      DIAG.storeLogByte



;; ----------------------------------------------
;;  void DIAG.storeLogByte( byte evt )
;;
;;  Stores the byte specified in the circular log buffer at the current tail
;;  pointer.  The pointer is advanced, but never more than the maximum buffer
;;  length, 64 (the head pointer may move to compensate).
;;
DIAG.storeLogByte:
   ; Keep track of overall event count, even across comm restarts.
   INCREG   DIAG.NumEvents

   ; Store the event byte at the tail of the buffer.
   lfsr     FSR0, kLogBuffer
   movf     DIAG.LogTail, W
   movff    MODBUS.Event, PLUSW0

   ; Increment the tail pointer, making sure it never exceeds the maximum buffer
   ; length of 64.
   incf     DIAG.LogTail      ; add 1 to the tail pointer
   movlw    kLogBufLen
   cpfslt   DIAG.LogTail      ; is the new tail >= max buffer length?
     clrf   DIAG.LogTail      ; yes, reset to 0

   ; The head pointer may need to be adjusted as well.
   movf     DIAG.LogHead, W
   cpfseq   DIAG.LogTail      ; are the head and tail pointers equal?
     return                   ; no, we're done

   incf     DIAG.LogHead      ; yes, add 1 to the head pointer, too
   movlw    kLogBufLen
   cpfslt   DIAG.LogHead      ; is the new head >= max buffer length?
     clrf   DIAG.LogHead      ; yes, reset to 0

   return



   end
