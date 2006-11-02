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



;; ---------------------------------------------------------------------------
            udata_acs
;; ---------------------------------------------------------------------------

DIAG.Options      res   1
                  ; 1------   ; return query data
                  ; -1-----   ; listen-only mode
                  ; --1----   ; busy
                  ; ---XXXX   ; reserved

DIAG.LogHead      res   1     ; pointer to the oldest event
DIAG.LogTail      res   1     ; pointer to most recent event

DIAG.ExceptStatus res   1
DIAG.Register     res   2




;; ---------------------------------------------------------------------------
.diag       code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  void DIAG.init()
;;
DIAG.init:
   clrf     DIAG.Options
   clrf     DIAG.LogHead
   clrf     DIAG.LogTail
   clrf     DIAG.ExceptStatus
   clrf     DIAG.Register
   return



;; ----------------------------------------------
;;  void DIAG.logListenOnly()
;;
DIAG.logListenOnly:
   movlw    0x4
   bra      DIAG.storeLogByte



;; ----------------------------------------------
;;  void DIAG.logRestart()
;;
DIAG.logRestart:
   movlw    0x0
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
   ; Store the event byte at the tail of the buffer.
   lfsr     FSR0, kLogBuffer
   movf     DIAG.LogTail, W
   movff    MODBUS.Event, PLUSW0

   ; Increment the tail pointer, making sure it never exceeds the maximum buffer
   ; length of 64.
   incf     DIAG.LogTail      ; add 1 to the tail pointer
   movlw    0x40
   cpfslt   DIAG.LogTail      ; is the new tail >= max buffer length?
     clrf   DIAG.LogTail      ; yes, reset to 0

   ; The head pointer may need to be adjusted as well.
   movf     DIAG.LogHead, W
   cpfseq   DIAG.LogTail      ; are the head and tail pointers equal?
     return                   ; no, we're done

   incf     DIAG.LogHead      ; yes, add 1 to the head pointer, too
   movlw    0x40
   cpfslt   DIAG.LogHead      ; is the new head >= max buffer length?
     clrf   DIAG.LogHead      ; yes, reset to 0

   return



   end
