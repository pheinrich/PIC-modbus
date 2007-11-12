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
   global   Modbus.Address
   global   Modbus.Event
   global   Modbus.Checksum
   global   Modbus.MsgHead
   global   Modbus.MsgTail
   global   Modbus.NoChecksum
   global   Modbus.State

   ; Public Methods
   global   Modbus.buildErrorReply
   global   Modbus.builtin
   global   Modbus.dispatchMsg
   global   Modbus.getFrameByte
   global   Modbus.init
   global   Modbus.isr
   global   Modbus.putFrameByte
   global   Modbus.replyMsg
   global   Modbus.resetFrame
   global   Modbus.unsupported
   global   Modbus.validateMsg

   ; Dependencies
   extern   ASCII.init
   extern   ASCII.isrTimeout
   extern   Diag.diagnostics
   extern   Diag.getEventCount
   extern   Diag.getEventLog
   extern   Diag.init
   extern   RTU.init
   extern   RTU.isrTimeout
   extern   USART.init
   extern   USART.isr
   extern   Util.Frame
   extern   Util.Save
   extern   VTable.dispatch



;; ---------------------------------------------------------------------------
                        udata_acs
;; ---------------------------------------------------------------------------

Modbus.Address          res   1     ; uniquely identifies this device on the bus
Modbus.Event            res   1     ; kRxEvt_CommErr, kRxEvt_Broadcast, kTxEvt_Abort, etc.
Modbus.NoChecksum       res   1     ; 0 = false (default), 255 = true
Modbus.State            res   1     ; current state of the state machine

Modbus.Checksum         res   2     ; LRC or CRC, depending on mode (ASCII or RTU)
Modbus.MsgHead          res   2     ; points to the first byte in the message
Modbus.MsgTail          res   2     ; points to next location to be read or written



;; ---------------------------------------------------------------------------
.modbus                 code
;; ---------------------------------------------------------------------------

BuiltinVTbl:
   data     Modbus.kDiagnostics, Diag.diagnostics
   data     Modbus.kGetEventCount, Diag.getEventCount
   data     Modbus.kGetEventLog, Diag.getEventLog
   data     -1, Modbus.unsupported



;; ----------------------------------------------
;;  void Modbus.buildReply( WREG size )
;;
Modbus.buildReply:
   ; Copy the device id from the original message, as well as the function code,
   movff    Modbus.kRxSlave, Modbus.kTxSlave
   movff    Modbus.kRxFunction, Modbus.kTxFunction
   return



;; ----------------------------------------------
;;  void Modbus.buildErrorReply()
;;
Modbus.buildErrorReply:
   ; Copy the device id from the original message, as well as the function code,
   ; but with the MSb set to indicate an error occured.
   movff    Modbus.kRxSlave, Modbus.kTxSlave
   movff    Modbus.kRxFunction, Modbus.kTxFunction
   movlb    2
   bsf      Modbus.kTxFunction, 7, BANKED

   ; Add the specified exception code to the message.
   movwf    Modbus.kTxErrorCode, BANKED

   ; Based on the exception code, we need to update our event log.
   movlw    Modbus.kErrorMemoryParity
   cpfslt   Modbus.kTxErrorCode, BANKED
     bra    setTail

   movlw    Modbus.kErrorNAKSent
   cpfslt   Modbus.kTxErrorCode, BANKED
     bsf    Modbus.Event, Modbus.kTxEvt_NAKEx

   movlw    Modbus.kErrorBusy
   cpfslt   Modbus.kTxErrorCode, BANKED
     bsf    Modbus.Event, Modbus.kTxEvt_BusyEx

   movlw    Modbus.kErrorFailure
   cpfslt   Modbus.kTxErrorCode, BANKED
     bsf    Modbus.Event, Modbus.kTxEvt_AbortEx
   bsf      Modbus.Event, Modbus.kTxEvt_ReadEx

setTail:
   ; Set the message tail pointer to one byte past the error code.
   movlw    LOW (Modbus.kTxErrorCode + 1)
   movwf    Modbus.MsgTail
   movlw    HIGH (Modbus.kTxErrorCode + 1)
   movwf    Modbus.MsgTail + 1
   return



;; ----------------------------------------------
;;  void Modbus.builtin( frame[0..1] funcKey )
;;
Modbus.builtin:
   SetTableBase BuiltinVTbl
   goto     VTable.dispatch



;; ----------------------------------------------
;;  void Modbus.dispatchMsg( TBLPTR vtable )
;;
Modbus.dispatchMsg:
   movff    Modbus.kRxFunction, Util.Frame
   clrf     Util.Frame + 1
   goto     VTable.dispatch



;; ----------------------------------------------
;;  byte Modbus.getFrameByte()
;;
Modbus.getFrameByte:
   ; Make sure we're not trying to read past the end of the buffer.
   movf     Modbus.MsgHead, W
   cpfseq   Modbus.MsgTail
     bra    getByte

   movf     Modbus.MsgHead + 1, W
   cpfseq   Modbus.MsgTail + 1
     bra    getByte

   ; The head and tail pointers are equal, so we must be done.  Set the carry to
   ; indicate that no byte was read.
   bsf      STATUS, C
   return

getByte:
   ; Not past the end yet, so read the next byte (indirectly).
   CopyWord Modbus.MsgHead, FSR1L
   movf     POSTINC1, W
   CopyWord FSR1L, Modbus.MsgHead

   ; Clear the status flag to indicate the value in W is valid.
   bcf      STATUS, C
   return



;; ----------------------------------------------
;;  void Modbus.init( bool ascii, enum baud, enum parity )
;;
;;  Initializes a state machine appropriate to the operating mode specified,
;;  then resets the diagnostic registers and clears the event log.  Finally,
;;  the hardware USART is initialized with baud rate and software parity
;;  requested.
;;
Modbus.init:
   ; The operating mode determines which state machine will be active.
   movf     Util.Frame, F           ; are we in ASCII (7-bit) mode?
   bz       initRTU                 ; no, initialize RTU mode
   call     ASCII.init              ; yes, initialize ASCII mode
   bra      initDiag

initRTU:
   call     RTU.init

initDiag:
   ; Save the hardware set-up for last, since the state machine initialization
   ; above needs to hook the rx/tx events first.  Note that the stackframe must
   ; be preserved until the then, since it holds the USART initialization para-
   ; meters.
   call     Diag.init               ; reset diagnostic registers and event log
   goto     USART.init              ; initialize the serial port



;; ----------------------------------------------
;;  void Modbus.isr()
;;
Modbus.isr:
   ; Service the serial hardware.  We have already hooked its transmit and re-
   ; ceive interrupt methods, which ensures the correct state machine will get a
   ; chance to process those events.
   call     USART.isr
   
   ; Determine if our timer overflowed.
   btfss    PIE1, TMR1IE            ; are timer1 interrupts enabled?
     return                         ; no, we can exit
   btfss    PIR1, TMR1IF            ; yes, has timer1 expired?
     return                         ; no, we're done

   ; A timer1 event did occur.  We must delegate this event ourselves, since the
   ; USART isn't interested in it and doesn't provide a convenient hook.
   bcf      PIR1, TMR1IF            ; clear the timer interrupt flag
   btfss    RCSTA, RX9              ; are we in RTU (8-bit) mode?
     goto   ASCII.isrTimeout        ; no, the ASCII (7-bit) state machine takes over
   goto     RTU.isrTimeout          ; yes, the RTU state machine takes over



;; ----------------------------------------------
;;  woid Modbus.putFrameByte( byte value )
;;
;;  todo: bounds checking
;;
Modbus.putFrameByte:
   ; Store the byte at the next message buffer location.  A tail pointer keeps
   ; track of where it goes, so we fetch it first, then update its value with the
   ; new tail location when we're finished.
   CopyWord Modbus.MsgTail, FSR1L
   movwf    POSTINC1
   CopyWord FSR1L, Modbus.MsgTail
   return



;; ----------------------------------------------
;;  void Modbus.replyMsg()
;;
;;  Attempts to send the message in the message buffer, which is always in re-
;;  ply to a message we previously received.
;;
Modbus.replyMsg:
   ; This method does nothing if a complete, error-free server request isn't al-
   ; ready available in the message buffer.
   movlw    Modbus.kState_MsgQueued
   cpfseq   Modbus.State            ; is there a message waiting for us?
     return                         ; no, bail

   ; DEBUG copy the received message to the transmit buffer (echo the message).
;   CopyWord Modbus.MsgHead, FSR0L   ; debug
;   movlw    LOW Modbus.kTxBuffer    ; debug
;   movwf    FSR1L                   ; debug
;   movlw    HIGH Modbus.kTxBuffer   ; debug
;   movwf    FSR1H                   ; debug
;                                    ; debug
;copyLoop:                           ; debug
;   movf     FSR0L, W                ; debug
;   cpfseq   Modbus.MsgTail          ; debug
;     bra    copyIt                  ; debug
;                                    ; debug
;   movf     FSR0H, W                ; debug
;   cpfseq   Modbus.MsgTail + 1      ; debug
;     bra    copyIt                  ; debug
;   CopyWord FSR1L, Modbus.MsgTail   ; debug

   ; Change state and enable the character transmitted interrupt.  If the transmit
   ; buffer is empty, this will fire immediately, otherwise it will trigger after
   ; the current byte is transmitted.
   movlw    Modbus.kState_EmitStart ; prepare to transmit the message
   movwf    Modbus.State
   bsf      PIE1, TXIE              ; enable the interrupt
   return

;copyIt:                             ; debug
;   movff    POSTINC0, POSTINC1      ; debug
;   bra      copyLoop                ; debug



;; ----------------------------------------------
;;  void Modbus.resetFrame( char buffer[] )
;;
;;  Initializes the message frame for fresh reception or transmission.  This
;;  method stashes the base pointer for later use by related routines.
;;
Modbus.resetFrame:
   ; Reset the pointer to the beginning of the buffer.
   movf     FSR0L, W
   movwf    Modbus.MsgHead
   movwf    Modbus.MsgTail

   movf     FSR0H, W
   movwf    Modbus.MsgHead + 1
   movwf    Modbus.MsgTail + 1

   ; Reset the event mask, since we're starting from scratch.
   clrf     Modbus.Event
   return



;; ----------------------------------------------
;;  void Modbus.unsupported()
;;
Modbus.unsupported:
   ; The requested function or subfunction code wasn't recognized, so we have no
   ; choice but to return a reply containing exception code 1.
   movlw    Modbus.kErrorBadFunction
   bra      Modbus.buildErrorReply



;; ----------------------------------------------
;;  boolean Modbus.validateMsg()
;;
Modbus.validateMsg:
   ; Verify the message is addressed to this device.
   bsf      Modbus.Event, Modbus.kRxEvt_Broadcast ; assume broadcast message
   movf     Modbus.kRxBuffer, W     ; is this a broadcast message (0 == address)?
   bz       valChecksum             ; yes, validate the checksum

   bcf      Modbus.Event, Modbus.kRxEvt_Broadcast ; no, clear our assumption
   cpfseq   Modbus.Address          ; is it addressed to this specific device?
     retlw  0xff                    ; no, discard frame

valChecksum:
   ; This message is addressed to us.
   bsf      Modbus.Event, Modbus.kRxEvt_SlaveMsg

   ; If checksum validation is turned off, we're done.
   tstfsz   Modbus.NoChecksum
     retlw  0

   ; Set a pointer to the last byte in the message buffer.
   CopyWord Modbus.MsgTail, FSR0L
   bsf      Modbus.Event, Modbus.kRxEvt_CommErr ; assume checksum fails

   ; Compare the checksum included in the message to the one we calculated.
   movf     POSTINC0, W             ; fetch the LSB
   cpfseq   Modbus.Checksum         ; does it match the computed value?
     retlw  0xff                    ; no, discard the frame  TODO: log event

   movf     INDF0, W                ; yes, fetch the MSB
   cpfseq   Modbus.Checksum + 1     ; does it match the computed value?
     retlw  0xff                    ; no, discard the frame  TODO: log event

   ; Success, so clear our earlier assumption about a bad checksum.
   bcf      Modbus.Event, Modbus.kRxEvt_CommErr
   retlw    0



   end
