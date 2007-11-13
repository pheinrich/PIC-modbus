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



   #include "private.inc"

   ; Global Variables
   global   Modbus.Address
   global   Modbus.Event
   global   Modbus.NoChecksum
   global   Modbus.State

   ; Public Methods
   global   Modbus.builtin
   global   Modbus.dispatchMsg
   global   Modbus.init
   global   Modbus.isr
   global   Modbus.replyMsg
   global   Modbus.unsupported

   ; Dependencies
   extern   ASCII.init
   extern   ASCII.isrTimeout
   extern   Diag.diagnostics
   extern   Diag.getEventCount
   extern   Diag.getEventLog
   extern   Diag.init
   extern   Frame.begin
   extern   Frame.end
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



;; ---------------------------------------------------------------------------
.modbus                 code
;; ---------------------------------------------------------------------------

BuiltinVTbl:
   data     Modbus.kDiagnostics, Diag.diagnostics
   data     Modbus.kGetEventCount, Diag.getEventCount
   data     Modbus.kGetEventLog, Diag.getEventLog
   data     -1, Modbus.unsupported



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
;   CopyWord Frame.Tail, FSR0L       ; debug
;   movlw    LOW Modbus.kTxBuffer    ; debug
;   movwf    FSR1L                   ; debug
;   movlw    HIGH Modbus.kTxBuffer   ; debug
;   movwf    FSR1H                   ; debug
;                                    ; debug
;copyLoop:                           ; debug
;   movf     FSR0L, W                ; debug
;   cpfseq   Frame.Head              ; debug
;     bra    copyIt                  ; debug
;                                    ; debug
;   movf     FSR0H, W                ; debug
;   cpfseq   Frame.Head + 1          ; debug
;     bra    copyIt                  ; debug
;   CopyWord FSR1L, Frame.Head       ; debug

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
;;  void Modbus.unsupported()
;;
Modbus.unsupported:
   ; The requested function or subfunction code wasn't recognized, so we have no
   ; choice but to return a reply containing exception code 1.
   movlw    Modbus.kErrorBadFunction
   call     Frame.begin
   goto     Frame.end



   end
