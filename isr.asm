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



   #include "modbus.inc"

   extern   CONF.Mode

   extern   ASCII.timeout
   extern   RTU.timeout
   extern   UART.rxCharacter
   extern   UART.txCharacter

   global   ISR.high
   global   ISR.low
   


;; ---------------------------------------------------------------------------
.isr        code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  void ISR.high()
;;
ISR.high:
   ; Determine if the UART is responsible for this interrupt.
   btfss    PIE1, RCIE        ; are character reception interrupts enabled?
     bra    checkTx           ; no, check for transmitted characters
   btfsc    PIR1, RCIF        ; yes, was a character received?
     call   UART.rxCharacter  ; yes, process it

checkTx:
   btfss    PIE1, TXIE        ; are character transmission interrupts enabled?
     bra    checkTimer        ; no, check for timer overflow
   btfsc    PIR1, TXIF        ; yes, was a character transmitted?
     call   UART.txCharacter  ; yes, process it

checkTimer:
   ; Determine if our timer overflowed.
   btfss    PIE1, TMR1IE      ; are timer1 interrupts enabled?
     retfie                   ; no, we can exit
   btfss    PIR1, TMR1IF      ; yes, has timer1 expired?
     retfie                   ; no, we're done

   ; A timer1 event did occur.
   bcf      PIR1, TMR1IF      ; clear the timer interrupt flag
   tstfsz   CONF.Mode         ; are we in RTU mode?
     bra    asciiTimeout      ; no, the ASCII state machine takes over

   call     RTU.timeout       ; yes, the RTU state machine takes over
   retfie

asciiTimeout:
   call     ASCII.timeout
   retfie



;; ----------------------------------------------
;;  void ISR.low()
;;
ISR.low:
   retfie



   end
