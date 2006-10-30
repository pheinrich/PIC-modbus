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
   extern   MODBUS.Mode

   ; Methods
   extern   ASCII.timeout
   extern   RTU.timeout
   extern   UART.rxCharacter
   extern   UART.txCharacter

   global   ISR.modbus
   


;; ---------------------------------------------------------------------------
.isr        code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  void ISR.modbus()
;;
ISR.modbus:
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
     return                   ; no, we can exit
   btfss    PIR1, TMR1IF      ; yes, has timer1 expired?
     return                   ; no, we're done

   ; A timer1 event did occur.
   bcf      PIR1, TMR1IF      ; clear the timer interrupt flag
   tstfsz   MODBUS.Mode       ; are we in RTU mode?
     goto   ASCII.timeout     ; no, the ASCII state machine takes over
   goto     RTU.timeout       ; yes, the RTU state machine takes over


   end
