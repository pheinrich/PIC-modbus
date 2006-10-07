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
   btfsc    PIR1, RCIF        ; was a character received?
     call   UART.rxCharacter  ; yes, process it

   ; Determine if our timer overflowed.
   btfss    PIR1, TMR1IF      ; has timer1 expired?
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
