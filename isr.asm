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
   


;; ---------------------------------------------------------------------------
            udata_acs
;; ---------------------------------------------------------------------------

ISR.SaveFSR0      res   2
ISR.SaveFSR1      res   2
ISR.SaveFSR2      res   2



;; ---------------------------------------------------------------------------
.isr        code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  void ISR.high()
;;
ISR.high:
   ; Save registers that may be overwritten during processing.
   LDADDR   FSR0L, ISR.SaveFSR0
   LDADDR   FSR1L, ISR.SaveFSR1
   LDADDR   FSR2L, ISR.SaveFSR2

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
     bra    exit              ; no, we can exit
   btfss    PIR1, TMR1IF      ; yes, has timer1 expired?
     bra    exit              ; no, we're done

   ; A timer1 event did occur.
   bcf      PIR1, TMR1IF      ; clear the timer interrupt flag
   tstfsz   CONF.Mode         ; are we in RTU mode?
     bra    asciiTimeout      ; no, the ASCII state machine takes over

   call     RTU.timeout       ; yes, the RTU state machine takes over
   bra      exit

asciiTimeout:
   call     ASCII.timeout

exit:
   ; Restore the registers we saved.
   LDADDR   ISR.SaveFSR2, FSR2L
   LDADDR   ISR.SaveFSR1, FSR1L
   LDADDR   ISR.SaveFSR0, FSR0L
   
   retfie   FAST



   end
