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

   extern   CONF.BaudRate
   extern   CONF.Mode
   extern   CONF.ParityCheck

   extern   ASCII.rxCharacter
   extern   RTU.rxCharacter

   global   UART.CharacterErrors
   global   UART.ParityErrors

   global   UART.init
   global   UART.rxCharacter
   global   UART.txCharacter



;; ---------------------------------------------------------------------------
            udata_acs
;; ---------------------------------------------------------------------------

UART.CharacterErrors   res   1
UART.ParityErrors      res   1



;; ---------------------------------------------------------------------------
.uart       code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  void UART.init()
;;
;;  Sets the initial parameters for the serial port hardware.  Modbus has
;;  strict requirements when it comes to serial line protocol compliance,
;;  such as supported baud rates, parity, character length, etc., but here
;;  we're just initializing the hardware to handle asynchronous reception
;;  and transmission.
;;
UART.init:
   ; Specify the baud rate.
   movff    CONF.BaudRate, SPBRG

   ; Specify how data is transmitted.
   movlw    b'01100100'
            ; X-------        ; [not used in asynchronous mode]
            ; -1------ TX9    ; assume 9-bit characters (8 data + 1 parity)
            ; --1----- TXEN   ; enable the transmitter
            ; ---0---- SYNC   ; be asynchronous
            ; ----X---        ; [unimplemented]
            ; -----1-- BRGH   ; use high-speed baud rate
            ; ------X-        ; [read-only shift register status]
            ; -------X        ; [used only during actual transmission]
   movwf    TXSTA

   ; Specify how data is received.
   movlw    b'11010000'
            ; 1------- SPEN   ; enable the serial port
            ; -1------ RX9    ; assume 9-bit characters (8 data + 1 parity)
            ; --X-----        ; [not used in asynchronous mode]
            ; ---1---- CREN   ; enable receiver
            ; ----0--- ADDEN  ; don't perform special address detection
            ; -----XXX        ; [read-only status/data bits]
   movwf    RCSTA

   ; Set the I/O direction for the RX and TX pins.
   bcf      TRISC, RC6        ; RC6/TX/CK will be an output
   bsf      TRISC, RC7        ; RC7/RX/DT will be an input

   ; Test our 9-bit character assumption.
   movf     CONF.Mode
   bz       initInts          ; if correct, we're done

   bcf      TXSTA, TX9        ; otherwise, use 8-bit characters (7 data + 1 parity)
   bcf      RCSTA, RX9

initInts:
   ; Enable interrupts.
   bsf      PIE1, RCIE        ; character received
   bsf      PIE1, TXIE        ; character transmitted

   return



;; ----------------------------------------------
;;  void UART.rxCharacter()
;;
;;  Processes a single character after it arrives via the UART.  Framing
;;  errors and input buffer overflows are detected here.  Parity checking
;;  (if any) is done by code appropriate to the current mode (RTU or ASCII).
;;  If all is well, mode then determines which state machine processes the
;;  received character.
;;
UART.rxCharacter:
   ; Mask the receiver status register to examine the error bits.
   movlw    FERR | OERR
   andwf    RCSTA, W          ; was there a framing error or buffer overflow?
   bz       rxChar            ; no, update the state machine

   ; There was an error, which we must clear and record.
   bcf      RCSTA, CREN
   bsf      RCSTA, CREN
   incf     UART.CharacterErrors

rxChar:
   ; Update the appropriate state machine.
   tstfsz   CONF.Mode         ; are we in RTU transmission mode?
     goto   ASCII.rxCharacter ; no, process an ASCII character
   goto     RTU.rxCharacter   ; yes, process a binary character



;; ----------------------------------------------
;;  void UART.txCharacter()
;;
UART.txCharacter:
   return



   end
