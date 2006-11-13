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
   extern   MODBUS.BaudRate
   extern   MODBUS.Event
   extern   MODBUS.FrameError
   extern   MODBUS.Mode

   global   UART.LastCharacter
   global   UART.LastParity

   ; Methods
   extern   ASCII.rxCharacter
   extern   ASCII.txCharacter
   extern   RTU.rxByte
   extern   RTU.txByte

   global   UART.init
   global   UART.rxCharacter
   global   UART.txCharacter



;; ---------------------------------------------------------------------------
            udata_acs
;; ---------------------------------------------------------------------------

UART.LastCharacter      res   1
UART.LastParity         res   1



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
   movff    MODBUS.BaudRate, SPBRG

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
   movf     MODBUS.Mode
   bz       initInts          ; if correct, we're done

   bcf      TXSTA, TX9        ; otherwise, use 8-bit characters (7 data + 1 parity)
   bcf      RCSTA, RX9

initInts:
   ; Enable interrupts.
   bsf      PIE1, RCIE        ; character received
;   bsf      PIE1, TXIE        ; character transmitted

   return



;; ----------------------------------------------
;;  void UART.rxCharacter()
;;
;;  Processes a single character after it arrives via the UART.  Framing
;;  errors and input buffer overflows are detected here (parity checking, if
;;  any, is done by code appropriate to the current mode--RTU or ASCII).  If
;;  all is well, mode then determines which state machine processes the re-
;;  ceived character.
;;
;;  To prevent illegal reads of the reception buffer register, this method
;;  initializes UART.LastCharacter with that value, as appropriate.  Similar-
;;  ly, UART.LastParity will hold the parity of the last character received.
;;
UART.rxCharacter:
   ; Mask the receiver status register to examine the error bits.
   movlw    (1 << FERR) | (1 << OERR)
   andwf    RCSTA, W          ; was there a framing error or buffer overflow?
   bz       rxChar            ; no, update the state machine

   ; There was an error, which we must clear and record.  Afterward, we continue
   ; processing the byte as normal.
   bcf      RCSTA, CREN       ; swizzle the bit to clear the error
   bsf      RCSTA, CREN
   bsf      MODBUS.Event, kRxEvt_Overrun
   setf     MODBUS.FrameError ; note to self: discard the frame later

rxChar:
   ; Read the character to clear the interrupt flag.
   movff    RCREG, UART.LastCharacter

   ; Determine how to find the parity bit and which state machine to update.
   clrf     UART.LastParity   ; assume parity bit is clear
   movf     MODBUS.Mode       ; are we in RTU transmission mode?
   bz       rxRTU             ; yes, process a binary character

   ; ASCII mode, so the parity bit comes from the MSB of the character.
   btfsc    UART.LastCharacter, 7
     setf   UART.LastParity   ; copy the MSB as the parity value, then clear it
   bcf      UART.LastCharacter, 7
   goto     ASCII.rxCharacter ; update the ASCII state machine

rxRTU:
   ; RTU mode, so the parity bit comes from the reception status register.
   btfsc    RCSTA, RX9D
     setf   UART.LastParity   ; copy the status bit as parity value
   goto     RTU.rxByte        ; yes, process a binary character



;; ----------------------------------------------
;;  void UART.txCharacter()
;;
UART.txCharacter:
   tstfsz   MODBUS.Mode       ; are we in RTU transmission mode?
     goto   ASCII.txCharacter ; no, process a text character
   goto     RTU.txByte        ; yes, process a binary character



   end
