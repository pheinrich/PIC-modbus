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
;;  This module may optionally be overridden by the application.
;;
;; ---------------------------------------------------------------------------
;;  $Author$
;;  $Date$
;; ---------------------------------------------------------------------------



   #include "modbus.inc"

   global   CONF.BaudRate
   global   CONF.Mode
   global   CONF.ParityCheck

   global   CONF.init



;; ----------------------------------------------
;;  Macro TSSC
;;
;;  The TSSC macro (Test Switch Skip if Closed) tests the position of a con-
;;  figuration switch, skipping the following instruction if the switch is
;;  closed (low).  Otherwise, the next instruction is executed as normal.
;;  The index parameter corresponds to a line from the B port.
;;
TSSC        macro index
   ; Read the hardware switch to check its position.
   btfsc    PORTB, RB#v(index)
   endm



;; ----------------------------------------------
;;  Macro TSSO
;;
;;  The TSSO macro (Test Switch Skip if Open) tests the position of a config-
;;  uration switch, skipping the following instruction if the switch is open
;;  (high).  Otherwise, the next instruction is executed as normal.  The in-
;;  dex parameter corresponds to a line from the B port.
;;
TSSO        macro index
   ; Read the hardware switch to check its position.
   btfss    PORTB, RB#v(index)
   endm



;; ----------------------------------------------
;;  Macro BDEF
;;
;;  The BDEF macro (Branch if Default) tests the "factory defaults" jumper
;;  (JP1) to determine if the configuration switches should be ignored.  If
;;  so, the branch is taken, otherwise execution continues with the statement
;;  immediately following.
;;
BDEF        macro label
   ; Determine if the config switches are active.
   TSSO     0                 ; test JP1
     bra    label             ; if shorted (low), use factory defaults
   endm



;; ---------------------------------------------------------------------------
            udata_acs
;; ---------------------------------------------------------------------------

CONF.BaudRate           res   1
CONF.Mode               res   1
CONF.ParityCheck        res   1



;; ---------------------------------------------------------------------------
.conf       code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  void CONF.init()
;;
;;  Initializes Port B to read switches and jumpers, then loads the config-
;;  uration.  Lines will appear LOW if the corresponding switch or jumper is
;;  CLOSED (because weak pull-ups will be enabled).
;;
CONF.init:
   ; Initialize the port.
   clrf     LATB              ; clear the latch
   movlw    b'11111111'       ; RB<7:0> will be inputs
   movwf    TRISB
   bcf      INTCON2, RBPU     ; enable weak pull-ups

   ; Read the switches to update the associated configuration variables.
   rcall    CONF.getBaudRate
   movwf    CONF.BaudRate     ; SW2-SW3

   rCall    CONF.getMode
   movwf    CONF.Mode         ; SW1

   rCall    CONF.getParityCheck
   movwf    CONF.ParityCheck  ; SW4-SW5

   return



;; ----------------------------------------------
;;  uint8 CONF.getBaudRate()
;;
;;  If JP1 is shorted (low), this method returns the factory default baud rate
;;  of 19200.  If not, the value is selected based on the configuration of
;;  switches SW2-SW3:
;;
;;    SW2  SW3  Data Signaling Rate
;;     0    0    9600
;;     0    1    19200
;;     1    0    57600        ; NON-COMPLIANT at 20MHz (3.34% error)
;;     1    1    115200       ; NON-COMPLIANT at 20MHz (8.51% error)
;;
;;  The 8-bit return value of this routine may be stored directly in SPBRG to
;;  generate the desired baud rate, assuming BRGH will also be set in TXSTA
;;  (forcing high baud rate mode).  Since this value varies according to CPU
;;  clock speed, this routine and its callers must agree on that frequency.
;;
;;  Note that Modbus requires that the baud rate be respected to better than
;;  1% during transmission and 2% during reception.  This is impossible for
;;  some standard rates when operating from a 20 MHz clock.
;;
CONF.getBaudRate:
   ; Determine if the config switches are active.
   BDEF     defRate           ; use defaults if JP1 shorted

   ; Read hardware switches to determine desired baud rate.
   TSSO     2                 ; test SW2
     bra    highSpeed         ; if closed, must be high speed rate
   TSSO     3                 ; test SW3

defRate:
     retlw  kBaud_19200       ; if closed, use 19200 baud
   retlw    kBaud_9600        ; otherwise, use 9600 baud

highSpeed:
   TSSO     3                 ; test SW3
     retlw  kBaud_115200      ; if closed, use 115200 baud
   retlw    kBaud_57600       ; otherwise, use 57600 baud



;; ----------------------------------------------
;;  uint8 CONF.getMode()
;;
;;  If JP1 is shorted (low), the default transmission mode is returned, which
;;  is RTU (binary), according to Modbus requirements.  If not, the mode is
;;  determined from the state of SW1:
;;
;;    SW1  Transmission Mode
;;     0    RTU (8-bit characters)
;;     1    ASCII (7-bit characters)
;;
CONF.getMode:
   ; Determine if the configuration switch is active.
   BDEF     defMode           ; use defaults if JP1 is shorted

   ; Read hardware switch to determine desired mode.
   TSSC     1                 ; test SW1

defMode:
     retlw  kMode_RTU         ; if open, use RTU (binary) mode
   retlw    kMode_ASCII       ; if closed, use ASCII (text) mode



;; ----------------------------------------------
;;  uint8 CONF.getParityCheck()
;;
;;  If JP1 is shorted (low), this method returns Even parity, the factory
;;  default.  If not, the value is selected based on configuration switches
;;  SW4-SW5:
;;
;;    SW4  SW5  Error Checking Methods
;;     0    X    Even
;;     1    0    Odd
;;     1    1    None (use two stop bits instead)
;;
CONF.getParityCheck:
   ; Determine if the configuration switches are active.
   BDEF     defParity         ; use defaults if JP1 is shorted

   ; Read hardware switches to determine desired parity.
   TSSC     4                 ; test SW4
   
defParity:
     retlw  kParity_Even      ; if open, parity is Even

   TSSC     5                 ; if closed, check SW5
     retlw  kParity_Odd       ; if open, parity is Odd
   retlw    kParity_None      ; if closed, parity is None



   end
