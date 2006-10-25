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
;;  This file provides a test harness for the MODBUS routines implemented by
;;  the other modules of this project.
;;
;; ---------------------------------------------------------------------------
;;  $Author$
;;  $Date$
;; ---------------------------------------------------------------------------



   config   OSC=HS, OSCS=OFF, PWRT=OFF, BOR=OFF, WDT=OFF, CCP2MUX=ON, STVR=ON
   config   LVP=OFF, DEBUG=OFF
   config   CP0=OFF, CP1=OFF, CPB=OFF, CPD=OFF
   config   WRT0=OFF, WRT1=OFF, WRTB=OFF, WRTD=OFF
   config   EBTR0=OFF, EBTR1=OFF, EBTRB=OFF

   #include "modbus.inc"

   extern   CONF.Mode
   extern   CONF.ParityCheck
   extern   UART.LastCharacter
   extern   UART.LastParity

   extern   ASCII.init
   extern   DEVICEID.getAddress
   extern   ISR.high
   extern   ISR.low
   extern   main
   extern   CONF.init
   extern   RTU.init
   extern   UART.init

   global   MODBUS.Address
   global   MODBUS.Scratch
   global   MODBUS.State
   global   MODBUS.FrameError

   global   MODBUS.calcParity
   global   MODBUS.checkParity
   global   MODBUS.queueMsg
   global   MODBUS.resetFrame
   global   MODBUS.storeFrameByte



;; ---------------------------------------------------------------------------
.isvReset   code     0x0000
;; ---------------------------------------------------------------------------

   ; Disable all interrupts and jump to relocatable initialization code.
   clrf     INTCON
   goto     MODBUS.init



;; ---------------------------------------------------------------------------
.isvHigh    code     0x0008
;; ---------------------------------------------------------------------------

   ; Jump to high-priority interrupt service routine in relocatable code block.
   goto     ISR.high



;; ---------------------------------------------------------------------------
.isvLow     code     0x0018
;; ---------------------------------------------------------------------------

   ; Jump to low-priority interrupt service routine in relocatable code block.
   goto     ISR.low



;; ---------------------------------------------------------------------------
            udata_acs
;; ---------------------------------------------------------------------------

MODBUS.Address    res   1     ; uniquely identifies this device on the bus
MODBUS.State      res   1     ; current state of the state machine
MODBUS.FrameError res   1     ; 0 = false, 255 = true
MODBUS.MsgBuffer  res   2     ; points to next location to be read or written
MODBUS.Scratch    res   1     ; temporary work variable



;; ---------------------------------------------------------------------------
.modbus     code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  byte MODBUS.calcParity( byte value )
;;
;;  Calculates the even parity of the byte specified in W.  The even parity is
;;  the 1-bit sum of all bits in the byte (also equivalent to XOR-ing them all
;;  together); the odd parity is the complement of that.  The result is re-
;;  turned in the LSB of W and MODBUS.Scratch.
;;
MODBUS.calcParity:
   ; Copy W into a temporary variable.
   movwf    MODBUS.Scratch    ; Scratch = |a|b|c|d|e|f|g|h|

   ; XOR the nybbles of W together.
   swapf    WREG              ; W = |e|f|g|h|a|b|c|d|
   xorwf    MODBUS.Scratch    ; Scratch = |e^a|f^b|g^c|h^d|a^e|b^f|c^g|d^h|

   ; Now shift the value by 1 in order to XOR adjacent bits together.
   rrcf     MODBUS.Scratch, W ; W = |?|e^a|f^b|g^c|h^d|a^e|b^f|c^g|
   xorwf    MODBUS.Scratch    ; Scratch = |?^e^a|e^a^f^b|f^b^g^c|g^c^h^d|h^d^a^e|a^e^b^f|b^f^c^g|c^g^d^h|

   ; Note that bit 2 = a^e^b^f, which is the parity of half the bits in the byte.
   ; Bit 0 = c^g^d^h, the parity of the other half, so (bit 2) ^ (bit 0) is the
   ; parity for the whole byte.  If bit 2 = 0, just take the value of bit 0, since
   ; parity = 0 ^ (bit 0) = bit 0.  For bit 2 = 1, the value is complemented,
   ; since parity = 1 ^ (bit 0) = !bit 0.
   btfsc    MODBUS.Scratch, 2 ; is a^e^b^f = 0?
     btg    MODBUS.Scratch, 0 ; no, toggle bit 0
   movf     MODBUS.Scratch, W ; yes, we're done

   return
   


;; ----------------------------------------------
;;  void MODBUS.checkParity()
;;
;;  Determines if the last character received by the UART satisfies the parity
;;  condition configured by the user.  If not, MODBUS.FrameError is set to
;;  indicate as much.
;;
MODBUS.checkParity:
   ; If configured for No Parity, don't do any checks.
   movlw    kParity_None
   cpfslt   CONF.ParityCheck
     return

   ; Compute the even parity of the character received.
   movf     UART.LastCharacter, W
   rcall    MODBUS.calcParity

   ; Add the last received parity bit into the mix.
   tstfsz   UART.LastParity
     incf   MODBUS.Scratch

   ; If configured for Odd Parity, we complement the result.
   movlw    kParity_Odd
   cpfslt   CONF.ParityCheck
     incf   MODBUS.Scratch

   ; If the final result is not 0, the parity does not match.
   btfsc    MODBUS.Scratch, 0
     setf   MODBUS.FrameError
   return



;; ----------------------------------------------
;;  void MODBUS.init()
;;
;;  Initializes the device before falling through to the main event loop.
;;  This routine calls other methods to handle specific peripherals.
;;
MODBUS.init:
   ; Some components of the system must be initialized.
   call     CONF.init         ; read configuration jumpers/switches
   call     UART.init         ; set UART mode, baud rate, etc.

   ; Initialize the correct mode according to the configuration.
   tstfsz   CONF.Mode         ; are we in RTU mode?
     bra    asciiInit         ; no, initialize ASCII mode

   call     RTU.init          ; calculate intercharacter/frame timeouts
   bra      clearInts

asciiInit:
   call     ASCII.init        ; set default delimiter

clearInts:
   ; Clear all pending peripheral interrupt flags.
   clrf     PIR1
   clrf     PIR2

   ; Re-enable interrupts.
   bsf      INTCON, PEIE      ; enable all peripheral interrupts
   bsf      INTCON, GIE       ; enable all unmasked interrupts

   ; Shadow our address on the bus.
   call     DEVICEID.getAddress
   movwf    MODBUS.Address

   ; Enter the main event loop.
   goto     main



;; ----------------------------------------------
;;  woid MODBUS.queueMsg()
;;
;;  Raises a flag to the main event loop to indicate that a MODBUS message has
;;  been received and parity, checksum, and target address have been verified.
;;  It's up to that code to actually process the message contents.
;;
;;  The active state machine will remain in kState_MsgQueued for the duration;
;;  MODBUS.replyMsg() must be called explicitly when processing is complete.
;;
MODBUS.queueMsg:
   return



;; ----------------------------------------------
;;  woid MODBUS.replyMsg()
;;
MODBUS.replyMsg:
   return



;; ----------------------------------------------
;;  woid MODBUS.resetFrame()
;;
MODBUS.resetFrame:
   ; Reset the pointer to the beginning of the buffer.
   movlw    LOW kMsgBuffer
   movwf    FSR1L
   movlw    HIGH kMsgBuffer
   movwf    FSR1H

   ; Reset the frame error indicator, since we're starting from scratch.
   clrf     MODBUS.FrameError
   return



;; ----------------------------------------------
;;  woid MODBUS.storeFrameByte( byte value )
;;
MODBUS.storeFrameByte:
   ; Write the byte indirectly.
   movwf    POSTINC1
;<debug>
   movwf    TXREG
;</debug>
   return



   end
