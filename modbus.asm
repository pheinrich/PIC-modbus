;; ---------------------------------------------------------------------------
;;
;;  MODBUS
;;
;;  Copyright � 2006  Peter Heinrich
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
   extern   DIAG.Options
   extern   UART.LastCharacter
   extern   UART.LastParity

   global   MODBUS.Address
   global   MODBUS.BaudRate
   global   MODBUS.Event
   global   MODBUS.Checksum
   global   MODBUS.FrameError
   global   MODBUS.Mode
   global   MODBUS.MsgTail
   global   MODBUS.NoChecksum
   global   MODBUS.ParityCheck
   global   MODBUS.Scratch
   global   MODBUS.State

   ; Methods
   extern   ASCII.init
   extern   DIAG.init
   extern   RTU.init
   extern   UART.init

   global   MODBUS.calcParity
   global   MODBUS.checkParity
   global   MODBUS.init
   global   MODBUS.replyMsg
   global   MODBUS.resetFrame
   global   MODBUS.storeFrameByte
   global   MODBUS.validateMsg



;; ---------------------------------------------------------------------------
            udata_acs
;; ---------------------------------------------------------------------------

MODBUS.Address       res   1  ; uniquely identifies this device on the bus
MODBUS.BaudRate      res   1  ; kBaud_9600, kBaud_19200 (default), etc.
MODBUS.Event         res   1  ; kRxEvt_CommErr, kRxEvt_Broadcast, kTxEvt_Abort, etc.
MODBUS.FrameError    res   1  ; 0 = false, 255 = true
MODBUS.Mode          res   1  ; kMode_ASCII or kMode_RTU (default)
MODBUS.NoChecksum    res   1  ; 0 = false (default), 255 = true
MODBUS.ParityCheck   res   1  ; kParity_Even (default), kParity_Off, kParity_None
MODBUS.State         res   1  ; current state of the state machine

MODBUS.Checksum      res   2  ; LRC or CRC, depending on mode (ASCII or RTU)
MODBUS.MsgHead       res   2  ; points to the first byte in the message
MODBUS.MsgTail       res   2  ; points to next location to be read or written
MODBUS.Scratch       res   2  ; temporary work variables



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
   cpfslt   MODBUS.ParityCheck
     return

   ; Compute the even parity of the character received.
   movf     UART.LastCharacter, W
   rcall    MODBUS.calcParity

   ; Add the last received parity bit into the mix.
   tstfsz   UART.LastParity
     incf   MODBUS.Scratch

   ; If configured for Odd Parity, we complement the result.
   movlw    kParity_Odd
   cpfslt   MODBUS.ParityCheck
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
   call     DIAG.init         ; reset diagnostic registers and event log
   call     UART.init         ; set UART mode, baud rate, etc.

   ; Initialize the correct mode according to the configuration.
   tstfsz   MODBUS.Mode       ; are we in RTU mode?
     goto   ASCII.init        ; no, initialize ASCII mode
   goto     RTU.init          ; yes, initialize RTU mode



;; ----------------------------------------------
;;  woid MODBUS.replyMsg()
;;
;;  Attempts to send the message in the message buffer, which is a always in
;;  reply to a message we previously received.
;;
MODBUS.replyMsg:
   movlw    kState_Idle       ; debug
   movwf    MODBUS.State      ; debug
   return



;; ----------------------------------------------
;;  woid MODBUS.resetFrame( char* buffer )
;;
;;  Initializes the message frame for fresh reception or transmission.  This
;;  method stashes the base pointer for later use by related routines.
;;
MODBUS.resetFrame:
   ; Reset the pointer to the beginning of the buffer.
   movf     FSR0L, W
   movwf    MODBUS.MsgHead
   movwf    MODBUS.MsgTail

   movf     FSR0H, W
   movwf    MODBUS.MsgHead + 1
   movwf    MODBUS.MsgTail + 1

   ; Reset the frame error and event mask, since we're starting from scratch.
   clrf     MODBUS.FrameError
   clrf     MODBUS.Event
   return



;; ----------------------------------------------
;;  woid MODBUS.storeFrameByte( byte value )
;;
MODBUS.storeFrameByte:
   ; Store the byte at the next message buffer location.  A tail pointer keeps
   ; track of where it goes, so we fetch it first, then update its value with the
   ; new tail location when we're finished.
   LDADDR   MODBUS.MsgTail, FSR1L
   movwf    POSTINC1
   LDADDR   FSR1L, MODBUS.MsgTail
   return



;; ----------------------------------------------
;;  boolean MODBUS.validateMsg()
;;
MODBUS.validateMsg:
   ; Verify the message is addressed to this device.
   bsf      MODBUS.Event, kRxEvt_Broadcast ; assume broadcast message
   movf     kRxBuffer, W      ; is this a broadcast message (0 == address)?
   bz       valChecksum       ; yes, validate the checksum

   bcf      MODBUS.Event, kRxEvt_Broadcast ; no, clear our assumption
   cpfseq   MODBUS.Address    ; is it addressed to this specific device?
     retlw  0xff              ; no, discard frame

valChecksum:
   ; This message is addressed to us.
   bsf      MODBUS.Event, kRxEvt_SlaveMsg

   ; If checksum validation is turned off, we're done.
   tstfsz   MODBUS.NoChecksum
     retlw  0

   ; Set a pointer to the last byte in the message buffer.
   LDADDR   MODBUS.MsgTail, FSR0L
   bsf      MODBUS.Event, kRxEvt_CommErr ; assume checksum fails

   ; Compare the checksum included in the message to the one we calculated.
   movf     POSTINC0, W       ; fetch the LSB
   cpfseq   MODBUS.Checksum   ; does it match the computed value?
     retlw  0xff              ; no, discard the frame  TODO: log event

   movf     INDF0, W          ; yes, fetch the MSB
   cpfseq   MODBUS.Checksum + 1; does it match the computed value?
     retlw  0xff              ; no, discard the frame  TODO: log event

   ; Success, so clear our earlier assumption about a bad checksum.
   bcf      MODBUS.Event, kRxEvt_CommErr
   retlw    0



   end
