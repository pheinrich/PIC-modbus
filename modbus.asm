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
   extern   DIAG.Options
   extern   UART.LastCharacter
   extern   UART.LastParity

   global   MODBUS.Address
   global   MODBUS.BaudRate
   global   MODBUS.Event
   global   MODBUS.Checksum
   global   MODBUS.Mode
   global   MODBUS.MsgHead
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
   global   MODBUS.getFrameByte
   global   MODBUS.init
   global   MODBUS.putFrameByte
   global   MODBUS.replyMsg
   global   MODBUS.resetFrame
   global   MODBUS.setParity
   global   MODBUS.validateMsg



;; ---------------------------------------------------------------------------
            udata_acs
;; ---------------------------------------------------------------------------

MODBUS.Address       res   1  ; uniquely identifies this device on the bus
MODBUS.BaudRate      res   1  ; kBaud_9600, kBaud_19200 (default), etc.
MODBUS.Event         res   1  ; kRxEvt_CommErr, kRxEvt_Broadcast, kTxEvt_Abort, etc.
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
;;  condition configured by the user.  If not, kRxEvt_CommErr is added to the
;;  current event to indicate as much.
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
     bsf    MODBUS.Event, kRxEvt_CommErr
   return



;; ----------------------------------------------
;;  byte MODBUS.getFrameByte()
;;
MODBUS.getFrameByte:
   ; Make sure we're not trying to read past the end of the buffer.
   movf     MODBUS.MsgHead, W
   cpfseq   MODBUS.MsgTail
     bra    getByte

   movf     MODBUS.MsgHead + 1, W
   cpfseq   MODBUS.MsgTail + 1
     bra    getByte

   ; The head and tail pointers are equal, so we must be done.  Set the carry to
   ; indicate that no byte was read.
   bsf      STATUS, C
   return

getByte:
   ; Not past the end yet, so read the next byte (indirectly).
   LDADDR   MODBUS.MsgHead, FSR1L
   movf     POSTINC1, W
   LDADDR   FSR1L, MODBUS.MsgHead

   ; Clear the status flag to indicate the value in W is valid.
   bcf      STATUS, C
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
;;  woid MODBUS.putFrameByte( byte value )
;;
;;  todo: bounds checking
;;
MODBUS.putFrameByte:
   ; Store the byte at the next message buffer location.  A tail pointer keeps
   ; track of where it goes, so we fetch it first, then update its value with the
   ; new tail location when we're finished.
   LDADDR   MODBUS.MsgTail, FSR1L
   movwf    POSTINC1
   LDADDR   FSR1L, MODBUS.MsgTail
   return



;; ----------------------------------------------
;;  woid MODBUS.replyMsg()
;;
;;  Attempts to send the message in the message buffer, which is always in re-
;;  ply to a message we previously received.
;;
MODBUS.replyMsg:
   ; This method does nothing if a complete, error-free server request isn't al-
   ; ready available in the message buffer.
   movlw    kState_MsgQueued
   cpfseq   MODBUS.State      ; is there a message waiting for us?
     return                   ; no, bail

   ; DEBUG copy the received message to the transmit buffer (echo the message).
   LDADDR   MODBUS.MsgHead, FSR0L; debug
   movlw    LOW kTxBuffer     ; debug
   movwf    FSR1L             ; debug
   movlw    HIGH kTxBuffer    ; debug
   movwf    FSR1H             ; debug
                              ; debug
copyLoop:                     ; debug
   movf     FSR0L, W          ; debug
   cpfseq   MODBUS.MsgTail    ; debug
     bra    copyIt            ; debug
                              ; debug
   movf     FSR0H, W          ; debug
   cpfseq   MODBUS.MsgTail + 1; debug
     bra    copyIt            ; debug

   ; Change state and enable the character transmitted interrupt.  If the transmit
   ; buffer is empty, this will fire immediately, otherwise it will trigger after
   ; the current byte is transmitted.
   LDADDR   FSR1L, MODBUS.MsgTail
   movlw    kState_EmitStart  ; prepare to transmit the message
   movwf    MODBUS.State
   bsf      PIE1, TXIE        ; enable the interrupt
   return

copyIt:                       ; debug
   movff    POSTINC0, POSTINC1; debug
   bra      copyLoop          ; debug



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

   ; Reset the event mask, since we're starting from scratch.
   clrf     MODBUS.Event
   return



;; ----------------------------------------------
;;  void MODBUS.setParity( byte toSend )
;;
MODBUS.setParity:
   ; Assume no parity.
   bcf      STATUS, C
   movwf    MODBUS.Scratch + 1; preserve the character during computation

   ; If configured for No Parity, we're done.
   movlw    kParity_None
   cpfslt   MODBUS.ParityCheck
     bra    setDone

   ; Calculate the even parity of the character to send.
   movf     MODBUS.Scratch + 1, W
   call     MODBUS.calcParity ; count bits in character

   bcf      STATUS, C         ; assume the bit count is even
   btfsc    WREG, 0           ; is it?
     bsf    STATUS, C         ; no, make it so

   ; If configured for Odd Parity, flip the MSB we just set (or reset).
   movlw    kParity_Odd
   cpfslt   MODBUS.ParityCheck
     btg    STATUS, C

setDone:
   ; Restore the original character, plus or minus the correct parity bit.
   movf     MODBUS.Scratch + 1, W
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
