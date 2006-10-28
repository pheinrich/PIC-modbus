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



   include "modbus.inc"

   extern   MODBUS.Checksum
   extern   MODBUS.FrameError
   extern   MODBUS.MsgTail
   extern   MODBUS.Scratch
   extern   MODBUS.State
   extern   UART.LastCharacter

   global   ASCII.Delimiter

   extern   MODBUS.calcParity
   extern   MODBUS.checkParity
   extern   MODBUS.queueMsg
   extern   MODBUS.resetFrame
   extern   MODBUS.storeFrameByte
   extern   MODBUS.validateMsg

   global   ASCII.init
   global   ASCII.rxCharacter
   global   ASCII.timeout



; Count of timer1 overflows in 1 second (~77 @ 20 MHz).
kOneSecond        equ   1 + (kFrequency / (4 * 65536))



;; ----------------------------------------------
;;  Macro RESET_TIMER1
;;
;;  The RESET_TIMER1 macro starts Timer1 in 16-bit timer mode with the max-
;;  imum possible period, corresponding to a total delay of about 13ms.  To
;;  measure longer intervals we must count repeated overflows, so this macro
;;  also clears that counter.
;;
RESET_TIMER1  macro
   movlw    kOneSecond
   movwf    ASCII.Timeouts    ; prepare to measure 1 second

   ; Stop the timer and prepare to initialize its countdown period.
   movlw    b'10000000'
            ; 1------- RD16   ; enable 16-bit read/write operations
            ; -X------        ; [unimplemented]
            ; --00---- T1CKPS ; 1:1 prescaler
            ; ----0--- T1OSCEN; disable external oscillator
            ; -----X-- T1SYNC ; [not used when using internal clock]
            ; ------0- TMR1CS ; use internal clock
            ; -------0 TMR1ON ; disable timer, for now
   movwf    T1CON

   ; Set the countdown period to the maximum.
   clrf     TMR1H
   clrf     TMR1L

   ; Start the timer.
   bsf      PIE1, TMR1IE      ; enable associated overflow interrupt
   bsf      T1CON, TMR1ON     ; enable timer
   bcf      PIR1, TMR1IF      ; clear the timer interrupt flag
   endm



;; ---------------------------------------------------------------------------
.modeovr    access_ovr
;; ---------------------------------------------------------------------------

ASCII.Delimiter      res   1     ; frame delimiter character
ASCII.Timeouts       res   1     ; supports extra long (>1s) delays



;; ---------------------------------------------------------------------------
.ascii      code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  void ASCII.calcLRC()
;;
;;  Calculates the Longitudinal Redundancy Checksum on the ASCII characters in
;;  the message buffer, not including the checksum at the end (inserted by the
;;  sender).  The LRC is a simple sum, discarding all carries, which is then
;;  2s-complemented.  It's an 8-bit value, so the upper byte of the checksum
;;  is set to be 0.
;;
ASCII.calcLRC:
   ; Initialize the checksum and a pointer to the message buffer.
   clrf     MODBUS.Checksum   ; LRC starts at 0
   clrf     MODBUS.Checksum + 1
   lfsr     FSR0, kMsgBuffer  ; FSR0 = message head (FSR1 = message tail)

   ; Back up past the last two characters received, since they represent the
   ; checksum itself, which we don't want to include in the calculation.
   movf     POSTDEC1
   movf     POSTDEC1

lrcLoop:
   ; Compare the head and tail pointers.
   movf     FSR0L, W
   cpfseq   FSR1L             ; are low bytes equal?
     bra    lrcUpdate         ; no, keep going

   movf     FSR0H, W
   cpfseq   FSR1H             ; are high bytes equal?
     bra    lrcUpdate         ; no, keep going

   ; The head and tail pointers are equal, so we're done.
   negf     MODBUS.Checksum   ; LRC is 2s complement of actual sum
   return

lrcUpdate:
   ; Update the checksum with the current character.
   movf     POSTINC0, W       ; read the charecter at head
   addwf    MODBUS.Checksum   ; add to sum, discarding carry
   bra      lrcLoop           ; go back for the next one



;; ----------------------------------------------
;;  byte ASCII.char2Hex( byte ascii )
;;
;;  Converts the specified ASCII character code into the integer value corre-
;;  sponding to the hexadecimal digit it represents.  '0'-'9' become 0-9;
;;  'A'-'F' and 'a'-'f' become 10-15.
;;
ASCII.char2Hex:
   ; Shift the character.
   addlw    0x9f
   bnn      adjust            ; if positive, character was 'a' to 'f'
   addlw    0x20              ; otherwise, shift to next range of digits
   bnn      adjust            ; if now positive, character was 'A' to 'F'
   addlw    0x7               ; otherwise, character must have been '0' to '9'

adjust:
   addlw    0xa               ; shift the result to account for the alpha offset
   andlw    0xf               ; clamp the value to one nybble
   return



;; ----------------------------------------------
;;  void ASCII.init()
;;
;;  Initializes the ASCII mode state machine and some associated variables.
;;
ASCII.init:
   ; Initialize default frame delimiter.
   movlw    '\n'
   movwf    ASCII.Delimiter

   ; Start out Idle.
   movlw    kState_Idle
   movwf    MODBUS.State
   return



;; ----------------------------------------------
;;  void ASCII.rxCharacter()
;;
;;  Updates the state machine in response to a received character.  This may
;;  involve changing modes, resetting the message buffer, or storing a message
;;  byte.
;;
ASCII.rxCharacter:
   ; Determine the state of the state machine, since characters received at diff-
   ; erent times result in different actions.
   movlw    kState_Idle
   cpfseq   MODBUS.State      ; is state machine in idle state?
     bra    rxReception       ; no, check if reception state

   ; Idle State:  reception of a colon (":") here indicates the start of a frame.
   movlw    ':'
   cpfseq   UART.LastCharacter; was a colon receieved?
     return                   ; no, we can exit

rxReceive:
   movlw    kState_Reception  ; yes, enter reception mode
   movwf    MODBUS.State

rxReset:
   call     MODBUS.resetFrame

rxTimer:
   RESET_TIMER1               ; reset the inter-character delay timeout
   return                     ; note that the SOF character isn't buffered

rxReception:
   ; Check for the next state concerned with received characters.
   movlw    kState_Reception
   cpfseq   MODBUS.State      ; is state machine in reception state?
     bra    rxWaiting         ; no, check if waiting state

   ; Reception State:  received characters are buffered as part of the frame in
   ; progress unless a colon (":"), in which case the frame is reset, or carriage
   ; return, in which case we switch to waiting mode.
   movlw    ':'
   subwf    UART.LastCharacter, W ; was a colon received?
   bz       rxReset           ; yes, reset the frame pointer and exit

   movlw    '\r'
   cpfseq   UART.LastCharacter; was a carriage return received?
     bra    rxStash           ; no, buffer the character

   ; A carriage return was received.
   movlw    kState_Waiting    ; enter waiting state
   movwf    MODBUS.State
   bra      rxTimer

rxStash:
   ; Stash the character in the message buffer.
   call     MODBUS.checkParity; parity errors don't stop reception, just invalidate frame
   movf     UART.LastCharacter, W
   call     MODBUS.storeFrameByte
   bra      rxTimer

rxWaiting:
   ; Check for the next state concerned with received characters.
   movlw    kState_Waiting
   cpfseq   MODBUS.State      ; is state machine in waiting state?
     return                   ; no, we can exit

   ; Waiting State:  after a carriage return is received, this state waits for the
   ; end of frame marker specified by ASCII.Delimiter (usually linefeed, 0xd).  If
   ; a colon is received first, the frame is reset instead.
   movlw    ':'
   subwf    UART.LastCharacter, W ; was a colon received?
   bz       rxReceive         ; yes, clear buffer and move to reception state

   movf     ASCII.Delimiter, W
   cpfseq   UART.LastCharacter; was a linefeed (or alternative delimiter) received?
     return                   ; no, keep waiting

   ; A frame delimiter was received, so verify we didn't detect any overflow or
   ; parity errors.
   bcf      PIE1, TMR1IE      ; disable timer1 interrupts
   movf     MODBUS.FrameError ; was there an error during reception?
   bnz      rxDone            ; yes, discard the frame

   ; Compute the checksum from the original characters, then convert the message
   ; to the equivalent binary (RTU) format.  Once that's done, we can use common
   ; code to validate the address, verify the checksum, and parse the contents.
   LDADDR   MODBUS.MsgTail, FSR1L
   rcall    ASCII.calcLRC
   rcall    ASCII.xformRTU

   call     MODBUS.validateMsg
   tstfsz   WREG              ; was the validation successful?
     bra    rxDone            ; no, discard the frame

   ; No reception errors, no checksum errors, and the message is addressed to us.
   movlw    kState_MsgQueued
   movwf    MODBUS.State
   goto     MODBUS.queueMsg   ; let the main event loop process the message

rxDone:
   ; Become idle.
   movlw    kState_Idle       ; enter idle state
   movwf    MODBUS.State
   return

   

;; ----------------------------------------------
;;  void ASCII.timeout()
;;
;;  Updates the state machine in response to a timer overflow.  The timer is
;;  started whenever a character is received, and overflows approximately 77
;;  times a second.  A one second delay between characters indicates a problem
;;  with the frame, which this method flags, if detected.
;;
ASCII.timeout:
   ; Determine the state of the state machine.  A timeout while receiving or wait-
   ; ing means the frame has gone stale.
   movlw    kState_Reception
   subwf    MODBUS.State, W   ; is state machine in reception state?
   bz       timeoutUpdate     ; yes, update our long timer

   movlw    kState_Waiting    ; no, check next state
   cpfseq   MODBUS.State      ; is state machine in waiting state?
     return                   ; no, we can exit

timeoutUpdate:
   ; The timer fired, so decrement the timeout count.  Once that count has reached
   ; 0, our "long" timer has elapsed and we can assume the frame is incomplete.
   decfsz   ASCII.Timeouts    ; has 1 second expired?
     return                   ; no, keep waiting

   setf     MODBUS.FrameError ; yes, so frame is incomplete
   bcf      PIE1, TMR1IE      ; disable redundant timer1 interrupts
   return



;; ----------------------------------------------
;;  void ASCII.xformRTU()
;;
;;  Converts the message in the message buffer to binary (RTU) format.  Each
;;  original character contributes one nybble, so the result is 50% smaller.
;;  The main advantage, however, is being able to share common code to verify
;;  address, validate checksum, and parse ASCII and RTU messages.
;;
ASCII.xformRTU:
   ; Initialize some pointers.
   lfsr     FSR0, kMsgBuffer  ; FSR0 = message head (ASCII)
   movf     POSTINC1
   movf     POSTINC1          ; FSR1 = message tail (ASCII)
   lfsr     FSR2, kMsgBuffer  ; FSR2 = message tail (RTU)

xformLoop:
   ; Compare the head and tail pointers.
   movf     FSR0L, W
   cpfseq   FSR1L             ; are low bytes equal?
     bra    xformUpdate       ; no, keep going

   movf     FSR0H, W
   cpfseq   FSR1H             ; are high bytes equal?
     bra    xformUpdate       ; no, keep going

   ; The head and tail pointers are equal, so we're done.  The last thing we do is
   ; clear the LRC's "virtual" high byte.  The LRC is only 8-bits, but we treat it
   ; like a 16-bit little-endian word to mimic the CRC16 used in RTU mode.
   clrf     POSTINC2
   LDADDR   FSR2L, MODBUS.MsgTail
   return

xformUpdate:
   ; Read the next two characters and combine them into a single byte.
   movf     POSTINC0, W       ; read the first charecter
   rcall    ASCII.char2Hex    ; convert to nybble
   swapf    WREG
   movwf    MODBUS.Scratch

   movf     POSTINC0, W       ; read the second character
   rcall    ASCII.char2Hex    ; convert to nybble
   iorwf    MODBUS.Scratch, W

   ; Store the byte back into buffer.  We'll never catch up to our read pointer
   ; since it's moving twice as fast.
   movwf    POSTINC2
   bra      xformLoop         ; go back for the next pair



   end
