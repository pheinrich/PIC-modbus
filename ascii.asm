;; ---------------------------------------------------------------------------
;;
;;  Modbus
;;
;;  Copyright © 2006,7  Peter Heinrich
;;  All Rights Reserved
;;
;;  $URL$
;;  $Revision$
;;
;; ---------------------------------------------------------------------------
;;  $Author$
;;  $Date$
;; ---------------------------------------------------------------------------



   include "private.inc"

   ; Variables
   global   ASCII.Delimiter

   ; Methods
   global   ASCII.init
   global   ASCII.isrRx
   global   ASCII.isrTimeout
   global   ASCII.isrTx



; Count of timer1 overflows in 1 second (~92 @ 24 MHz).
kOneSecond              equ   1 + (kFrequency / (4 * 65536))



;; ----------------------------------------------
;;  Macro ResetTimer1()
;;
;;  Starts Timer1 in 16-bit timer mode with the maximum possible period,
;;  corresponding to a total delay of about 11ms.  To measure longer intervals
;;  we must count repeated overflows, so this macro also clears that counter.
;;
ResetTimer1             macro
   movlw    kOneSecond
   movwf    ASCII.Timeouts       ; prepare to measure 1 second

   ; Stop the timer and prepare to initialize its countdown period.
   movlw    b'10000000'
            ; 1------- RD16      ; enable 16-bit read/write operations
            ; -X------           ; [unimplemented]
            ; --00---- T1CKPS    ; 1:1 prescaler
            ; ----0--- T1OSCEN   ; disable external oscillator
            ; -----X-- T1SYNC    ; [not used when using internal clock]
            ; ------0- TMR1CS    ; use internal clock
            ; -------0 TMR1ON    ; disable timer, for now
   movwf    T1CON

   ; Set the countdown period to the maximum.
   clrf     TMR1H
   clrf     TMR1L

   ; Start the timer.
   bsf      PIE1, TMR1IE         ; enable associated overflow interrupt
   bsf      T1CON, TMR1ON        ; enable timer
   bcf      PIR1, TMR1IF         ; clear the timer interrupt flag
   endm



;; ---------------------------------------------------------------------------
.modeovr                access_ovr
;; ---------------------------------------------------------------------------

ASCII.Delimiter         res   1  ; frame delimiter character
ASCII.Timeouts          res   1  ; supports extra long (>1s) delays



;; ---------------------------------------------------------------------------
.ascii                  code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  void ASCII.init()
;;
;;  Initializes the ASCII mode state machine and some associated variables.
;;
ASCII.init:
   extern   Modbus.State

   ; Initialize default frame delimiter.
   movlw    '\n'
   movwf    ASCII.Delimiter

   ; Start out Idle.
   movlw    Modbus.kState_Idle
   movwf    Modbus.State
   return



;; ----------------------------------------------
;;  void ASCII.isrRx()
;;
;;  Updates the state machine in response to a received character.  This may
;;  involve changing modes, resetting the message buffer, or storing a message
;;  byte.
;;
ASCII.isrRx:
   extern   Diag.logRxEvt
   extern   Modbus.Event
   extern   Modbus.resetFrame
   extern   Modbus.State
   extern   Modbus.MsgTail
   extern   Modbus.validateMsg
   extern   USART.Read

   ; Determine the state of the state machine, since characters received at diff-
   ; erent times result in different actions.
   movlw    Modbus.kState_Idle
   cpfseq   Modbus.State         ; is state machine in idle state?
     bra    rxReception          ; no, check if reception state

   ; Idle State:  reception of a colon (":") here indicates the start of a frame.
   movlw    ':'
   cpfseq   USART.Read           ; was a colon receieved?
     return                      ; no, we can exit

rxReceive:
   movlw    Modbus.kState_Reception ; yes, enter reception mode
   movwf    Modbus.State

rxReset:
   lfsr     FSR0, Modbus.kASCIIBuffer
   call     Modbus.resetFrame

rxTimer:
   ResetTimer1                   ; reset the inter-character delay timeout
   return                        ; note that the SOF character isn't buffered

rxReception:
   ; Check for the next state concerned with received characters.
   movlw    Modbus.kState_Reception
   cpfseq   Modbus.State         ; is state machine in reception state?
     bra    rxWaiting            ; no, check if waiting state

   ; Reception State:  received characters are buffered as part of the frame in
   ; progress unless a colon (":"), in which case the frame is reset, or carriage
   ; return, in which case we switch to waiting mode.
   movlw    ':'
   subwf    USART.Read, W        ; was a colon received?
   bz       rxReset              ; yes, reset the frame pointer and exit

   movlw    '\r'
   cpfseq   USART.Read           ; was a carriage return received?
     bra    rxStash              ; no, buffer the character

   ; A carriage return was received.
   movlw    Modbus.kState_Waiting ; enter waiting state
   movwf    Modbus.State
   bra      rxTimer

rxStash:
   ; Stash the character in the message buffer.
   call     Modbus.putFrameByte
   bra      rxTimer

rxWaiting:
   ; Check for the next state concerned with received characters.
   movlw    Modbus.kState_Waiting
   cpfseq   Modbus.State         ; is state machine in waiting state?
     return                      ; no, we can exit

   ; Waiting State:  after a carriage return is received, this state waits for the
   ; end of frame marker specified by ASCII.Delimiter (usually linefeed, 0xd).  If
   ; a colon is received first, the frame is reset instead.
   movlw    ':'
   subwf    USART.Read, W        ; was a colon received?
   bz       rxReceive            ; yes, clear buffer and move to reception state

   movf     ASCII.Delimiter, W
   cpfseq   USART.Read           ; was a linefeed (or alternative delimiter) received?
     return                      ; no, keep waiting

   ; A frame delimiter was received.  Rewind the message tail to back up past the
   ; last two characters, since they represent the checksum computed by the send-
   ; er, which we don't want to include in our checksum calculation.
   bcf      PIE1, TMR1IE         ; disable timer1 interrupts
   movlw    (1 << Modbus.kRxEvt_CommErr) | (1 << Modbus.kRxEvt_Overrun)
   andwf    Modbus.Event, W      ; were there communication errors?
   bnz      rxDone               ; yes, discard the frame
   CopyWord Modbus.MsgTail, FSR1L ; save the old value for later

   movlw    0x2                  ; rewind 2 characters
   subwf    Modbus.MsgTail
   movlw    0x0
   subwfb   Modbus.MsgTail + 1

   ; Compute the checksum from the original characters, then convert the message
   ; to the equivalent binary (RTU) format.  Once that's done, we can use common
   ; code to validate the address, verify the checksum, and parse the contents.
   lfsr     FSR0, Modbus.kASCIIBuffer ; FSR0 = message head
   rcall    calcLRC
   rcall    ascii2rtu

   call     Modbus.validateMsg
   tstfsz   WREG                 ; was the validation successful?
     bra    rxDone               ; no, discard the frame

   ; No reception errors, no checksum errors, and the message is addressed to us.
   movlw    Modbus.kState_MsgQueued ; alert the main event loop that a message has arrived
   movwf    Modbus.State
   goto     Diag.logRxEvt        ; log the receive event in the event log

rxDone:
   ; There was a communication error (parity, overrun, checksum) or the message
   ; simply wasn't addressed to us.
   movlw    Modbus.kState_Idle   ; be ready to receive the next message
   movwf    Modbus.State
   bsf      Modbus.Event, Modbus.kRxEvt_NoResponse
   goto     Diag.logRxEvt        ; log the receive event in the event log

   

;; ----------------------------------------------
;;  void ASCII.isrTimeout()
;;
;;  Updates the state machine in response to a timer overflow.  The timer is
;;  started whenever a character is received, and overflows approximately 92
;;  times a second.  A one second delay between characters indicates a problem
;;  with the frame, which this method flags, if detected.
;;
ASCII.isrTimeout:
   extern   Modbus.Event
   extern   Modbus.State

   ; Determine the state of the state machine.  A timeout while receiving or wait-
   ; ing means the frame has gone stale.
   movlw    Modbus.kState_Reception
   subwf    Modbus.State, W      ; is state machine in reception state?
   bz       timeoutUpdate        ; yes, update our long timer

   movlw    Modbus.kState_Waiting ; no, check next state
   cpfseq   Modbus.State         ; is state machine in waiting state?
     return                      ; no, we can exit

timeoutUpdate:
   ; The timer fired, so decrement the timeout count.  Once that count has reached
   ; 0, our "long" timer has elapsed and we can assume the frame is incomplete.
   decfsz   ASCII.Timeouts       ; has 1 second expired?
     return                      ; no, keep waiting

   bsf      Modbus.Event, Modbus.kRxEvt_CommErr; yes, so frame is incomplete
   bcf      PIE1, TMR1IE         ; disable redundant timer1 interrupts
   return



;; ----------------------------------------------
;;  void ASCII.isrTx()
;;
ASCII.isrTx:
   extern   Diag.logTxEvt
   extern   Modbus.Checksum
   extern   Modbus.getFrameByte
   extern   Modbus.MsgTail
   extern   Modbus.State
   extern   USART.send
   extern   Util.hex2char

   ; Determine the state of the state machine.
   movlw    Modbus.kState_EmitStart
   cpfseq   Modbus.State
     bra    txEmission

   ; Emit Start State:  a message reply we want to send is waiting in kASCIIBuffer,
   ; but we must calculate its checksum before we can transmit it.
   rcall    rtu2ascii      ; convert to ASCII mode first
   lfsr     FSR0, Modbus.kASCIIBuffer
   rcall    calcLRC        ; calculate the checksum

   ; Store the checksum at the end of the message buffer and update the tail.
   movf     Modbus.Checksum, W
   swapf    WREG
   call     Util.hex2char
   movwf    POSTINC0

   movf     Modbus.Checksum, W
   call     Util.hex2char
   movwf    POSTINC0
   CopyWord FSR0L, Modbus.MsgTail

   ; Transmit the start-of-frame character and switch to the next state.
   movlw    Modbus.kState_Emission
   movwf    Modbus.State
   movlw    ':'
   bra      txStash

txEmission:
   ; Check for the next state concerned with transmitted characters.
   movlw    Modbus.kState_Emission
   cpfseq   Modbus.State
     bra    txEmitEnd

   ; Emission State:  send the next message character, if available.  If not, we
   ; must have sent the whole message.
   call     Modbus.getFrameByte
   bnc      txStash

   movlw    Modbus.kState_EmitEnd ; switch state
   movwf    Modbus.State
   movlw    '\r'                 ; send first frame delimiter character

txStash:
   ; Transmit the character.  Parity will be calculated and and set automatically
   ; by the USART.
   goto     USART.send

txEmitEnd:
   ; Check for the next state concerned with transmitted characters.
   movlw    Modbus.kState_EmitEnd
   cpfseq   Modbus.State
     bra    txEmitDone

   ; Emit End State: the first byte of the end-of-frame marker has been sent.  Now
   ; send the ASCII delimiter.
   movlw    Modbus.kState_EmitDone
   movwf    Modbus.State
   movf     ASCII.Delimiter, W
   bra      txStash

txEmitDone:
   ; Check for the next state concerned with transmitted characters.
   movlw    Modbus.kState_EmitDone
   cpfseq   Modbus.State
     return

   ; Emit Done State: we've successfully transmitted our message reply, including
   ; it's checksum, the end-of-frame marker, and ASCII delimiter.  We're idle!
   movlw    Modbus.kState_Idle
   movwf    Modbus.State
   bcf      PIE1, TXIE
   goto     Diag.logTxEvt



;; ----------------------------------------------
;;  void ascii2rtu()
;;
;;  Converts the message in the Modbus.kASCIIBuffer to binary (RTU) format in
;;  the Modbus.kRxBuffer.  Each original character contributes one nybble, so
;;  the result is 50% smaller.  The main advantage, however, is being able to
;;  share common code to verify address, validate checksum, and parse ASCII and
;;  RTU messages.
;;
;;  This method expects FSR1 to point one past the end of the message buffer,
;;  including the last two checksum characters (which we want to convert as
;;  much the message).
;;
ascii2rtu:
   extern   Modbus.MsgTail
   extern   Util.Volatile
   extern   Util.char2hex

   ; Initialize some pointers.
   lfsr     FSR0, Modbus.kASCIIBuffer ; FSR0 = message head (ASCII)
   lfsr     FSR2, Modbus.kRxBuffer ; FSR2 = message tail (RTU)

a2rLoop:
   ; Compare the head and tail pointers.
   movf     FSR0L, W
   cpfseq   FSR1L                ; are low bytes equal?
     bra    a2rUpdate            ; no, keep going

   movf     FSR0H, W
   cpfseq   FSR1H                ; are high bytes equal?
     bra    a2rUpdate            ; no, keep going

   ; The head and tail pointers are equal, so we're done.  The last thing we do is
   ; clear the LRC's "virtual" high byte.  The LRC is only 8-bits, but we treat it
   ; like a 16-bit little-endian word to mimic the CRC16 used in RTU mode.
   clrf     POSTDEC2
   CopyWord FSR2L, Modbus.MsgTail
   return

a2rUpdate:
   ; Read the next two characters and combine them into a single byte.
   movf     POSTINC0, W          ; read the first character
   call     Util.char2Hex        ; convert to nybble
   swapf    WREG
   movwf    Util.Volatile

   movf     POSTINC0, W          ; read the second character
   call     Util.char2Hex        ; convert to nybble
   iorwf    Util.Volatile, W

   ; Store the byte back into buffer.  We'll never catch up to our read pointer
   ; since it's moving twice as fast.
   movwf    POSTINC2
   bra      a2rLoop              ; go back for the next pair



;; ----------------------------------------------
;;  void calcLRC( const char buffer[] )
;;
;;  Calculates the Longitudinal Redundancy Checksum on the ASCII characters in
;;  the message buffer, not including the checksum at the end (inserted by the
;;  sender).  The LRC is a simple sum, discarding all carries, which is then
;;  2s-complemented.  It's an 8-bit value, so the upper byte of the checksum is
;;  set to 0.
;;
;;  This code expects Modbus.MsgTail to point to one past the last character
;;  to be included in the checksum.
;;
calcLRC:
   extern   Modbus.Checksum
   extern   Modbus.MsgTail

   ; Initialize the checksum and a pointer to the message buffer.
   clrf     Modbus.Checksum      ; LRC starts at 0
   clrf     Modbus.Checksum + 1

lrcLoop:
   ; Compare the head and tail pointers.
   movf     FSR0L, W
   cpfseq   Modbus.MsgTail       ; are low bytes equal?
     bra    lrcUpdate            ; no, keep going

   movf     FSR0H, W
   cpfseq   Modbus.MsgTail + 1   ; are high bytes equal?
     bra    lrcUpdate            ; no, keep going

   ; The head and tail pointers are equal, so we're done.
   negf     Modbus.Checksum      ; LRC is 2s complement of actual sum
   return

lrcUpdate:
   ; Update the checksum with the current character.
   movf     POSTINC0, W          ; read the charecter at head
   addwf    Modbus.Checksum      ; add to sum, discarding carry
   bra      lrcLoop              ; go back for the next one



;; ----------------------------------------------
;;  void rtu2ascii()
;;
;;  Converts a message in the Modbus.kTxBuffer (RTU mode) to its ASCII repre-
;;  sentation in kASCIIBuffer.  This method assumes Modbus.MsgTail points one
;;  past the last message byte, not including the checksum.
;;
rtu2ascii:
   extern   Modbus.MsgTail
   extern   Util.hex2char

   ; Initialize some pointers.
   lfsr     FSR0, Modbus.kTxBuffer ; FSR0 = message head (RTU)
   lfsr     FSR2, Modbus.kASCIIBuffer ; FSR2 = message tail (ASCII)

r2aLoop:
   ; Compare the head and tail pointers.
   movf     FSR0L, W
   cpfseq   Modbus.MsgTail       ; are low bytes equal?
     bra    r2aUpdate            ; no, keep going

   movf     FSR0H, W
   cpfseq   Modbus.MsgTail + 1   ; are high bytes equal?
     bra    r2aUpdate            ; no, keep going

   ; The head and tail pointers are equal, so we're done.
   CopyWord FSR2L, Modbus.MsgTail
   return

r2aUpdate:
   ; Convert the high nybble of the next byte to an ASCII character.
   movf     INDF0, W             ; read the byte, but don't advance the pointer
   swapf    WREG                 ; process the high nybble first
   call     Util.hex2char
   movwf    POSTINC2             ; store it in the kASCIIBuffer

   ; Convert the low nybble of the same byte.
   movf     POSTINC0, W          ; re-read the byte and advance this time
   call     Util.hex2char
   movwf    POSTINC2             ; store it
   bra      r2aLoop              ; go back for the next byte



   end
