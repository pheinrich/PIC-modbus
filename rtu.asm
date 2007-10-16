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



   include "private.inc"

   ; Methods
   global   RTU.init
   global   RTU.isrRx
   global   RTU.isrTimeout
   global   RTU.isrTx



;; ----------------------------------------------
;;  Macro TIMER1 timeout
;;
;;  The TIMER1 macro starts Timer1 in 16-bit timer mode.  The parameter
;;  points to a 16-bit number indicating how many instruction cycles should
;;  elapse before overflow (resulting in a timer interrupt).  Since overflow
;;  occurs when the timer goes from 0xffff to 0x0000, the number is negative.
;;
;;  Four system cycles correspond to one instruction cycle, so a 20MHz clock
;;  means instructions execute at 5MHz, which means each instruction takes
;;  0.2탎 (200ns).  This macro should only be included in code called when
;;  Timer1 interrupts are disabled.
;;
TIMER1      macro timeout
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

   ; Set the countdown period.
   movff    timeout + 1, TMR1H
   movff    timeout, TMR1L

   ; Start the timer.
   bsf      PIE1, TMR1IE      ; enable associated overflow interrupt
   bsf      T1CON, TMR1ON     ; enable timer
   bcf      PIR1, TMR1IF      ; clear the timer interrupt flag
   endm



;; ---------------------------------------------------------------------------
.modeovr                access_ovr
;; ---------------------------------------------------------------------------

CharTimeout             res   2     ; the inter-character timeout, in 탎
FrameTimeout            res   2     ; the inter-frame timeout, in 탎
TimeoutDelta            res   2     ; difference between timeout values



;; ---------------------------------------------------------------------------
.rtu                    code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  Delay Timeout Table
;;
;;  Precalculate the inter-character and inter-frame timeout delays (in 탎)
;;  for 9600, 19200, and >19200 baud.  For baud rates greater than 19200, the
;;  timeouts are fixed at 750탎 and 1750탎, respectively, to reduce the CPU
;;  overhead of more frequent timer processing (see the Remark in �2.5.1.1 of
;;  the MODBUS over Serial Line Specification and Implementation guide V1.0).
;;  At slower baud rates, the inter-character timeout is 1.5 times the char-
;;  acter time, the time to transmit one character (11 bits, total); the in-
;;  ter-frame timeout is 3.5 times the character time.
;;
;;  Note that the values below are negative, since the timer interrupt fires
;;  when the timer overflows from 0xffff to 0x0000.  Also note that the timer
;;  is incremented every instruction cycle (kFrequency / 4), not every CPU
;;  cycle.
;;
DelayTable:
   ; Character/frame timers at 9600 baud.
   data     -((kFrequency / 4000000) * (1.5 / 9600))  ; 156.25탎
   data     -((kFrequency / 4000000) * (3.5 / 9600))  ; 364.58탎
   data     -((kFrequency / 4000000) * (2.0 / 9600))  ; 354.58 - 156.25 ~= 208.33탎

   ; Character/frame timers at 19200 baud.
   data     -((kFrequency / 4000000) * (1.5 / 19200)) ; 78.125탎
   data     -((kFrequency / 4000000) * (3.5 / 19200)) ; 182.29탎
   data     -((kFrequency / 4000000) * (2.0 / 19200)) ; 182.29 - 78.125 ~= 104.166탎

   ; Character/frame timers for all baud rates greater than 19200.
   data     -((kFrequency / 4000000) *  750)          ;  750탎
   data     -((kFrequency / 4000000) * 1750)          ; 1750탎
   data     -((kFrequency / 4000000) * 1000)          ; 1000탎



;; ----------------------------------------------
;;  void RTU.init()
;;
;;  Initializes the character and frame timeout delay variables based on the
;;  user-configured baud rate.  See ::DelayTable::, above.
;;
RTU.init:
   extern   USART.HookRx
   extern   USART.HookTx

   ; Set up a pointer to our table of timeout values.
   SetTableBase DelayTable

   ; Advance the table pointer if required for the requested baud rate.
   movlw    USART.kBaud_19200
   cpfslt   SPBRG                ; is baud rate more than 19200?
     bra    check9600            ; no, check lower rate

   ; The requested baud rate is greater than 19200, so advance the table index.
   movlw    0x6
   tblrd*+
   decfsz   WREG
     bra    $-4

check9600:
   ; Advance the table pointer if required for the requested baud rate.
   movlw    USART.kBaud_9600
   cpfslt   SPBRG                ; is baud rate more than 9600?
     bra    copyDelays           ; no, leave the table index unchanged

   ; The requested baud rate is greater than 9600, so advance the table index.
   movlw    0x6
   tblrd*+
   decfsz   WREG
     bra    $-4

copyDelays:
   ; Copy the correct table values to two variables in the access area, starting
   ; with the character timeout.
   lfsr     FSR0, CharTimeout

   ; Read the correct timing values from the table.
   movlw    0x6
   tblrd*+
   movff    TABLAT, POSTINC0
   decfsz   WREG
     bra    $-6

   ; Hook the serial port.
   movlw    LOW RTU.isrRx	 ; set the reception callback
   movwf    USART.HookRx
   movlw    HIGH RTU.isrRx
   movwf    USART.HookRx + 1

   movlw    LOW RTU.isrTx	 ; set the transmission callback
   movwf    USART.HookTx
   movlw    HIGH RTU.isrTx
   movwf    USART.HookTx + 1

   ; Initialize the state machine.
   clrf     Modbus.State
   TIMER1   FrameTimeout

   return



;; ----------------------------------------------
;;  void RTU.isrRx()
;;
;;  Processes a received binary character according to the current state of
;;  the state machine.  See �2.5.1.1 of the "MODBUS over serial line imple-
;;  mentation guide V1.0".
;;
RTU.isrRx:
   extern   Modbus.putFrameByte
   extern   Modbus.resetFrame
   extern   Modbus.State

   ; Determine the state of the state machine, since characters received at diff-
   ; erent times result in different actions.
   movlw    Modbus.kState_Init
   cpfseq   Modbus.State         ; is state machine in initial state?
     bra    rxIdle               ; no, check if idle state

   ; Initial State:  characters received now are from a frame already in progress,
   ; so reset the frame timeout timer and wait for it to expire before changing to
   ; the idle state (ready to receive or send).
   bra      rxFrame

rxIdle:
   ; Check for the next state concerned with received characters.
   movlw    Modbus.kState_Idle
   cpfseq   Modbus.State         ; is state machine in idle state?
     bra    rxReception          ; no, check if reception state

   ; Idle State:  a character received now indicates the beginning of a new frame,
   ; so begin reception.
   movlw    Modbus.kState_Reception ; switch to reception state
   movwf    Modbus.State

   lfsr     FSR0, Modbus.kRxBuffer
   call     Modbus.resetFrame
   bra      rxStash              ; start buffering frame characters

rxReception:
   ; Check for the next state concerned with received characters.
   movlw    Modbus.kState_Reception
   cpfseq   Modbus.State         ; is state machine in reception state?
     bra    rxCtrlWait           ; no, check if control-wait state

rxStash:
   ; Reception State:  characters received now are buffered until a character gap
   ; is detected.
   call     Modbus.putFrameByte
   TIMER1   CharTimeout          ; reset the character timeout timer
   return

rxCtrlWait:
   ; Check for the next state concerned with received characters.
   movlw    Modbus.kState_Waiting
   cpfseq   Modbus.State         ; is state machine in control-wait state?
     return                      ; no, we can exit

   ; Control-Wait State:  characters received now indicate a partial frame was
   ; received.  Now we must wait for a full frame timeout period to elapse before
   ; it's safe to go idle again.
   bsf      Modbus.Event, Modbus.kRxEvt_CommErr ; note that the frame should be discarded

rxFrame:
   ; Reset the frame timeout timer.
   TIMER1   FrameTimeout
   return



;; ----------------------------------------------
;;  void RTU.isrTimeout()
;;
RTU.isrTimeout:
   extern   Diag.logRxEvt
   extern   Diag.logTxEvt
   extern   Modbus.Event
   extern   Modbus.MsgTail
   extern   Modbus.State
   extern   Modbus.validateMsg

   ; Determine the state of the state machine, since timeouts that occur at diff-
   ; erent times result in different actions.
   movlw    Modbus.kState_Init
   cpfseq   Modbus.State         ; is state machine in initial state?
     bra    timeoutEmitEnd       ; no, check if emit end state

   ; Initial State:  a timeout here indicates a full frame timeout period has
   ; elapsed.  It's now safe to enter the idle state.
   bra      timeoutIdle

timeoutEmitEnd:
   ; Check for the next state concerned with timeouts.
   movlw    Modbus.kState_EmitEnd
   cpfseq   Modbus.State         ; is state machine in emit end state?
     bra    timeoutReception     ; no, check if reception state

   ; Emit End State:  a timeout here indicates our emission has been set off by a
   ; full frame timeout period.  We're idle again.
   call     Diag.logTxEvt
   bra      timeoutIdle

timeoutReception:
   ; Check for the next state concerned with timeouts.
   movlw    Modbus.kState_Reception
   cpfseq   Modbus.State         ; is state machine in reception state?
     bra    timeoutWaiting       ; no, check if control-wait state

   ; Reception State:  if a timeout occurs here, it must be the inter-character
   ; delay.  We switch to the control-wait state and postpone the timeout until
   ; the end of a full frame timeout period.
   movlw    Modbus.kState_Waiting ; enter control-wait state
   movwf    Modbus.State
   TIMER1   TimeoutDelta         ; reset frame timeout timer
   return

timeoutWaiting:
   ; Check for the next state concerned with timeouts.
   movlw    Modbus.kState_Waiting
   cpfseq   Modbus.State         ; is state machine in control-wait state?
     return                      ; no, we can exit

   ; Control-Wait State:  a timeout here means a full frame timeout period has
   ; elapsed since the last character was received.
   movlw    (1 << Modbus.kRxEvt_CommErr) | (1 << Modbus.kRxEvt_Overrun)
   andwf    Modbus.Event, W      ; were there communication errors?
   bnz      timeoutDone          ; yes, discard the frame

   movlw    0x2                  ; rewind 2 characters
   subwf    Modbus.MsgTail
   movlw    0x0
   subwfb   Modbus.MsgTail + 1

   ; Compute the checksum of the message so it can be validated, along with the
   ; target address.
   lfsr     FSR0, Modbus.kRxBuffer ; FSR0 = message head
   rcall    calcCRC
   call     Modbus.validateMsg
   tstfsz   WREG                 ; was the validation successful?
     bra    timeoutDone          ; no, discard the frame

   ; No reception errors, no checksum errors, and the message is addressed to us.
   movlw    Modbus.kState_MsgQueued ; alert the main event loop that a message has arrived
   movwf    Modbus.State
   bcf      PIE1, TMR1IE         ; disable further timer1 interrupts
   goto     Diag.logRxEvt        ; log the receive event in the event log

timeoutDone:
   bsf      Modbus.Event, Modbus.kRxEvt_NoResponse
   call     Diag.logRxEvt        ; log the receive event in the event log

timeoutIdle:
   ; Become idle, since we know a full frame timeout period has elapsed.
   movlw    Modbus.kState_Idle   ; be ready to receive the next message
   movwf    Modbus.State
   bcf      PIE1, TMR1IE         ; disable timer1 interrupts

   return



;; ----------------------------------------------
;;  void RTU.isrTx()
;;
RTU.isrTx:
   extern   Modbus.Checksum
   extern   Modbus.getFrameByte
   extern   Modbus.MsgHead
   extern   Modbus.MsgTail
   extern   Modbus.State
   extern   USART.send

   ; Determine the state of the state machine.
   movlw    Modbus.kState_EmitStart
   cpfseq   Modbus.State
     bra    txEmission

   ; Emit Start State:  a message reply we want to send is waiting in kTxBuffer,
   ; but we must calculate its checksum before we can transmit it.
   lfsr     FSR0, Modbus.kTxBuffer
   CopyWord FSR0L, Modbus.MsgHead
   rcall    calcCRC

   ; Store the checksum at the end of the message buffer and update the tail.
   movff    Modbus.Checksum, POSTINC0
   movff    Modbus.Checksum + 1, POSTINC0
   CopyWord FSR0L, Modbus.MsgTail

   ; Switch states so we can start sending message bytes.
   movlw    Modbus.kState_Emission
   movwf    Modbus.State

txStash:
   ; Get the next byte from the message buffer.  If none is available, the carry
   ; flag will be set on return.
   call     Modbus.getFrameByte
   bc       txEnd
   goto     USART.send

txEmission:
   ; Check for the next state concerned with transmitted bytes.
   movlw    Modbus.kState_Emission
   cpfseq   Modbus.State
     return

   ; Emission State:  send the next message byte, if available.  If not, we must
   ; have sent the whole message.
   bra      txStash

txEnd:
   ; Set a timer after the last character is transmitted.  Once it expires we can
   ; return to the idle state.
   movlw    Modbus.kState_EmitEnd
   movwf    Modbus.State
   bcf      PIE1, TXIE           ; disable empty transmit buffer interrupts
   TIMER1   FrameTimeout     ; pause before returning to idle state
   return



;; ----------------------------------------------
;;  void calcCRC( const byte buffer[] )
;;
;;  Computes the CRC-16 checksum of the buffer, storing the little-endian
;;  result in MODBUS.Checksum.  The MODBUS generating polynomial is 0xa001,
;;  equivalent to:
;;
;;    x^16 + x^15 + x^13 + x^0
;;
;;  This method expects MODBUS.MsgTail to point one past the last message
;;  buffer byte to be included in the checksum.
;;
calcCRC:
   extern   Modbus.Checksum
   extern   Modbus.MsgTail
   extern   Util.Frame

   ; Compute the message length, which is limited to 256 bytes in RTU mode.  This
   ; means we can ignore the high byte of the message tail pointer, even if it
   ; crosses a page boundary.
   movff    Modbus.MsgTail, Util.Frame
   movf     FSR0L, W
   subwf    Util.Frame           ; compute the 8-bit message length

   ; Initialize the checksum and a pointer to the message buffer.
   setf     Modbus.Checksum      ; CRC starts at 0xffff
   setf     Modbus.Checksum + 1

crcLoop:
   ; Update the checksum with the current byte.
   movf     POSTINC0, W          ; read the byte at head
   xorwf    Modbus.Checksum      ; add it to the checksum's low byte
   movlw    0x08                 ; prepare to loop through all bits
   movwf    Util.Frame + 1

crcXOR:
   ; Shift the checksum one bit.
   bcf      STATUS, C            ; shift 0 into the MSB
   rrcf     Modbus.Checksum + 1
   rrcf     Modbus.Checksum      ; was the LSB set?
   bnc      crcNext              ; no, process the next bit

   ; The LSB was set, so apply the polynomial.
   movlw    0xa0
   xorwf    Modbus.Checksum + 1
   movlw    0x01
   xorwf    Modbus.Checksum

crcNext:
   ; Repeat for every bit in the current byte.
   decfsz   Util.Frame + 1
     bra    crcXOR

   ; Repeat for every byte in the message.
   decfsz   Util.Frame
     bra    crcLoop
   return



   end
