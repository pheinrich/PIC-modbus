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

   ; Variables
   extern   MODBUS.BaudRate
   extern   MODBUS.Checksum
   extern   MODBUS.FrameError
   extern   MODBUS.MsgTail
   extern   MODBUS.Scratch
   extern   MODBUS.State
   extern   UART.LastCharacter

   ; Methods
   extern   DIAG.logRxEvt
   extern   MODBUS.calcParity
   extern   MODBUS.checkParity
   extern   MODBUS.resetFrame
   extern   MODBUS.storeFrameByte
   extern   MODBUS.validateMsg

   global   RTU.init
   global   RTU.rxCharacter
   global   RTU.timeout



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
.modeovr    access_ovr
;; ---------------------------------------------------------------------------

RTU.CharTimeout   res   2     ; the inter-character timeout, in 탎
RTU.FrameTimeout  res   2     ; the inter-frame timeout, in 탎
RTU.TimeoutDelta  res   2     ; difference between timeout values



;; ---------------------------------------------------------------------------
.rtu        code
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
   data     -((kFrequency / 8 * 33) / 9600)     ; 1718.6탎
   data     -((kFrequency / 8 * 77) / 9600)     ; 4010.4탎
   data     -((kFrequency / 8 * 44) / 9600)     ; 4010.4 - 1718.6 ~= 2291.6탎

   ; Character/frame timers at 19200 baud.
   data     -((kFrequency / 8 * 33) / 19200)    ;  859.2탎
   data     -((kFrequency / 8 * 77) / 19200)    ; 2005.2탎
   data     -((kFrequency / 8 * 44) / 19200)    ; 2005.2 - 859.2 ~= 1145.8탎

   ; Character/frame timers for all baud rates greater than 19200.
   data     -((kFrequency / 4000000) *  750)    ;  750탎
   data     -((kFrequency / 4000000) * 1750)    ; 1750탎
   data     -((kFrequency / 4000000) * 1000)    ; 1000탎



;; ----------------------------------------------
;;  void RTU.calcCRC()
;;
;;  Computes the CRC-16 checksum of the message buffer, storing the little-
;;  endian result in MODBUS.Checksum.  The MODBUS generating polynomial is
;;  x^16 + x^15 + x^13 + x^0 (0xa001).
;;
;;  This method expects MODBUS.MsgTail to point one past the last message
;;  buffer byte to be included in the checksum.
;;
RTU.calcCRC:
   ; Compute the message length, which is limited to 256 bytes in RTU mode.  This
   ; means we can ignore the high byte of the message tail pointer, even if it
   ; crosses a page boundary.
   movff    MODBUS.MsgTail, MODBUS.Scratch
   movlw    LOW kMsgBuffer
   subwf    MODBUS.Scratch    ; compute the 8-bit message length

   ; Initialize the checksum and a pointer to the message buffer.
   setf     MODBUS.Checksum   ; CRC starts at 0xffff
   setf     MODBUS.Checksum + 1
   lfsr     FSR0, kMsgBuffer  ; FSR0 = message head

crcLoop:
   ; Update the checksum with the current byte.
   movf     POSTINC0, W       ; read the byte at head
   xorwf    MODBUS.Checksum   ; add it to the checksum's low byte
   movlw    0x08              ; prepare to loop through all bits
   movwf    MODBUS.Scratch + 1

crcXOR:
   ; Shift the checksum one bit.
   bcf      STATUS, C         ; shift 0 into the MSB
   rrcf     MODBUS.Checksum + 1
   rrcf     MODBUS.Checksum   ; was the LSB set?
   bnc      crcNext           ; no, process the next bit

   ; The LSB was set, so apply the polynomial.
   movlw    0xa0
   xorwf    MODBUS.Checksum + 1
   movlw    0x01
   xorwf    MODBUS.Checksum

crcNext:
   ; Repeat for every bit in the current byte.
   decfsz   MODBUS.Scratch + 1
     bra    crcXOR

   ; Repeat for every byte in the message.
   decfsz   MODBUS.Scratch
     bra    crcLoop
   return



;; ----------------------------------------------
;;  void RTU.init()
;;
;;  Initializes the character and frame timeout delay variables based on the
;;  user-configured baud rate.  See ::DelayTable::, above.
;;
RTU.init:
   ; Set up a pointer to our table of timeout values.
   clrf     TBLPTRU           ; always 0 for devices with < 64k program memory
   movlw    HIGH DelayTable
   movwf    TBLPTRH
   movlw    LOW DelayTable
   movwf    TBLPTRL

   ; Advance the table pointer if required for the requested baud rate.
   movlw    kBaud_19200
   cpfslt   MODBUS.BaudRate   ; is baud rate more than 19200?
     bra    check9600         ; no, check lower rate

   ; The requested baud rate is greater than 19200, so advance the table index.
   movlw    0x6
   tblrd*+
   decfsz   WREG
     bra    $-4

check9600:
   ; Advance the table pointer if required for the requested baud rate.
   movlw    kBaud_9600
   cpfslt   MODBUS.BaudRate   ; is baud rate more than 9600?
     bra    copyDelays        ; no, leave the table index undchanged

   ; The requested baud rate is greater than 9600, so advance the table index.
   movlw    0x6
   tblrd*+
   decfsz   WREG
     bra    $-4

copyDelays:
   ; Copy the correct table values to two variables in the access area, starting
   ; with the character timeout.
   lfsr     FSR0, RTU.CharTimeout

   ; Read the correct timing values from the table.
   movlw    0x6
   tblrd*+
   movff    TABLAT, POSTINC0
   decfsz   WREG
     bra    $-6

   ; Initialize the state machine and exit.
   clrf     MODBUS.State
   TIMER1   RTU.FrameTimeout
   return



;; ----------------------------------------------
;;  void RTU.rxCharacter()
;;
;;  Processes a received character according to the current state of the state
;;  machine.  See �2.5.1.1 of the "MODBUS over serial line implementation guide
;;  V1.0".
;;
RTU.rxCharacter:
   ; Determine the state of the state machine, since characters received at diff-
   ; erent times result in different actions.
   movlw    kState_Init
   cpfseq   MODBUS.State      ; is state machine in initial state?
     bra    rxIdle            ; no, check if idle state

   ; Initial State:  characters received now are from a frame already in progress,
   ; so reset the frame timeout timer and wait for it to expire before changing to
   ; the idle state (ready to receive or send).
   bra      rxFrame

rxIdle:
   ; Check for the next state concerned with received characters.
   movlw    kState_Idle
   cpfseq   MODBUS.State      ; is state machine in idle state?
     bra    rxReception       ; no, check if reception state

   ; Idle State:  a character received now indicates the beginning of a new frame,
   ; so begin reception.
   movlw    kState_Reception  ; switch to reception state
   movwf    MODBUS.State
   call     MODBUS.resetFrame
   bra      rxStash           ; start buffering frame characters

rxReception:
   ; Check for the next state concerned with received characters.
   movlw    kState_Reception
   cpfseq   MODBUS.State      ; is state machine in reception state?
     bra    rxCtrlWait        ; no, check if control-wait state

rxStash:
   ; Reception State:  characters received now are buffered until a character gap
   ; is detected.
   call     MODBUS.checkParity; parity errors don't stop reception, just invalidate frame
   movf     UART.LastCharacter, W
   call     MODBUS.storeFrameByte
   TIMER1   RTU.CharTimeout   ; reset the character timeout timer
   return

rxCtrlWait:
   ; Check for the next state concerned with received characters.
   movlw    kState_Waiting
   cpfseq   MODBUS.State      ; is state machine in control-wait state?
     return                   ; no, we can exit

   ; Control-Wait State:  characters received now indicate a partial frame was
   ; received.  Now we must wait for a full frame timeout period to elapse before
   ; it's safe to go idle again.
   setf     MODBUS.FrameError ; note that the frame should be discarded

rxFrame:
   ; Reset the frame timeout timer.
   TIMER1   RTU.FrameTimeout
   return



;; ----------------------------------------------
;;  void RTU.timeout()
;;
RTU.timeout:
   ; Determine the state of the state machine, since timeouts that occur at diff-
   ; erent times result in different actions.
   movlw    kState_Init
   cpfseq   MODBUS.State      ; is state machine in initial state?
     bra    timeoutEmission   ; no, check if emission state

   ; Initial State:  a timeout here indicates a full frame timeout period has
   ; elapsed.  It's now safe to enter the idle state.
   bra      timeoutIdle

timeoutEmission:
   ; Check for the next state concerned with timeouts.
   movlw    kState_Emission
   cpfseq   MODBUS.State      ; is state machine in emission state?
     bra    timeoutReception  ; no, check if reception state

   ; Emission State:  a timeout here indicates our emission has been set off by a
   ; full frame timeout period.  We're idle again.
   bra      timeoutIdle

timeoutReception:
   ; Check for the next state concerned with timeouts.
   movlw    kState_Reception
   cpfseq   MODBUS.State      ; is state machine in reception state?
     bra    timeoutWaiting    ; no, check if control-wait state

   ; Reception State:  if a timeout occurs here, it must be the inter-character
   ; delay.  We switch to the control-wait state and postpone the timeout until
   ; the end of a full frame timeout period.
   movlw    kState_Waiting    ; enter control-wait state
   movwf    MODBUS.State
   TIMER1   RTU.TimeoutDelta  ; reset frame timeout timer
   return

timeoutWaiting:
   ; Check for the next state concerned with timeouts.
   movlw    kState_Waiting
   cpfseq   MODBUS.State      ; is state machine in control-wait state?
     return                   ; no, we can exit

   ; Control-Wait State:  a timeout here means a full frame timeout period has
   ; elapsed since the last character was received.  The last two message bytes
   ; hold the checksum computed by the sender (which we don't want to include in
   ; our checksum calculation), so pull the tail in by 2.
   movlw    0x2
   subwf    MODBUS.MsgTail
   movlw    0x0
   subwfb   MODBUS.MsgTail + 1

   ; Compute the checksum of the message so it can be validated, along with the
   ; target address.
   rcall    RTU.calcCRC
   call     MODBUS.validateMsg
   tstfsz   WREG              ; was the validation successful?
     bra    timeoutDone       ; no, discard the frame

   ; No reception errors, no checksum errors, and the message is addressed to us.
   movlw    kState_MsgQueued  ; alert the main event loop that a message has arrived
   movwf    MODBUS.State
   goto     DIAG.logRxEvt     ; log the receive event in the event log

timeoutDone:
   call     DIAG.logRxEvt     ; log the receive event in the event log

timeoutIdle:
   ; Become idle, since we know a full frame timeout period has elapsed.
   movlw    kState_Idle       ; be ready to receive the next message
   movwf    MODBUS.State
   bcf      PIE1, TMR1IE      ; disable timer1 interrupts

   return



   end
