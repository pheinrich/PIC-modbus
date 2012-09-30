;; ---------------------------------------------------------------------------
;;
;;  PIC Modbus
;;  Copyright © 2006,2008  Peter Heinrich
;;
;;  This program is free software; you can redistribute it and/or
;;  modify it under the terms of the GNU General Public License
;;  as published by the Free Software Foundation; either version 2
;;  of the License, or (at your option) any later version.
;;
;;  Linking this library statically or dynamically with other modules
;;  is making a combined work based on this library. Thus, the terms
;;  and conditions of the GNU General Public License cover the whole
;;  combination.
;;
;;  As a special exception, the copyright holders of this library give
;;  you permission to link this library with independent modules to
;;  produce an executable, regardless of the license terms of these
;;  independent modules, and to copy and distribute the resulting
;;  executable under terms of your choice, provided that you also meet,
;;  for each linked independent module, the terms and conditions of the
;;  license of that module. An independent module is a module which is
;;  not derived from or based on this library. If you modify this
;;  library, you may extend this exception to your version of the
;;  library, but you are not obligated to do so. If you do not wish to
;;  do so, delete this exception statement from your version.
;;
;;  This program is distributed in the hope that it will be useful,
;;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;  GNU General Public License for more details.
;;
;;  You should have received a copy of the GNU General Public License
;;  along with this program; if not, write to the Free Software
;;  Foundation, Inc., 51 Franklin Street, Boston, MA  02110-1301, USA.
;;
;; ---------------------------------------------------------------------------



   include "private.inc"

   ; Methods
   global   RTU.init
   global   RTU.isrRx
   global   RTU.isrTimeout
   global   RTU.isrTx

   ; Dependencies
   extern   Diag.logRxEvt
   extern   Diag.logTxEvt
   extern   Frame.Checksum
   extern   Frame.isValid
   extern   Frame.Head
   extern   Frame.reset
   extern   Frame.rxByte
   extern   Frame.txByte
   extern   Modbus.idle
   extern   Modbus.Event
   extern   Modbus.State
   extern   USART.HookRx
   extern   USART.HookTx
   extern   USART.Read
   extern   USART.send
   extern   Util.Frame




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
;;  0.2µs (200ns).  This macro should only be included in code called when
;;  Timer1 interrupts are disabled.
;;
TIMER1      macro timeout
   ; Stop the timer and prepare to initialize its countdown period.
   movlw    b'10000000'
            ; 1------- RD16         ; enable 16-bit read/write operations
            ; -X------              ; [unimplemented]
            ; --00---- T1CKPS       ; 1:1 prescaler
            ; ----0--- T1OSCEN      ; disable external oscillator
            ; -----X-- T1SYNC       ; [not used when using internal clock]
            ; ------0- TMR1CS       ; use internal clock
            ; -------0 TMR1ON       ; disable timer, for now
   movwf    T1CON

   ; Set the countdown period.
   movff    timeout + 1, TMR1H
   movff    timeout, TMR1L

   ; Start the timer.
   bsf      PIE1, TMR1IE            ; enable associated overflow interrupt
   bsf      T1CON, TMR1ON           ; enable timer
   bcf      PIR1, TMR1IF            ; clear the timer interrupt flag
   endm



;; ---------------------------------------------------------------------------
.modeovr                access_ovr
;; ---------------------------------------------------------------------------

CharTimeout             res   2     ; the inter-character timeout, in µs
FrameTimeout            res   2     ; the inter-frame timeout, in µs
TimeoutDelta            res   2     ; difference between timeout values



;; ---------------------------------------------------------------------------
.rtu                    code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  Delay Timeout Table
;;
;;  Precalculate the inter-character and inter-frame timeout delays (in µs)
;;  for 9600, 19200, and >19200 baud.  For baud rates greater than 19200, the
;;  timeouts are fixed at 750µs and 1750µs, respectively, to reduce the CPU
;;  overhead of more frequent timer processing (see the Remark in §2.5.1.1 of
;;  the MODBUS over Serial Line Specification and Implementation guide V1.0).
;;  At slower baud rates, the inter-character timeout is 1.5 times the char-
;;  acter time, the time to transmit one character; the inter-frame timeout is
;;  3.5 times the character time.
;;
;;  Note that the values below are negative, since the timer interrupt fires
;;  when the timer overflows from 0xffff to 0x0000.  Also note that the timer
;;  is incremented every instruction cycle (kFrequency / 4), not every CPU
;;  cycle.
;;
DelayTable:
   ; Character/frame timers at 9600 baud.
   data     0xffff & -((3 * kFrequency) / (9600 << 3))   ; 156.25µs
   data     0xffff & -((7 * kFrequency) / (9600 << 3))   ; 364.58µs
   data     0xffff & -((4 * kFrequency) / (9600 << 3))   ; 354.58 - 156.25 ~= 208.33µs

   ; Character/frame timers at 19200 baud.
   data     0xffff & -((3 * kFrequency) / (19200 << 3))  ; 78.125µs
   data     0xffff & -((7 * kFrequency) / (19200 << 3))  ; 182.29µs
   data     0xffff & -((4 * kFrequency) / (19200 << 3))  ; 182.29 - 78.125 ~= 104.166µs

   ; Character/frame timers for all baud rates greater than 19200.
   data     0xffff & -((kFrequency / 4000000) *  750)    ;  750µs
   data     0xffff & -((kFrequency / 4000000) * 1750)    ; 1750µs
   data     0xffff & -((kFrequency / 4000000) * 1000)    ; 1000µs



;; ----------------------------------------------
;;  void RTU.init()
;;
;;  Initializes the character and frame timeout delay variables based on the
;;  user-configured baud rate.  See ::DelayTable::, above.
;;
RTU.init:
   ; Set up a pointer to our table of timeout values.
   SetTableBase DelayTable

   ; Advance the table pointer if required for the requested baud rate.
   movlw    USART.kBaud_19200
   cpfslt   SPBRG                   ; is baud rate more than 19200?
     bra    check9600               ; no, check lower rate

   ; The requested baud rate is greater than 19200, so advance the table index.
   movlw    0x6
   tblrd*+
   decfsz   WREG, W
     bra    $-4

check9600:
   ; Advance the table pointer if required for the requested baud rate.
   movlw    USART.kBaud_9600
   cpfslt   SPBRG                   ; is baud rate more than 9600?
     bra    copyDelays              ; no, leave the table index unchanged

   ; The requested baud rate is greater than 9600, so advance the table index.
   movlw    0x6
   tblrd*+
   decfsz   WREG, W
     bra    $-4

copyDelays:
   ; Copy the correct table values to two variables in the access area, starting
   ; with the character timeout.
   lfsr     FSR0, CharTimeout

   ; Read the correct timing values from the table.
   movlw    0x6
   tblrd*+
   movff    TABLAT, POSTINC0
   decfsz   WREG, W
     bra    $-6

   ; Hook the serial port.
   SetWord RTU.isrRx, USART.HookRx  ; set the reception callback
   SetWord RTU.isrTx, USART.HookTx  ; set the transmission callback

   ; Initialize the state machine.
   clrf     Modbus.State
   TIMER1   FrameTimeout

   return



;; ----------------------------------------------
;;  void RTU.isrRx()
;;
;;  Processes a received binary character according to the current state of
;;  the state machine.  See §2.5.1.1 of the "MODBUS over serial line imple-
;;  mentation guide V1.0".
;;
RTU.isrRx:
   ; Determine the state of the state machine, since characters received at diff-
   ; erent times result in different actions.
   movlw    Modbus.kState_Init
   cpfseq   Modbus.State            ; is state machine in initial state?
     bra    rxIdle                  ; no, check if idle state

   ; Initial State:  characters received now are from a frame already in progress,
   ; so reset the frame timeout timer and wait for it to expire before changing to
   ; the idle state (ready to receive or send).
   bra      rxFrame

rxIdle:
   ; Check for the next state concerned with received characters.
   movlw    Modbus.kState_Idle
   cpfseq   Modbus.State            ; is state machine in idle state?
     bra    rxReception             ; no, check if reception state

   ; Idle State:  a character received now indicates the beginning of a new frame,
   ; so begin reception.
   movlw    Modbus.kState_Reception ; switch to reception state
   movwf    Modbus.State

   lfsr     FSR0, Modbus.kRxBuffer
   call     Frame.reset
   bra      rxStash                 ; start buffering frame characters

rxReception:
   ; Check for the next state concerned with received characters.
   movlw    Modbus.kState_Reception
   cpfseq   Modbus.State            ; is state machine in reception state?
     bra    rxCtrlWait              ; no, check if control-wait state

rxStash:
   ; Reception State:  characters received now are buffered until a character gap
   ; is detected.
   movf     USART.Read, W
   call     Frame.rxByte
   TIMER1   CharTimeout             ; reset the character timeout timer
   return

rxCtrlWait:
   ; Check for the next state concerned with received characters.
   movlw    Modbus.kState_Waiting
   cpfseq   Modbus.State            ; is state machine in control-wait state?
     return                         ; no, we can exit

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
   ; Determine the state of the state machine, since timeouts that occur at diff-
   ; erent times result in different actions.
   movlw    Modbus.kState_Init
   cpfseq   Modbus.State            ; is state machine in initial state?
     bra    timeoutEmitEnd          ; no, check if emit end state

   ; Initial State:  a timeout here indicates a full frame timeout period has
   ; elapsed.  It's now safe to enter the idle state.
   bra      timeoutIdle

timeoutEmitEnd:
   ; Check for the next state concerned with timeouts.
   movlw    Modbus.kState_EmitEnd
   cpfseq   Modbus.State            ; is state machine in emit end state?
     bra    timeoutReception        ; no, check if reception state

   ; Emit End State:  a timeout here indicates our emission has been set off by a
   ; full frame timeout period.  We're idle again.
   call     Diag.logTxEvt
   bra      timeoutIdle

timeoutReception:
   ; Check for the next state concerned with timeouts.
   movlw    Modbus.kState_Reception
   cpfseq   Modbus.State            ; is state machine in reception state?
     bra    timeoutWaiting          ; no, check if control-wait state

   ; Reception State:  if a timeout occurs here, it must be the inter-character
   ; delay.  We switch to the control-wait state and postpone the timeout until
   ; the end of a full frame timeout period.
   movlw    Modbus.kState_Waiting   ; enter control-wait state
   movwf    Modbus.State
   TIMER1   TimeoutDelta            ; reset frame timeout timer
   return

timeoutWaiting:
   ; Check for the next state concerned with timeouts.
   movlw    Modbus.kState_Waiting
   cpfseq   Modbus.State            ; is state machine in control-wait state?
     return                         ; no, we can exit

   ; Control-Wait State:  a timeout here means a full frame timeout period has
   ; elapsed since the last character was received.
   movlw    (1 << Modbus.kRxEvt_CommErr) | (1 << Modbus.kRxEvt_Overrun)
   andwf    Modbus.Event, W         ; were there communication errors?
   bnz      timeoutDone             ; yes, discard the frame

   movlw    0x2                     ; rewind 2 characters
   subwf    Frame.Head, F
   movlw    0x0
   subwfb   Frame.Head + 1, F

   ; Compute the checksum of the message so it can be validated, along with the
   ; target address.
   lfsr     FSR0, Modbus.kRxBuffer  ; FSR0 = message tail
   rcall    calcCRC
   call     Frame.isValid
   andlw    0xff                    ; was the validation successful?
   bz       timeoutDone             ; no, discard the frame

   ; No reception errors, no checksum errors, and the message is addressed to us.
   movlw    Modbus.kState_MsgQueued ; alert the main event loop that a message has arrived
   movwf    Modbus.State
   bcf      PIE1, TMR1IE            ; disable further timer1 interrupts
   goto     Diag.logRxEvt           ; log the receive event in the event log

timeoutDone:
   bsf      Modbus.Event, Modbus.kRxEvt_NoResponse
   call     Diag.logRxEvt           ; log the receive event in the event log

timeoutIdle:
   ; Become idle, since we know a full frame timeout period has elapsed.
   call     Modbus.idle             ; be ready to receive the next message
   bcf      PIE1, TMR1IE            ; disable timer1 interrupts
   return



;; ----------------------------------------------
;;  void RTU.isrTx()
;;
RTU.isrTx:
   ; Determine the state of the state machine.
   movlw    Modbus.kState_EmitStart
   cpfseq   Modbus.State
     bra    txEmission

   ; Emit Start State:  a message reply we want to send is waiting in kTxBuffer,
   ; but we must calculate its checksum before we can transmit it.
   lfsr     FSR0, Modbus.kTxBuffer
   rcall    calcCRC

   ; Store the checksum at the end of the message buffer and update the tail.
   movff    Frame.Checksum, POSTINC0
   movff    Frame.Checksum + 1, POSTINC0
   CopyWord FSR0L, Frame.Head

   ; Switch states so we can start sending message bytes.
   incf     Modbus.State, F         ; state = Modbus.kState_Emission

txStash:
   ; Get the next byte from the message buffer.  If none is available, the carry
   ; flag will be set on return.
   call     Frame.txByte
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
   bcf      PIE1, TXIE              ; disable empty transmit buffer interrupts
   TIMER1   FrameTimeout            ; pause before returning to idle state
   return



;; ----------------------------------------------
;;  void calcCRC( FSR0 txBuffer )
;;
;;  Computes the CRC-16 checksum of the buffer, storing the little-endian
;;  result in Frame.Checksum.  The Modbus generating polynomial is 0xa001,
;;  equivalent to:
;;
;;    x^16 + x^15 + x^13 + x^0
;;
;;  This method expects Frame.Head to point one past the last message buffer
;;  byte to be included in the checksum.
;;
calcCRC:
   ; Compute the message length, which is limited to 256 bytes in RTU mode.  This
   ; means we can ignore the high byte of the message tail pointer, even if it
   ; crosses a page boundary.
   movff    Frame.Head, Util.Frame
   movf     FSR0L, W
   subwf    Util.Frame, F           ; frame[0] = 8-bit message length

   ; Initialize the checksum and a pointer to the message buffer.
   setf     Frame.Checksum
   setf     Frame.Checksum + 1      ; crc = 0xffff, initially

crcLoop:
   ; Update the checksum with the current byte.
   movf     POSTINC0, W             ; read the byte at tail
   xorwf    Frame.Checksum, F       ; add it to the checksum's low byte
   movlw    0x08                    ; prepare to loop through all bits
   movwf    Util.Frame + 1

crcXOR:
   ; Shift the checksum one bit.
   bcf      STATUS, C               ; shift 0 into the MSB
   rrcf     Frame.Checksum + 1, F
   rrcf     Frame.Checksum, F       ; was the LSB set?
   bnc      crcNext                 ; no, process the next bit

   ; The LSB was set, so apply the polynomial.
   movlw    0xa0
   xorwf    Frame.Checksum + 1, F
   movlw    0x01
   xorwf    Frame.Checksum, F

crcNext:
   ; Repeat for every bit in the current byte.
   decfsz   Util.Frame + 1, F
     bra    crcXOR

   ; Repeat for every byte in the message.
   decfsz   Util.Frame, F
     bra    crcLoop
   return



   end
