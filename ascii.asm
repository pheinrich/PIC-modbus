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



   include "modbus.inc"

   extern   CONF.ParityCheck
   extern   MODBUS.FrameError
   extern   MODBUS.State
   extern   UART.LastCharacter

   extern   MODBUS.resetFrame
   extern   MODBUS.storeFrameByte

   global   ASCII.Delimiter

   global   ASCII.init
   global   ASCII.rxCharacter
   global   ASCII.timeout



; Count of timer1 overflows in 1 second (~= 77 @ 20 MHz).
kOneSecond        equ   1 + (kFrequency / (4 * 65536))

; State machine constants.
kState_Idle       equ   0
kState_EmitStart  equ   1
kState_Emission   equ   2
kState_EmitEnd    equ   3
kState_Reception  equ   4
kState_Waiting    equ   5



;; ----------------------------------------------
;;  Macro RESET_TIMER1
;;
;;  The RESET_TIMER1 macro starts Timer1 in 16-bit timer mode with the max-
;;  imum possible period, corresponding to a total delay of about 0.013s.
;;  To measure longer intervals we must count repeated overflows, so this
;;  macro also clears that counter.
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
ASCII.IsOddNybble    res   1     ; 0 = false, 255 = true
ASCII.LRC            res   1     ; Longitudinal Redundancy Check
ASCII.Nybbles        res   1     ; buffers successive hex character codes
ASCII.Scratch        res   1     ; work variable
ASCII.Timeouts       res   1     ; supports extra long (>1s) delays



;; ---------------------------------------------------------------------------
.ascii      code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  void ASCII.checkParity()
;;
ASCII.checkParity:
   return



;; ----------------------------------------------
;;  void ASCII.init()
;;
ASCII.init:
   movlw    '\n'              ; delimiter defaults to linefeed
   movwf    ASCII.Delimiter
   clrf     MODBUS.State      ; start in idle state
   return



;; ----------------------------------------------
;;  void ASCII.resetFrame()
;;
ASCII.resetFrame:
   ; Do the basic frame reset, which is mode-independent.
   call     MODBUS.resetFrame

   ; Initialize some variables used only in ASCII mode.
   clrf     ASCII.Nybbles
   setf     ASCII.IsOddNybble
   clrf     ASCII.LRC
   return



;; ----------------------------------------------
;;  void ASCII.rxCharacter()
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
   rcall    ASCII.resetFrame

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
   rcall    ASCII.checkParity ; parity errors don't stop reception, just invalidate frame
   rcall    ASCII.storeFrameByte
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

   ; A frame delimiter was received.  If no errors were detected with the frame,
   ; process it (otherwise it will be discarded).
   ; process frame (or not)

   ; Become idle, since we received a complete frame.
   movlw    kState_Idle       ; enter idle state
   movwf    MODBUS.State
   bcf      PIE1, TMR1IE      ; disable timer1 interrupts
   return

   

;; ----------------------------------------------
;;  void ASCII.storeFrameByte()
;;
ASCII.storeFrameByte:
   ; Update the Longitudinal Redundancy Checksum.
   movf     UART.LastCharacter, W
   addwf    ASCII.LRC

   ; Convert the ASCII character code into the integer value corresponding to the
   ; hexadecimal digit it represents.  '0'-'9' become 0-9; 'A'-'F' and 'a'-'f'
   ; become 10-15.
   addlw    0x9f
   bnn      adjust            ; if positive, character was 'a' to 'f'
   addlw    0x20              ; otherwise, shift to next range of digits
   bnn      adjust            ; if now positive, character was 'A' to 'F'
   addlw    0x7               ; otherwise, character must have been '0' to '9'

adjust:
   addlw    0xa               ; shift the result to account for offset
   andlw    0xf               ; clamp the value to one nybble
   movwf    ASCII.Scratch

   ; Shift the nybble if necessary, depending on whether it's high or low.
   tstfsz   ASCII.IsOddNybble ; is this the second nybble in the byte?
     swapf  ASCII.Scratch, W  ; no, shift it up four bits
   iorwf    ASCII.Nybbles     ; combine nybbles to form a byte

   ; Toggle flag so we'll update alternating nybbles.
   comf     ASCII.IsOddNybble
   bz       stored            ; if we we processed a whole byte, we're done

   ; We have two nybbles, so store the byte they form in the message buffer.
   movf     ASCII.Nybbles, W
   call     MODBUS.storeFrameByte
   clrf     ASCII.Nybbles     ; clear the nybble buffer

stored:
   return



;; ----------------------------------------------
;;  void ASCII.timeout()
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



   end
