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



   #include "private.inc"

   ; Global Variables
   global   Modbus.Address
   global   Modbus.Event
   global   Modbus.State

   ; Public Methods
   global   Modbus.builtin
   global   Modbus.dispatchMsg
   global   Modbus.idle
   global   Modbus.illegalAddress
   global   Modbus.illegalData
   global   Modbus.illegalFunction
   global   Modbus.init
   global   Modbus.isr
   global   Modbus.replyMsg

   ; Dependencies
   extern   ASCII.init
   extern   ASCII.isrTimeout
   extern   Diag.diagnostics
   extern   Diag.getEventCount
   extern   Diag.getEventLog
   extern   Diag.getExceptions
   extern   Diag.init
   extern   Diag.noResponse
   extern   Diag.Options
   extern   Frame.begin
   extern   Frame.end
   extern   Frame.endWithError
   extern   RTU.init
   extern   RTU.isrTimeout
   extern   USART.init
   extern   USART.isr
   extern   Util.Frame
   extern   Util.Save
   extern   VTable.dispatch



;; ---------------------------------------------------------------------------
                        udata_acs
;; ---------------------------------------------------------------------------

Modbus.Address          res   1     ; uniquely identifies this device on the bus
Modbus.Event            res   1     ; kRxEvt_CommErr, kRxEvt_Broadcast, kTxEvt_Abort, etc.
Modbus.State            res   1     ; current state of the state machine



;; ---------------------------------------------------------------------------
.modbus                 code
;; ---------------------------------------------------------------------------

BuiltinVTbl:
   data     Modbus.kGetExceptions, Diag.getExceptions
   data     Modbus.kDiagnostics, Diag.diagnostics
   data     Modbus.kGetEventCount, Diag.getEventCount
   data     Modbus.kGetEventLog, Diag.getEventLog
   data     0xffff, Modbus.illegalFunction



;; ----------------------------------------------
;;  void Modbus.builtin( frame[0..1] funcKey )
;;
Modbus.builtin:
   SetTableBase BuiltinVTbl
   goto     VTable.dispatch



;; ----------------------------------------------
;;  void Modbus.dispatchMsg( TBLPTR vtable )
;;
Modbus.dispatchMsg:
   movff    Modbus.kRxFunction, Util.Frame
   clrf     Util.Frame + 1
   goto     VTable.dispatch



;; ----------------------------------------------
;;  void Modbus.idle()
;;
;;  Turns off the tranceiver bus master before entering the idle state.  This
;;  method waits for the current byte (if any) to be transmitted completely,
;;  then disables the tranceiver's transmitter, putting it in a high-impedance
;;  state.  High-Z mode allows other transmitters to use the bus.
;;
;;  The last step is to update the actual value stored in the state machine,
;;  so it knows it's free to start processing new messages.
;;
Modbus.idle:
   ; Turn off transceiver bus master.
   btfss    TXSTA, TRMT             ; is transmit shift register empty?
     bra    $-2                     ; no, wait for last byte to be transmitted
   bcf      PORTC, RC0              ; put transmitter in high-Z mode

   ; Update state machine.
   movlw    Modbus.kState_Idle
   movwf    Modbus.State
   return



;; ----------------------------------------------
;;  void Modbus.illegalAddress()
;;
;;  Creates an exception response with an error code of 2, which indicates an
;;  invalid data address.
;;
Modbus.illegalAddress:
   call     Frame.begin
   movlw    Modbus.kErrorBadAddress
   goto     Frame.endWithError



;; ----------------------------------------------
;;  void Modbus.illegalData()
;;
;;  Creates an exception response with an error code of 3, which indicates a
;;  value in the query data field isn't allowed.
;;
Modbus.illegalData:
   call     Frame.begin
   movlw    Modbus.kErrorBadData
   goto     Frame.endWithError



;; ----------------------------------------------
;;  void Modbus.illegalFunction()
;;
;;  Creates an exception response with an error code of 1, which indicates an
;;  invalid function or subfunction has been requested.
;;
Modbus.illegalFunction:
   call     Frame.begin
   movlw    Modbus.kErrorBadFunction
   goto     Frame.endWithError



;; ----------------------------------------------
;;  void Modbus.init( frame[0] ascii, frame[1] baud, frame[2] enum parity )
;;
;;  Initializes a state machine appropriate to the operating mode specified,
;;  then resets the diagnostic registers and clears the event log.  Finally,
;;  the hardware USART is initialized with baud rate and software parity
;;  requested.
;;
Modbus.init:
   ; Initialize a pin to control tranceiver bus master.
   bcf      TRISC, RC0              ; make RC0/T1OSO/T1CKI an output, if not already

   ; The operating mode determines which state machine will be active.
   movf     Util.Frame, F           ; are we in ASCII (7-bit) mode?
   bz       initRTU                 ; no, initialize RTU mode
   call     ASCII.init              ; yes, initialize ASCII mode
   bra      initDiag

initRTU:
   call     RTU.init

initDiag:
   ; Save the hardware set-up for last, since the state machine initialization
   ; above needs to hook the rx/tx events first.  Note that the stackframe must
   ; be preserved until the then, since it holds the USART initialization para-
   ; meters.
   call     Diag.init               ; reset diagnostic registers and event log
   goto     USART.init              ; initialize the serial port



;; ----------------------------------------------
;;  void Modbus.isr()
;;
Modbus.isr:
   ; Service the serial hardware.  We have already hooked its transmit and re-
   ; ceive interrupt methods, which ensures the correct state machine will get a
   ; chance to process those events.
   call     USART.isr
   
   ; Determine if our timer overflowed.
   btfss    PIE1, TMR1IE            ; are timer1 interrupts enabled?
     return                         ; no, we can exit
   btfss    PIR1, TMR1IF            ; yes, has timer1 expired?
     return                         ; no, we're done

   ; A timer1 event did occur.  We must delegate this event ourselves, since the
   ; USART isn't interested in it and doesn't provide a convenient hook.
   bcf      PIR1, TMR1IF            ; clear the timer interrupt flag
   btfss    RCSTA, RX9              ; are we in RTU (8-bit) mode?
     goto   ASCII.isrTimeout        ; no, the ASCII (7-bit) state machine takes over
   goto     RTU.isrTimeout          ; yes, the RTU state machine takes over



;; ----------------------------------------------
;;  void Modbus.replyMsg()
;;
;;  Attempts to send the message in the message buffer, which is always in re-
;;  ply to a message we previously received.
;;
Modbus.replyMsg:
   ; This method does nothing if a complete, error-free server request isn't al-
   ; ready available in the message buffer.
   movlw    Modbus.kState_MsgQueued
   cpfseq   Modbus.State            ; is there a message waiting for us?
     return                         ; no, bail

   ; If the last message received was a broadcast message, or this device is in
   ; listen-only mode, maintain radio silence.
   movlw    (1 << Modbus.kDiag_Broadcast) | (1 << Modbus.kDiag_ListenOnly)
   andwf    Diag.Options, W         ; should we reply?
   bz       reply                   ; yes, reply normally

   ; Monitoring messages only, so we're done with this one.
   rcall    Modbus.idle
   goto     Diag.noResponse

reply:
   ; Change state and enable the character transmitted interrupt.  If the transmit
   ; buffer is empty, this will fire immediately, otherwise it will trigger after
   ; the current byte is transmitted.
   movlw    Modbus.kState_EmitStart ; prepare to transmit the message
   movwf    Modbus.State
   bsf      PORTC, RC0              ; enable tranceiver bus master
   bsf      PIE1, TXIE              ; enable the interrupt
   return



   end
