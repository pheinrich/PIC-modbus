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
   global   Frame.Checksum
   global   Frame.Head
   global   Frame.Tail

   ; Public Methods
   global   Frame.begin
   global   Frame.beginEcho
   global   Frame.beginWithSub
   global   Frame.end
   global   Frame.endWithError
   global   Frame.isValid
   global   Frame.reset
   global   Frame.rxByte
   global   Frame.txByte

   ; Dependencies
   extern   Diag.Options
   extern   Modbus.Address
   extern   Modbus.Event



;; ---------------------------------------------------------------------------
                        udata_acs
;; ---------------------------------------------------------------------------

Frame.Checksum         	res   2     ; LRC or CRC, depending on mode (ASCII or RTU)
Frame.Head              res   2     ; points to next location to be read or written
Frame.Tail              res   2     ; points to the first byte in the message



;; ---------------------------------------------------------------------------
.frame                 	code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  FSR0 Frame.begin()
;;
;;  Prepares the transmission buffer to hold a response frame.  The head and
;;  tail pointers are reset to the start of the buffer, then the slave id and
;;  function code from the reception buffer are copied over.
;;
;;  When this method returns, FSR0 points one byte past the last byte written.
;;  Other code can then append additional data before calling Frame.end() or
;;  Frame.endWithError().
;;
Frame.begin:
   ; Save a pointer to the beginning of the message.
   lfsr     FSR0, Modbus.kTxBuffer
   rcall    Frame.reset
   SetBank Modbus.kRxFunction

   ; Copy the device id from the original message, as well as the function code.
   movff    Modbus.kRxSlave, POSTINC0
   movff    Modbus.kRxFunction, POSTINC0
   return



;; ----------------------------------------------
;;  FSR0 Frame.beginEcho()
;;
;;  Prepares the transmission buffer with a copy of the received frame (it's
;;  common for Modbus functions to simply echo request data).  On completion,
;;  FSR0 will point one byte past the last one written to the new frame.  The
;;  caller may then use Frame.end() or Frame.endWithError() to terminate the
;;  frame, as usual.
;;
Frame.beginEcho:
   ; Set pointers to both buffers (Rx and Tx).
   lfsr     FSR0, Modbus.kTxBuffer
   lfsr     FSR1, Modbus.kRxBuffer

echoLoop:
   ; Have we advanced our pointer past the last received byte?
   movf     FSR1L, W
   cpfseq   Frame.Head              ; low words match?
     bra    echoNext                ; no, keep copying

   movf     FSR1H, W
   cpfseq   Frame.Head + 1          ; high words match?
     bra    echoNext                ; no, keep copying

   ; We've copied the whole reception buffer, so exit.
   rcall    Frame.reset             ; resets Frame.Head and Frame.Tail from FSR0
   SetWord Modbus.kTxBuffer, Frame.Tail ; correct tail (buffer start) pointer
   SetBank Modbus.kRxFunction
   return

echoNext:
   ; Copy the next byte, advancing both pointers.
   movff    POSTINC1, POSTINC0
   bra      echoLoop



;; ----------------------------------------------
;;  FSR0 Frame.beginWithSub()
;;
;;  Initializes the transmission frame in preparation for a response which in-
;;  cludes both function and subfunction codes.
;;  
Frame.beginWithSub:
   rcall    Frame.begin
   movff    Modbus.kRxSubFunction, POSTINC0
   movff    Modbus.kRxSubFunction + 1, POSTINC0
   return



;; ----------------------------------------------
;;  void Frame.end( FSR0 nextByte )
;;
;;  Terminates the frame by saving the current pointer as the new head.  It
;;  will then be used to compute the frame length.
;;
Frame.end:
   CopyWord FSR0L, Frame.Head
   return



;; ----------------------------------------------
;;  STATUS<C> Frame.endWithError( WREG exception, FSR0 nextByte  )
;;
;;  Discards the current contents of the frame and reinitializes it to be an
;;  exception response.  This method also sets the correct event flags based
;;  on the specified exception code, then terminates the frame.
;;
;;  This method always returns with STATUS<C> clear to indicate the frame now
;;  contains an exception response.
;;
Frame.endWithError:
   lfsr     FSR0, Modbus.kTxFunction

   ; Turn this into an exception PDU.
   bsf      POSTINC0, 7             ; set the function code's high bit
   movwf    INDF0                   ; store the specified exception code

   ; Based on the exception code, we need to update our event log.
   movlw    Modbus.kErrorMemoryParity
   cpfslt   INDF0
     bra    errorDone

   movlw    Modbus.kErrorNAKSent
   cpfslt   INDF0
     bsf    Modbus.Event, Modbus.kTxEvt_NAKEx

   movlw    Modbus.kErrorBusy
   cpfslt   INDF0
     bsf    Modbus.Event, Modbus.kTxEvt_BusyEx

   movlw    Modbus.kErrorFailure
   cpfslt   INDF0
     bsf    Modbus.Event, Modbus.kTxEvt_AbortEx
   bsf      Modbus.Event, Modbus.kTxEvt_ReadEx

errorDone:
   ; Tidy up on our way out.
   movf     POSTINC0, W             ; advance the pointer one byte
   bcf      STATUS, C               ; announce the exception response
   bra      Frame.end



;; ----------------------------------------------
;;  WREG Frame.isValid()
;;
;;  Returns true (0xff) if the current frame is "valid," in that it's addressed
;;  to this device (or is a broadcast message) and its checksum is correct.  If
;;  not, this method returns false (0x00).
;;
Frame.isValid:
   ; Verify the message is addressed to this device.
   bsf      Diag.Options, Modbus.kDiag_Broadcast ; assume broadcast message
   SetBank Modbus.kRxBuffer
   movf     Modbus.kRxBuffer, W     ; is this a broadcast message (0 == address)?
   bz       valChecksum             ; yes, validate the checksum

   bcf      Diag.Options, Modbus.kDiag_Broadcast ; no, clear our assumption
   cpfseq   Modbus.Address          ; is it addressed to this specific device?
     retlw  0x00                    ; no, discard frame

valChecksum:
   ; This message is addressed to us.
   bsf      Modbus.Event, Modbus.kRxEvt_SlaveMsg

   ; Set a pointer to the last byte in the message buffer.
   CopyWord Frame.Head, FSR0L
   bsf      Modbus.Event, Modbus.kRxEvt_CommErr ; assume checksum fails

   ; Compare the checksum included in the message to the one we calculated.
   movf     POSTINC0, W             ; fetch the LSB
   cpfseq   Frame.Checksum          ; does it match the computed value?
     retlw  0x00                    ; no, discard the frame  TODO: log event

   movf     INDF0, W                ; yes, fetch the MSB
   cpfseq   Frame.Checksum + 1      ; does it match the computed value?
     retlw  0x00                    ; no, discard the frame  TODO: log event

   ; Success, so clear our earlier assumption about a bad checksum.
   bcf      Modbus.Event, Modbus.kRxEvt_CommErr
   retlw    0xff



;; ----------------------------------------------
;;  void Frame.reset( FSR0 buffer )
;;
;;  Initializes the message frame for fresh reception or transmission.  This
;;  method stashes the base pointer for later use by related routines.
;;
Frame.reset:
   ; Reset the pointer to the beginning of the buffer.
   movf     FSR0L, W
   movwf    Frame.Head
   movwf    Frame.Tail

   movf     FSR0H, W
   movwf    Frame.Head + 1
   movwf    Frame.Tail + 1

   ; Reset the event mask, since we're starting from scratch.
   clrf     Modbus.Event
   return



;; ----------------------------------------------
;;  void Frame.rxByte( WREG value )
;;
;;  Store the byte at the next message buffer location.  A tail pointer keeps
;;  track of where it goes, so we fetch it first, then update its value with
;;  the new tail location when we're finished.
;;
Frame.rxByte:
   ;  TODO: bounds checking
   CopyWord Frame.Head, FSR1L
   movwf    POSTINC1
   CopyWord FSR1L, Frame.Head
   return



;; ----------------------------------------------
;;  WREG, Carry Frame.txByte()
;;
Frame.txByte:
   ; Make sure we're not trying to read past the end of the buffer.
   movf     Frame.Tail, W
   cpfseq   Frame.Head
     bra    getByte

   movf     Frame.Tail + 1, W
   cpfseq   Frame.Head + 1
     bra    getByte

   ; The head and tail pointers are equal, so we must be done.  Set the carry to
   ; indicate that no byte was read.
   bsf      STATUS, C
   return

getByte:
   ; Not past the end yet, so read the next byte (indirectly).
   CopyWord Frame.Tail, FSR1L
   movf     POSTINC1, W
   CopyWord FSR1L, Frame.Tail

   ; Clear the status flag to indicate the value in W is valid.
   bcf      STATUS, C
   return



   end
