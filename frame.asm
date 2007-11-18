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



   #include "private.inc"

   ; Global Variables
   global   Frame.Checksum
   global   Frame.Head
   global   Frame.Tail

   ; Public Methods
   global   Frame.begin
   global   Frame.end
   global   Frame.endWithError
   global   Frame.isValid
   global   Frame.reset
   global   Frame.rxByte
   global   Frame.txByte

   ; Dependencies
   extern   Modbus.Address
   extern   Modbus.Event
   extern   Modbus.NoChecksum



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
;;  Other code can then append additional data before calling Frame.end().
;;
Frame.begin:
   ; Save a pointer to the beginning of the message.
   lfsr     FSR0, Modbus.kTxBuffer
   rcall    Frame.reset

   ; Copy the device id from the original message, as well as the function code.
   movff    Modbus.kRxSlave, POSTINC0
   movff    Modbus.kRxFunction, POSTINC0
   return



;; ----------------------------------------------
;;  void Frame.end( FSR0 nextByte )
;;
Frame.end:
   CopyWord FSR0L, Frame.Head
   return



;; ----------------------------------------------
;;  void Frame.endWithError( WREG exception, FSR0 nextByte  )
;;
Frame.endWithError:
   lfsr     FSR0, Modbus.kTxErrorCode

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
   ; Advance the pointer one byte.
   movf     POSTINC0, W
   bra      Frame.end



;; ----------------------------------------------
;;  WREG Frame.isValid()
;;
Frame.isValid:
   ; Verify the message is addressed to this device.
   bsf      Modbus.Event, Modbus.kRxEvt_Broadcast ; assume broadcast message
   movlb    1
   movf     Modbus.kRxBuffer, W     ; is this a broadcast message (0 == address)?
   bz       valChecksum             ; yes, validate the checksum

   bcf      Modbus.Event, Modbus.kRxEvt_Broadcast ; no, clear our assumption
   cpfseq   Modbus.Address          ; is it addressed to this specific device?
     retlw  0xff                    ; no, discard frame

valChecksum:
   ; This message is addressed to us.
   bsf      Modbus.Event, Modbus.kRxEvt_SlaveMsg

   ; If checksum validation is turned off, we're done.
   tstfsz   Modbus.NoChecksum
     retlw  0

   ; Set a pointer to the last byte in the message buffer.
   CopyWord Frame.Head, FSR0L
   bsf      Modbus.Event, Modbus.kRxEvt_CommErr ; assume checksum fails

   ; Compare the checksum included in the message to the one we calculated.
   movf     POSTINC0, W             ; fetch the LSB
   cpfseq   Frame.Checksum          ; does it match the computed value?
     retlw  0xff                    ; no, discard the frame  TODO: log event

   movf     INDF0, W                ; yes, fetch the MSB
   cpfseq   Frame.Checksum + 1      ; does it match the computed value?
     retlw  0xff                    ; no, discard the frame  TODO: log event

   ; Success, so clear our earlier assumption about a bad checksum.
   bcf      Modbus.Event, Modbus.kRxEvt_CommErr
   retlw    0



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
;;  todo: bounds checking
;;
Frame.rxByte:
   ; Store the byte at the next message buffer location.  A tail pointer keeps
   ; track of where it goes, so we fetch it first, then update its value with the
   ; new tail location when we're finished.
   CopyWord Frame.Head, FSR1L
   movwf    POSTINC1
   CopyWord FSR1L, Frame.Head
   return



;; ----------------------------------------------
;;  WREG+C Frame.txByte()
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
