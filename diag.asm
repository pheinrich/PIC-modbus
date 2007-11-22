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
   global   Diag.ExceptStatus
   global   Diag.Options

   ; Public Methods
   global   Diag.diagnostics
   global   Diag.getEventCount
   global   Diag.getEventLog
   global   Diag.init
   global   Diag.logListenOnly
   global   Diag.logRestart
   global   Diag.logRxEvt
   global   Diag.logTxEvt
   global   Diag.noResponse

   ; Dependencies
   extern   ASCII.Delimiter
   extern   Frame.begin
   extern   Frame.beginEcho
   extern   Frame.end
   extern   Modbus.Event
   extern   Modbus.unsupported
   extern   Util.Frame
   extern   Util.Save
   extern   VTable.dispatch



;; ---------------------------------------------------------------------------
                        udata_acs
;; ---------------------------------------------------------------------------

Diag.ExceptStatus       res   1
Diag.Options            res   1
                        ; 1-------  listen-only mode
                        ; -1------  busy
                        ; --1-----  don't count event
                        ; ---XXXXX  reserved

LogHead                 res   1     ; pointer to next write position
LogTail                 res   1     ; pointer to next read position

NumEvents               res   2
NumCommErrs             res   2
NumExceptErrs           res   2
NumMsgs                 res   2
NumNoResponse           res   2
NumOverruns             res   2
NumSlaveBusy            res   2
NumSlaveMsgs            res   2
NumSlaveNAKs            res   2
Register                res   2



;; ---------------------------------------------------------------------------
.diag                   code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  Diagnostics Virtual Function Table
;;
;;  This table associates Modbus diagnostic subfunction codes with specific
;;  handler methods.  See VTable.dispatch() for more information.
;;
DiagnosticsVTbl:
   data     Modbus.kDiagReturnQuery, returnQuery
   data     Modbus.kDiagRestartComm, restartComm
   data     Modbus.kDiagGetRegister, getRegister
   data     Modbus.kDiagSetDelim, setDelim
   data     Modbus.kDiagSetListenOnly, setListenOnly
   data     Modbus.kDiagClear, clear
   data     Modbus.kDiagGetMsgCount, getMsgCount
   data     Modbus.kDiagGetErrorCount, getErrorCount
   data     Modbus.kDiagGetExceptCount, getExceptCount
   data     Modbus.kDiagGetSlaveMsgCount, getSlaveMsgCount
   data     Modbus.kDiagGetNoRespCount, getNoRespCount
   data     Modbus.kDiagGetNAKCount, getNAKCount
   data     Modbus.kDiagGetBusyCount, getBusyCount
   data     Modbus.kDiagGetOverrunCount, getOverrunCount
   data     Modbus.kDiagClearOverrun, clearOverrun
   data     -1, Modbus.unsupported



;; ----------------------------------------------
;;  void Diag.diagnostics()
;;
;;  Handles the diagnostic family of Modbus functions.  Since these are all
;;  attached to subfunction codes, this method extracts the subfunction id from
;;  the received message, then dispatches through the vtable above.
;;
Diag.diagnostics:
   ; Pull the 16-bit subfunction identifier from the request.
   movff    Modbus.kRxSubFunction + 1, Util.Frame
   movff    Modbus.kRxSubFunction, Util.Frame + 1

   ; Use the id to perform a virtual function call.
   SetTableBase DiagnosticsVTbl
   goto     VTable.dispatch



;; ----------------------------------------------
;;  void Diag.getEventCount()
;;
;;  Returns a frame describing the number of bus "events" detected by this de-
;;  vice.  An event is simply a message received or transmitted, whether it's
;;  addressed to us or not.
;;
Diag.getEventCount:
   call     Frame.begin             ; start a new frame
   bsf      Diag.Options, Modbus.kDiag_DontCountEvent

   rcall    logDetails              ; include device status and event count
   goto     Frame.end               ; end the frame



;; ----------------------------------------------
;;  void Diag.getEventLog()
;;
;;  Returns a frame containing log entries for up to 64 of the most recent
;;  events recorded by this device.  Each entry is single byte whose format
;;  varies according to event type, error conditions, and our own state.  For
;;  more information, see Diag.logRxEvt(), Diag.logTxEvt(), Diag.logRestart(),
;;  or Diag.logListenOnly().
;;
Diag.getEventLog:
   ; Start a new frame and initialize it.
   call     Frame.begin
   bsf      Diag.Options, Modbus.kDiag_DontCountEvent

   ; Add some header data.
   movlw    6
   movwf    POSTINC0                ; save space for the byte count
   rcall    logDetails              ; include device status and event count

   movff    NumSlaveMsgs + 1, POSTINC0
   movff    NumSlaveMsgs, POSTINC0

   ; Calculate how many events are in the circular event log.
   movf     LogTail, W
   subwf    LogHead, W              ; W = head - tail
   btfsc    STATUS, N               ; is tail > head?
     addlw  Modbus.kLogBufLen       ; yes, adjust for wrap-around

   SetBank Modbus.kTxByteCount
   addwf    Modbus.kTxByteCount, F  ; update byte count in frame
   movwf    Util.Save               ; store as counter variable
   incf     Util.Save

   ; Add the actual event log entries to the frame.
   lfsr     FSR1, Modbus.kLogBuffer ; set base pointer
   movf     LogHead, W              ; offset index to next empty entry
   bra      chkLoop                 ; skip to loop termination check

logLoop:
   decf     WREG                    ; predecrement the index
   btfsc    STATUS, N               ; has index moved past start of log buffer?
     addlw  Modbus.kLogBufLen       ; yes, reposition to end
   movff    PLUSW1, POSTINC0        ; copy from the log to the frame

chkLoop:
   decfsz   Util.Save               ; have we transferred all the entries?
     bra    logLoop                 ; no, loop until done

   ; End the frame.
   goto     Frame.end



;; ----------------------------------------------
;;  void Diag.init()
;;
;;  Initializes this device's diagnostics, which means clearing all counters
;;  and the event log.
;;
Diag.init:
   ; Point to our block of local variables, with total byte length in W.
   lfsr     FSR0, Diag.ExceptStatus
   movlw    Register - Diag.ExceptStatus + 2

   ; Clear the block.
   clrf     POSTINC0
   decfsz   WREG, F
     bra    $-4

   return



;; ----------------------------------------------
;;  void Diag.logListenOnly()
;;
;;  Adds a single event log entry describing a request to enter listen-only
;;  mode.
;;
Diag.logListenOnly:
   movlw    Modbus.kCmdEvt_ListenOnly
   bra      storeLogByte



;; ----------------------------------------------
;;  void Diag.logRestart()
;;
;;  Adds a single event log entry describing a request to restart the commun-
;;  ication system.
;;
Diag.logRestart:
   movlw    Modbus.kCmdEvt_Restart
   bra      storeLogByte



;; ----------------------------------------------
;;  void Diag.logRxEvt()
;;
;;  Adds an entry to the event log corresponding to a receive event.  The
;;  entry is an 8-bit bitfield:
;;
;;    1-------    ; always 1
;;    -1------    ; broadcast received (address = 0)
;;    --1-----    ; currently in listen-only mode
;;    ---1----    ; character overrun (buffer overflow)
;;    ----XX--    ; not used
;;    ------1-    ; communication error (bad checksum)
;;    -------X    ; not used
;;
Diag.logRxEvt:
   ; Count every message we see on the bus, even if it's not addressed to us.
   IncrementWord NumMsgs

   ; Count checksum failures.
   btfss    Modbus.Event, Modbus.kRxEvt_CommErr
     bra    rxSlave
   IncrementWord NumCommErrs

rxSlave:
   ; Count messages this device has processed, which means all broadcast messages
   ; and messages addressed to it specifically.
   btfss    Modbus.Event, Modbus.kRxEvt_SlaveMsg
     bra    rxOverrun
   IncrementWord NumSlaveMsgs
   bcf      Modbus.Event, Modbus.kRxEvt_SlaveMsg

rxOverrun:
   ; Count buffer overruns.  This will also include framing errors.
   btfss    Modbus.Event, Modbus.kRxEvt_Overrun
     bra    rxWrite
   IncrementWord NumOverruns

rxWrite:
   ; Set the event type and copy the state of the listen-only mode indicator bit.
   bsf      Modbus.Event, 7
   btfsc    Diag.Options, Modbus.kDiag_ListenOnly
     bsf    Modbus.Event, Modbus.kRxEvt_ListenOnly

   ; Store the event byte in the log.
   bra      storeLogByte



;; ----------------------------------------------
;;  void Diag.logTxEvt()
;;
;;  Adds an entry to the event log corresponding to a transmit event.  The
;;  entry is an 8-bit bitfield:
;;
;;    01------    ; always 01
;;    --1-----    ; currently in listen-only mode
;;    ---1----    ; a write timeout occurred
;;    ----1---    ; slave program NAK exception sent (exception code 7)
;;    -----1--    ; slave busy exception sent (exception codes 5-6)
;;    ------1-    ; slave abort exception sent (exception code 4)
;;    -------1    ; read exception sent (exception codes 1-3)
;;
Diag.logTxEvt:
   ; Count all exception responses transmitted by this device.
   movlw    (1 << Modbus.kTxEvt_ReadEx) | (1 << Modbus.kTxEvt_AbortEx) | (1 << Modbus.kTxEvt_BusyEx) | (1 << Modbus.kTxEvt_NAKEx)
   andwf    Modbus.Event, W
   bz       txBusy
   IncrementWord NumExceptErrs

txBusy:
   ; Count the messages for which this device returned a "slave busy" exception.
   btfss    Modbus.Event, Modbus.kTxEvt_BusyEx
     bra    txNAK
   IncrementWord NumSlaveBusy

txNAK:
   ; Count the messages for which this device returned a negative acknowledgement
   ; exception.  This seems pathological, since this exception (7) isn't even
   ; documented in the Modbus Application Protocol V1.1a document and it's unclear
   ; how or when a NAK is generated.
   btfss    Modbus.Event, Modbus.kTxEvt_NAKEx
     bra    txWrite
   IncrementWord NumSlaveNAKs

txWrite:
   ; Set the event type and copy the state of the listen-only mode indicator bit.
   bcf      Modbus.Event, 7
   bsf      Modbus.Event, 6
   btfsc    Diag.Options, Modbus.kDiag_ListenOnly
     bsf    Modbus.Event, Modbus.kTxEvt_ListenOnly

   ; Store the event byte in the log.
   bra      storeLogByte



;; ----------------------------------------------
;;  void Diag.noResponse()
;;
;;  Increments the counter tracking messages recognized but not responded to.
;;  These generally correspond to any message we receive while in listen-only
;;  mode.
;;
Diag.noResponse:
   IncrementWord NumNoResponse
   return



;; ----------------------------------------------
;;  FSR0 begin()
;;
;;  Initializes the transmission frame in preparation for a diagnostic message
;;  with function and subfunction codes.
;;
begin:
   call     Frame.begin
   movff    Modbus.kRxSubFunction, POSTINC0
   movff    Modbus.kRxSubFunction + 1, POSTINC0
   return



;; ----------------------------------------------
;;
;;
clear:
   return



;; ----------------------------------------------
;;  void clearOverrun()
;;
;;  Clears the overrun counter.
;;
clearOverrun:
   call     Frame.beginEcho
   clrf     NumOverruns
   clrf     NumOverruns + 1
   goto     Frame.end



;; ----------------------------------------------
;;  void getBusyCount()
;;
;;  Returns the number messages to which this device replied with a Slave
;;  Device Busy exception response.
;;
getBusyCount:
   rcall    begin
   movff    NumSlaveBusy + 1, POSTINC0
   movff    NumSlaveBusy, POSTINC0
   goto     Frame.end   



;; ----------------------------------------------
;;  void getErrorCount()
;;
;;  Returns the number of message checksum errors encountered by this device.
;;
getErrorCount:
   rcall    begin
   movff    NumCommErrs + 1, POSTINC0
   movff    NumCommErrs, POSTINC0
   goto     Frame.end   



;; ----------------------------------------------
;;  void getExceptCount()
;;
;;  Returns the number of exception responses returned by this device.
;;
getExceptCount:
   rcall    begin
   movff    NumExceptErrs + 1, POSTINC0
   movff    NumExceptErrs, POSTINC0
   goto     Frame.end   



;; ----------------------------------------------
;;  void getMsgCount()
;;
;;  Returns the number of messages this device has detected, whether addressed
;;  to it or not (including broadcast messages).
;;
getMsgCount:
   rcall    begin
   movff    NumMsgs + 1, POSTINC0
   movff    NumMsgs, POSTINC0
   goto     Frame.end   



;; ----------------------------------------------
;;  void getNAKCount()
;;
;;  Returns the number of messages to which this device has replied with a
;;  negative acknowledgement.
;;
getNAKCount:
   rcall    begin
   movff    NumSlaveNAKs + 1, POSTINC0
   movff    NumSlaveNAKs, POSTINC0
   goto     Frame.end   



;; ----------------------------------------------
;;  void getNoRespCount()
;;
;;  Returns the number of messages to which this device did not reply.  This
;;  includes all messages received in listen-only mode.
;;
getNoRespCount:
   rcall    begin
   movff    NumNoResponse + 1, POSTINC0
   movff    NumNoResponse, POSTINC0
   goto     Frame.end   



;; ----------------------------------------------
;;  void getOverrunCount()
;;
;;  Returns the number of unhandled messages due to character overruns, which
;;  include any situation in which characters arrive faster than they can be
;;  processed.
;;
getOverrunCount:
   rcall    begin
   movff    NumOverruns + 1, POSTINC0
   movff    NumOverruns, POSTINC0
   goto     Frame.end   



;; ----------------------------------------------
;;  void getRegister()
;;
;;  Returns the diagnostic register.  The Modbus documentation is pretty light
;;  on this subject, neglecting to define the diagnostic register or describe
;;  its use.  That implies it may be application-specific.
;;
getRegister:
   rcall    begin
   movff    Register + 1, POSTINC0
   movff    Register, POSTINC0
   goto     Frame.end   



;; ----------------------------------------------
;;  void getSlaveMsgCount()
;;
;;  Returns the number of messages directed to this device, either addressed
;;  to it specifically or the broadcast address.
;;
getSlaveMsgCount:
   rcall    begin
   movff    NumSlaveMsgs + 1, POSTINC0
   movff    NumSlaveMsgs, POSTINC0
   goto     Frame.end   



;; ----------------------------------------------
;;  FSR0 logDetails( FSR0 nextByte )
;;
;;  Adds some basic event log data to the current response frame.  This data
;;  is returned for both the Modbus.kGetEventCount and Modbus.kGetEventLog
;;  commands, so we share the common code.
;;
logDetails:
   ; Calculate the current status.
   movlw    0x00                    ; assume we're idle (0x0000)
   btfsc    Diag.Options, Modbus.kDiag_Busy ; are we actually busy?
     movlw  0xff                    ; yes, return 0xffff

   movwf    POSTINC0                ; add the status to the frame
   movwf    POSTINC0

   ; Add the event count to the frame, too.
   movff    NumEvents + 1, POSTINC0
   movff    NumEvents, POSTINC0
   return



;; ----------------------------------------------
;;
;;
restartComm:
   return



;; ----------------------------------------------
;;  void returnQuery()
;;
;;  Simply echos the request data back to the client.  The data itself is
;;  totally client-specific; it isn't inspected or processed in any way, so it
;;  may contain anything.
;;
returnQuery:
   call     Frame.beginEcho
   goto     Frame.end



;; ----------------------------------------------
;;  void setDelim()
;;
;;  Changes the line termination character from the default LF (linefeed).
;;  This is strictly an ASCII-mode operation, so attempting to execute it
;;  while RTU mode is active will result in an Illegal Function exception
;;  response.
;;
setDelim:
   ; Check to make sure we're in ASCII mode.
   btfsc    TXSTA, TX9              ; USART in 7-bit mode?
     goto Modbus.unsupported        ; no, so this method isn't supported

   ; Go ahead and change the delimiter character.
   call     Frame.beginEcho
   movff    Modbus.kRxDelimiter, ASCII.Delimiter
   goto     Frame.end   



;; ----------------------------------------------
;;  void setListenOnly()
;;
;;  Forces this device into "listen-only" mode, in which all messages are
;;  monitored and logged, but no actions are taken.  A communications restart
;;  must be requested in order to exit this mode.
;;
setListenOnly:
   bsf      Diag.Options, Modbus.kDiag_ListenOnly
   bra      Diag.logListenOnly



;; ----------------------------------------------
;;  void storeLogByte()
;;
;;  Stores the current Modbus.Event byte in the circular log buffer.  The tail
;;  pointer is advanced, but never more than the maximum buffer length, 64
;;  (the head pointer may move to compensate).
;;
storeLogByte:
   ; Keep track of overall event count.
   IncrementWord NumEvents

   ; Store the event byte at the head of the buffer.
   lfsr     FSR0, Modbus.kLogBuffer
   movf     LogHead, W
   movff    Modbus.Event, PLUSW0

   ; Increment the head pointer, making sure it never exceeds the maximum buffer
   ; length of 65 (one more than we need, to avoid circular buffer difficulties).
   incf     LogHead, F              ; add 1 to the head pointer
   movlw    Modbus.kLogBufLen
   cpfslt   LogHead                 ; is the new head >= max buffer length?
     clrf   LogHead                 ; yes, reset to 0

   ; The tail pointer may need to be adjusted as well.
   movf     LogTail, W
   cpfseq   LogHead                 ; are the head and tail pointers equal?
     return                         ; no, we're done

   incf     LogTail, F              ; yes, add 1 to the tail pointer, too
   movlw    Modbus.kLogBufLen
   cpfslt   LogTail                 ; is the new tail >= max buffer length?
     clrf   LogTail                 ; yes, reset to 0

   return



   end
