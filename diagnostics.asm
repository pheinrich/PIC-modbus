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



   #include "private.inc"

   global   Diag.readExceptStatus
   global   Diag.returnQueryData
   global   Diag.restartCommOption
   global   Diag.returnDiagRegister
   global   Diag.changeASCIIDelimiter
   global   Diag.forceListenOnlyMode
   global   Diag.clearCountersDiagRegister
   global   Diag.returnBusMessageCount
   global   Diag.returnBusCommErrorCount
   global   Diag.returnSlaveExceptErrorCount
   global   Diag.returnSlaveMessageCount
   global   Diag.returnSlaveNoResponseCount
   global   Diag.returnSlaveNAKCount
   global   Diag.returnSlaveBusyCount
   global   Diag.returnBusOverrunCount
   global   Diag.getCommEventCounter
   global   Diag.getCommEventLog
   global   Diag.reportSlaveID
   global   Diag.readDeviceID



;; ---------------------------------------------------------------------------
            udata_acs
;; ---------------------------------------------------------------------------

Options        res   1
               ; 1------      ; return query data
               ; -1-----      ; listen-only mode
               ; --1----      ; busy
               ; ---XXXX      ; reserved

ExceptStatus   res   1
DiagRegister   res   2




;; ---------------------------------------------------------------------------
.diag       code
;; ---------------------------------------------------------------------------

Diag.changeASCIIDelimiter:
Diag.clearCountersDiagRegister:
Diag.forceListenOnlyMode:
Diag.getCommEventCounter:
Diag.getCommEventLog:
Diag.readDeviceID:
Diag.readExceptStatus:
Diag.reportSlaveID:
Diag.restartCommOption:
Diag.returnBusOverrunCount:
Diag.returnBusCommErrorCount:
Diag.returnBusMessageCount:
Diag.returnDiagRegister:
Diag.returnQueryData:
Diag.returnSlaveBusyCount:
Diag.returnSlaveExceptErrorCount:
Diag.returnSlaveMessageCount:
Diag.returnSlaveNAKCount:
Diag.returnSlaveNoResponseCount:
   return



   end
