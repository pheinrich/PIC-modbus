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
;;  This module should be provided by the application.
;;
;; ---------------------------------------------------------------------------
;;  $Author$
;;  $Date$
;; ---------------------------------------------------------------------------



   #include "modbus.inc"

   global   main



;; ---------------------------------------------------------------------------
.main       code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  void main()
;;
;;  Main execution loop.  This routine cycles endlessly, updating the state of
;;  the application and responding to external events.
;;
main:
   ; Loop here waiting for something to do.
   goto     main



   end
