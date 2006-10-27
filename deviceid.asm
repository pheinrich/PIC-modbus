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



   include "modbus.inc"

   global   DEVICEID.MajorMinorRevision
   global   DEVICEID.ModelName
   global   DEVICEID.ProductCode
   global   DEVICEID.ProductName
   global   DEVICEID.VendorName
   global   DEVICEID.VendorURL
   global   DEVICEID.UserApplicationName

   global   DEVICEID.getAddress
   global   DEVICEID.setAddress



;; ---------------------------------------------------------------------------
.eeprom     code     0xf00000
;; ---------------------------------------------------------------------------

EEPROM.Address    de    0x1



;; ---------------------------------------------------------------------------
.deviceid   code
;; ---------------------------------------------------------------------------

DEVICEID.MajorMinorRevision:
   data     "1.0\0"

DEVICEID.ModelName:
   data     "Ifos 1\0"

DEVICEID.ProductCode:
   data     "Ifos1\0"

DEVICEID.ProductName:
   data     "Ifos Starfield Controller\0"

DEVICEID.VendorName:
   data     "Peter Heinrich\0"

DEVICEID.VendorURL:
   data     "http://www.saphum.com/ifos1\0"

DEVICEID.UserApplicationName:
   data     "Ifos Kernel 1.0\0"



;; ----------------------------------------------
;;  byte DEVICEID.getAddress()
;;
;;  Reads the device address from EEPROM memory.  The address uniquely ident-
;;  ifies this physical device on the MODBUS bus.  It's stored in EEPROM to
;;  allow for simple updates via a user command.
;;
DEVICEID.getAddress:
   ; Set up to read EEPROM memory.
   movlw    EEPROM.Address
   movwf    EEADR             ; latch the read target address
   bcf      EECON1, EEPGD     ; EEPROM instead of Flash
   bcf      EECON1, CFGS      ; data memory instead of config/calibration registers

   ; Read the EEPROM location into W.
   bsf      EECON1, RD        ; initiate the EEPROM read
   movf     EEDATA, W         ; copy our value from the latch register
   return



;; ----------------------------------------------
;;  void DEVICEID.setAddress( byte value )
;;
;;  Writes the device address to EEPROM memory.  This value corresponds to the
;;  "address" field in every MODBUS message.  Valid values are in the range
;;  [1, 247], since 0 represents the broadcast address and [248, 255] are re-
;;  served.  This method does NOT validate the value specified, however.
;;
;;  Once this method completes successfully, this device *immediately* begins
;;  processing messages to the specified address *only* (besides broadcast).
;;
DEVICEID.setAddress:
   ; Set up to write EEPROM memory.
   movwf    EEDATA            ; latch the value we want to write
   movlw    EEPROM.Address
   movwf    EEADR             ; latch the write target address
   bcf      EECON1, EEPGD     ; EEPROM instead of Flash
   bcf      EECON1, CFGS      ; data memory instead of config/calibration registers

   ; Enable EEPROM writes and disable interrupts.
   bsf      EECON1, WREN
   bcf      INTCON, GIE

   ; Write the security sequence (ensures against spurious writes).
   movlw    0x55
   movwf    EECON2
   movlw    0xaa
   movwf    EECON2

   ; Write the data into the EEPROM location.
   bsf      EECON1, WR        ; initiate the EEPROM write
   btfsc    EECON1, WR        ; is the write complete?
     bra    $-2               ; no, keep polling until it is

   ; Cleanup after writing the value.
   bcf      PIR2, EEIF        ; clear the EEPROM interrupt flag
   bsf      INTCON, GIE       ; re-enable interrupts
   bcf      EECON1, WREN      ; disable EEPROM writes.
   return



   end
