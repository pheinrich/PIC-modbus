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
;;  This file provides a test harness for the MODBUS routines implemented by
;;  the other modules of this project.
;;
;; ---------------------------------------------------------------------------
;;  $Author$
;;  $Date$
;; ---------------------------------------------------------------------------



   config   OSC=HS, OSCS=OFF, PWRT=OFF, BOR=OFF, WDT=OFF, CCP2MUX=ON, STVR=ON
   config   LVP=OFF, DEBUG=OFF
   config   CP0=OFF, CP1=OFF, CPB=OFF, CPD=OFF
   config   WRT0=OFF, WRT1=OFF, WRTB=OFF, WRTD=OFF
   config   EBTR0=OFF, EBTR1=OFF, EBTRB=OFF

   #include "modbus.inc"

   extern   ISR.high
   extern   ISR.low
   extern   main
   extern   CONF.init
   extern   RTU.init
   extern   UART.init

   global   MODBUS.resetMsgBuffer



;; ---------------------------------------------------------------------------
.reset      code     0x0000
;; ---------------------------------------------------------------------------

   ; Disable all interrupts and jump to relocatable initialization code.
   clrf     INTCON
   goto     MODBUS.init



;; ---------------------------------------------------------------------------
.irqHigh    code     0x0008
;; ---------------------------------------------------------------------------

   ; Jump to high-priority interrupt service routine in relocatable code block.
   goto     ISR.high



;; ---------------------------------------------------------------------------
.irqLow     code     0x0018
;; ---------------------------------------------------------------------------

   ; Jump to low-priority interrupt service routine in relocatable code block.
   goto     ISR.low



;; ---------------------------------------------------------------------------
            udata_acs
;; ---------------------------------------------------------------------------

MODBUS.State      res   1     ; current state of the state machine
MODBUS.FrameError res   1     ; 0 = false, 255 = true
MODBUS.MsgBuffer  res   2     ; points to next location to be read or written



;; ---------------------------------------------------------------------------
.modbus     code
;; ---------------------------------------------------------------------------

;; ----------------------------------------------
;;  void MODBUS.init()
;;
;;  Initializes the device before falling through to the main event loop.
;;  This routine calls other methods to handle specific peripherals.
;;
MODBUS.init:
   ; Some components of the system must be initialized.
   call     CONF.init         ; read configuration jumpers/switches
   call     UART.init         ; set UART mode, baud rate, etc.
   call     RTU.init          ; calculate intercharacter/frame timeouts

   ; Clear all pending peripheral interrupts.
   clrf     PIR1
   clrf     PIR2

   ; Re-enable interrupts.
   bsf      INTCON, PEIE      ; enable all peripheral interrupts
   bsf      INTCON, GIE       ; enable all unmasked interrupts

   ; Enter main loop
   goto     main



;; ----------------------------------------------
;;  woid MODBUS.resetMsgBuffer
;;
MODBUS.resetMsgBuffer:
   ; Reset the pointer to the beginning of the buffer.
   movlw    LOW kMsgBuffer
   movwf    MODBUS.MsgBuffer
   movlw    HIGH kMsgBuffer
   movwf    MODBUS.MsgBuffer

   ; Reset the frame error indicator, since we're starting from scratch.
   clrf     MODBUS.FrameError
   return



;; ----------------------------------------------
;;  woid MODBUS.writeMsgByte
;;
MODBUS.writeMsgByte:
   ; Read the address of the last byte written.
   movf     MODBUS.MsgBuffer, W
   movwf    FSR0L
   movf     MODBUS.MsgBuffer, W
   movwf    FSR0H

   ; Write the byte indirectly.
   movf     RCREG, W          ; read the serial port latch
   movwf    POSTINC0          ; store and post increment

   ; Save the current tail pointer so we can resume there next time.
   movf     FSR0L, W
   movwf    MODBUS.MsgBuffer
   movf     FSR0H, W
   movwf    MODBUS.MsgBuffer + 1

   return



   end
