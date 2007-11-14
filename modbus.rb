#!/usr/bin/ruby
## ---------------------------------------------------------------------------
##
##  Modbus
##
##  Copyright © 2006,7  Peter Heinrich
##  All Rights Reserved
##
##  $URL$
##  $Revision$
##
##  This file defines a tiny Modbus master (client) suitable for testing and
##  debugging of the PIC Modbus slave (server) library.  This master is de-
##  signed for interactive use in irb, exposing specific Modbus send/receive
##  actions as public methods.
##
## ---------------------------------------------------------------------------
##  $Author$
##  $Date$
## ---------------------------------------------------------------------------



require "serialport"



DEF_BAUD = 19200
DEF_DATABITS = 8
DEF_STOPBITS = 1
DEF_PARITY = SerialPort::EVEN



class Integer
  def le_word
    return (0xff & self).chr + (self >> 8).chr
  end
end



class Modbus
  def initialize( port, baud = DEF_BAUD, databits = DEF_DATABITS, stopbits = DEF_STOPBITS, parity = DEF_PARITY )
    begin
      @sp = SerialPort.open( port, baud, databits, stopbits, parity );
    rescue StandardError => bang
      puts "Couldn't initialize serial port (#{bang})."
    end

    rtu!
  end

  def ascii!
    @rtu = false
  end

  def rtu!
    @rtu = true
  end

  def is_rtu?
    @rtu
  end

  def crc( pdu )
    sum = 0xffff
    pdu.each_byte do |b|
      sum ^= b
      8.times do
        carry = (1 == 1 & sum)
        sum = 0x7fff & (sum >> 1)
        sum ^= 0xa001 if carry
      end
    end
    sum
  end

  def lrc( pdu )
    sum = 0
    pdu.each_byte { |b| sum += b }
    0xff & ~sum
  end

  def tx( slave, pdu )
    if is_rtu?
      adu  = slave.chr
      adu += pdu
      sum  = crc( adu )
      adu += (0xff & sum).chr + (sum >> 8).chr
    else
      adu  = ":%02x" % slave
      pdu.each_byte { |b| adu += "%02x" % b }
      adu += "%02x\r\n" % lrc( adu[ 1..-1 ] )
    end

    @sp.puts( adu ) if @sp
    puts( "Tx: \"#{adu}\"" )
  end

  def rx
    adu = @sp.gets if @sp
    puts( "Rx: \"#{adu}\"" )

    if is_rtu?
      slave = adu[ 0 ]
      pdu = adu[ 1..-3 ]

      sum = crc( adu[ 0..-3 ] )
      if sum != adu[ -2 ] + (adu[ -1 ] << 8)
        puts( "CRC incorrect! (Calculated 0x%04x)" % sum )
      end
    else
      slave = adu[1..2].hex
      pdu = ""
   
      sum = lrc( adu[ 1..-5 ] )
      if sum != adu[ -4..-3 ].hex
        puts( "LRC incorrect! (Calculated 0x%02x)" % sum )
      end

      adu = adu[ 3..-5 ]
      (0...adu.length).step( 2 ) { |i| pdu << adu[ i..i+1 ].hex.chr }
    end

    return slave, pdu
  end



  def readCoils( slave, address, count )
    tx( slave, 1.chr + address.le_word + count.le_word )
  end

  def readDiscretes( slave, address, count )
    tx( slave, 2.chr + address.le_word + count.le_word )
  end

  def readRegisters( slave, address, count )
    tx( slave, 3.chr + address.le_word + count.le_word )
  end

  def readInputs( slave, address, count )
    tx( slave, 4.chr + address.le_word + count.le_word )
  end

  def writeCoil( slave, address, value )
    tx( slave, 5.chr + address.le_word + value.le_word )
  end

  def writeRegister( slave, address, value )
    tx( slave, 6.chr + address.le_word + value.le_word )
  end

  def getExceptions( slave )
    tx( slave, 7.chr )
  end

  def getEventCount( slave )
    tx( slave, 11.chr )
  end

  def getEventLog( slave )
    tx( slave, 12.chr )
  end

  def writeCoils( slave, address, values )
  end

  def writeRegisters( slave, address, values )
    tx( slave, 16.chr + address.le_word + values.length.le_word +
         (values << 1).chr + values.pack( "v#{values.length}" )
  end

  def getSlaveId( slave )
    tx( slave, 17.chr )
  end

  def readFileRecord( slave, subreqs )
  end

  def writeFileRecord( slave, subreqs )
  end

  def writeRegMask( slave, address, andMask, orMask )
    tx( slave, 22.chr + address.le_word + andMask.le_word + orMask.le_word )
  end

  def readWriteRegs( slave, readAddr, count, writeAddr, values )
  end

  def readFIFOQueue( slave, queue )
    tx( slave, 24.chr + queue.le_word )
  end
end
