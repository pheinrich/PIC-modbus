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
  def to_word
    return (self >> 8).chr + (0xff & self).chr
  end
end



class Modbus
  @@errors = { 1  => "Illegal Function",
               2  => "Illegal Data Address",
               3  => "Illegal Data Value",
               4  => "Slave Device Failure",
               5  => "Acknowledge",
               6  => "Slave Device Busy",
               8  => "Memory Parity Error",
               10 => "Gateway Path Unavailable" }

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

  def is_error?( pdu )
    if 0 != (0x80 & pdu[ 0 ])
       puts "Error: #{@@errors[ pdu[ 1 ] ]}"
       return true
    end
    return false
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
    0xff & ~pdu.sum( 128 )
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
    # DEBUG
    return 1, "\027\014\000\376\012\315\000\001\000\003\000\015\377" if !@sp

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

  def showEvents( log )
    log.each_with_index do |e, i|
      if 0 == e
        puts "%2d Communication Restart" % i
      elsif 4 == e
        puts "%2d Entering Listen Only Mode" % i
      elsif 0 != (0x80 & e)
        print "%2d Message Received: " % i
        print "broadcast/ " if 0 != (0x40 & e)
        print "listen-only/ " if 0 != (0x20 & e)
        print "overrun/ " if 0 != (0x10 & e)
        print "error/" if 0 != (0x02 & e)
        puts
      else
        print "%2d Message Sent: " % i
        print "listen-only/ " if 0 != (0x20 & e)
        print "timeout/ " if 0 != (0x10 & e)
        print "NAK err/ " if 0 != (0x08 & e)
        print "Busy err/ " if 0 != (0x04 & e)
        print "Abort err/ " if 0 != (0x02 & e)
        print "Read err/" if 0 != (0x01 & e)
        puts
      end
    end
  end



  def getEventCount( slave )
    tx( slave, 11.chr )
    slave, pdu = rx()

    unless is_error?( pdu )
      status, events = pdu[1, 4].unpack( "n2" )

      puts "Slave #{slave} [getEventCount]:"
      puts "  status: #{0 == status ? "READY" : "BUSY"}"
      puts "  events:  #{events}"
    end
  end

  def getEventLog( slave )
    tx( slave, 12.chr )
    slave, pdu = rx()

    unless is_error?( pdu )
      bytes, status, events, messages = pdu[1, 7].unpack( "cn3" )
      log = pdu[8..-1].unpack( "c#{bytes - 6}" )

      puts "Slave #{slave} [getEventLog]:"
      puts "  status:   #{0 == status ? "READY" : "BUSY"}"
      puts "  events:   #{events}"
      puts "  messages: #{messages}"
      log
    end
  end

  def getExceptions( slave )
    tx( slave, 7.chr )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [getExceptions]:"
      8.times { |i| puts "  exception ##{i}: #{0 == (1 << i) & pdu[ 1 ] ? "NO" : "YES"}" }
    end
  end

  def getSlaveId( slave )
    tx( slave, 17.chr )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [getSlaveId]"
      pdu[ 2..-1 ]
    end
  end

  def readCoils( slave, address, count )
    tx( slave, 1.chr + (address - 1).to_word + count.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      coils = []
      pdu[ 2..-1 ].unpack( "b*" ).join.each_byte { |b| coils << b - ?0 }

      puts "Slave #{slave} [readCoils]"
      coils[ 0, count ]
    end
  end

  def readDiscretes( slave, address, count )
    tx( slave, 2.chr + (address - 1).to_word + count.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      discretes = []
      pdu[ 2..-1 ].unpack( "b*" ).join.each_byte { |b| discretes << b - ?0 }

      puts "Slave #{slave} [readDiscretes]"
      discretes[ 0, count ]
    end
  end

  def readFIFOQueue( slave, queue )
    tx( slave, 24.chr + queue.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      count = pdu[ 3, 2 ].unpack( "n" )
      queue = pdu[ 5..-1 ].unpack( "n#{count}" )

      puts "Slave #{slave} [readFIFOQueue]"
      queue
    end
  end

  def readFileRecord( slave, subreqs )
  end

  def readInputs( slave, address, count )
    tx( slave, 4.chr + (address - 1).to_word + count.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [readInputs]"
      inputs = pdu[ 2..-1 ].unpack( "n#{pdu[ 1 ] >> 1}" )
    end
  end

  def readRegisters( slave, address, count )
    tx( slave, 3.chr + (address - 1).to_word + count.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [readRegisters]"
      registers = pdu[ 2..-1 ].unpack( "n#{pdu[ 1 ] >> 1}" )
    end
  end

  def readWriteRegs( slave, readAddr, count, writeAddr, values )
  end

  def writeCoil( slave, address, value )
    tx( slave, 5.chr + address.to_word + value.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
    end
  end

  def writeCoils( slave, address, values )
    count = values.length
    pdu = ""
    0.step( count, 8 ) { |i| pdu << values[ i...i+8 ].join.reverse.to_i( 2 ).chr }

    tx( slave, 15.chr + (address - 1).to_word + count.to_word + pdu.length.chr + pdu )
    slave, pdu = rx()

    unless is_error?( pdu )
    end
  end

  def writeFileRecord( slave, subreqs )
  end

  def writeRegister( slave, address, value )
    tx( slave, 6.chr + (address - 1).to_word + value.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
    end
  end

  def writeRegisters( slave, address, values )
    length = values.length

    tx( slave, 16.chr + (address - 1).to_word + length.to_word + (length << 1).chr + values.pack( "v#{length}" ) )
    slave, pdu = rx()

    unless is_error?( pdu )
    end
  end

  def writeRegMask( slave, address, andMask, orMask )
    tx( slave, 22.chr + (address - 1).to_word + andMask.to_word + orMask.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
    end
  end
end
