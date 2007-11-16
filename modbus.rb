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
    return 1, "\025\015\006\000\004\000\007\000\003\006\257\004\276\020\015" if !@sp

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
        print "Broadcast/ " if 0 != (0x40 & e)
        print "Listen-only/ " if 0 != (0x20 & e)
        print "Overrun/ " if 0 != (0x10 & e)
        print "Error/" if 0 != (0x02 & e)
        puts
      else
        print "%2d Message Sent: " % i
        print "Listen-only/ " if 0 != (0x20 & e)
        print "Timeout/ " if 0 != (0x10 & e)
        print "NAK err/ " if 0 != (0x08 & e)
        print "Busy err/ " if 0 != (0x04 & e)
        print "Abort err/ " if 0 != (0x02 & e)
        print "Read err/" if 0 != (0x01 & e)
        puts
      end
    end
  end



  def diagClear( slave )
    tx( slave, 8.chr + 10.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [diagClear]"
    end
  end

  def diagClearOverrun( slave )
    tx( slave, 8.chr + 20.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [diagClearOverrun]"
    end
  end

  def diagGetBusyCount( slave )
    tx( slave, 8.chr + 17.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      busy = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

      puts "Slave #{slave} [diagGetBusyCount]:"
      puts "  busy: #{busy}"
      busy
    end
  end

  def diagGetErrorCount( slave )
    tx( slave, 8.chr + 12.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      errors = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

      puts "Slave #{slave} [diagGetErrorCount]:"
      puts "  bus errors: #{errors}"
      errors
    end
  end

  def diagGetExceptCount( slave )
    tx( slave, 8.chr + 13.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      errors = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

      puts "Slave #{slave} [diagGetExceptCount]"
      puts "  exceptions: #{errors}"
      errors
    end
  end

  def diagGetMsgCount( slave )
    tx( slave, 8.chr + 11.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      messages = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

      puts "Slave #{slave} [diagGetMsgCount]"
      puts "  messages: #{messages}"
      messages
    end
  end

  def diagGetNAKCount( slave )
    tx( slave, 8.chr + 16.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      naks = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

      puts "Slave #{slave} [diagGetNAKCount]"
      puts "  NAKs: #{naks}"
      naks
    end
  end

  def diagGetNoRespCount( slave )
    tx( slave, 8.chr + 15.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      noResp = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

      puts "Slave #{slave} [diagGetNoRespCount]"
      puts "  no response: #{noResp}"
      noResp
    end
  end

  def diagGetOverrunCount( slave )
    tx( slave, 8.chr + 18.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      overruns = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

      puts "Slave #{slave} [diagGetOverrunCount]"
      puts "  overruns: #{overruns}"
      overruns
    end
  end

  def diagGetRegister( slave )
    tx( slave, 8.chr + 2.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      register = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

      puts "Slave #{slave} [diagGetRegister]"
      puts "  register: #{register}"
    end
  end

  def diagGetSlaveMsgCount( slave )
    tx( slave, 8.chr + 14.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      messages = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

      puts "Slave #{slave} [diagGetSlaveMsgCount]"
      puts "  messages: #{messages}"
    end
  end

  def diagRestartComm( slave, clearLog = false )
    tx( slave, 8.chr + 1.to_word + (clearLog ? 0 : 0xff00).to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [diagRestartComm]:"
      puts "  log: #{clearLog ? "cleared" : "preserved"}"
    end
  end

  def diagReturnQuery( slave, data )
    tx( slave, 8.chr + 0.to_word + data.pack( "c*" ) )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [diagReturnQuery]"
      pdu[ 3..-1 ].unpack( "c*" )
    end
  end

  def diagSetDelim( slave, delim )
    tx( slave, 8.chr + 3.to_word + delim + 0.chr )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [diagSetDelim]:"
      puts "  delimiter: \"" + delim + "\""
    end
  end

  def diagSetListenOnly( slave )
    tx( slave, 8.chr + 4.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [diagSetListenOnly]"
    end
  end

  def encapCANopen( slave )
    puts "Not Yet Implemented"
  end

  def encapGetDeviceId( slave )
    puts "Not Yet Implemented"
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
      status, events, messages = pdu[2, 6].unpack( "n3" )
      log = pdu[8..-1].unpack( "c*" )

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
      queue = pdu[ 5..-1 ].unpack( "n*" )

      puts "Slave #{slave} [readFIFOQueue]"
      queue
    end
  end

  # [[file1, start1, count1], [file2, start2, count2], ... ]
  def readFileRecord( slave, subreqs )
    pdu = ""
    subreqs.each { |sr| pdu << 6.chr << sr.pack( "nnn" ) }

    tx( slave, 20.chr + pdu.length.chr + pdu )
    slave, pdu = rx()
 
    unless is_error?( pdu )
      records = []
      offset = 2

      while offset < pdu.length - 3 do
        records << pdu[ 2 + offset, pdu[ offset ] - 1 ].unpack( "n*" )
        offset += pdu[ offset ] + 1
      end
 
      puts "Slave #{slave} [readFileRecord]:"
      puts "  records: #{records.length}"
      puts "  total:   #{pdu[ 1 ]} bytes"
      records
    end
  end

  def readInputs( slave, address, count )
    tx( slave, 4.chr + (address - 1).to_word + count.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [readInputs]"
      inputs = pdu[ 2..-1 ].unpack( "n*" )
    end
  end

  def readRegisters( slave, address, count )
    tx( slave, 3.chr + (address - 1).to_word + count.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [readRegisters]"
      registers = pdu[ 2..-1 ].unpack( "n*" )
    end
  end

  def readWriteRegs( slave, readAddr, count, writeAddr, values )
    length = values.length

    tx( slave, 23.chr + readAddr.to_word + count.to_word +
        writeAddr.to_word + length.to_word + (length << 1).chr + values.pack( "n*" ) )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [readWriteRegs]"
      inputs = pdu[ 2..-1 ].unpack( "n*" )
    end
  end

  def writeCoil( slave, address, value )
    tx( slave, 5.chr + (address - 1).to_word + (0 == value ? 0 : 0xff00).to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      value = pdu[ 3, 2 ].unpack( "n" )
 
      puts "Slave #{slave} [writeCoil]:"
      puts "  coil #{address}: #{0 == value[ 0 ] ? "RESET" : "SET"}"
      value
    end
  end

  def writeCoils( slave, address, values )
    count = values.length
    pdu = ""
    0.step( count, 8 ) { |i| pdu << values[ i...i+8 ].join.reverse.to_i( 2 ).chr }

    tx( slave, 15.chr + (address - 1).to_word + count.to_word + pdu.length.chr + pdu )
    slave, pdu = rx()

    unless is_error?( pdu )
      count = pdu[ 3, 2 ].unpack( "n" )
 
      puts "Slave #{slave} [writeCoils]:"
      puts "  #{count} coil(s) written"
    end
  end

  # [[file1, start1, [data1]], [file2, start2, [data2]], ... ]
  def writeFileRecord( slave, subreqs )
    pdu = ""
    subreqs.each { |sr| pdu << 6.chr << sr.pack( "nn" ) << sr[ 2 ].pack( "n*" ) }

    tx( slave, 21.chr + pdu.length.chr + pdu )
    slave, pdu = rx()
 
    unless is_error?( pdu )
      puts "Slave #{slave} [writeFileRecord]:"
      puts "  records: #{subreqs.length}"
      puts "  total:   #{pdu[ 1 ]} bytes"
    end
  end

  def writeRegister( slave, address, value )
    tx( slave, 6.chr + (address - 1).to_word + value.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      value = pdu[ 3, 2 ].unpack( "n" )

      puts "Slave #{slave} [writeRegister]:"
      puts "  register #{address}: 0x%04x" % value
      value
    end
  end

  def writeRegisters( slave, address, values )
    length = values.length

    tx( slave, 16.chr + (address - 1).to_word + length.to_word + (length << 1).chr + values.pack( "n*" ) )
    slave, pdu = rx()

    unless is_error?( pdu )
      count = pdu[ 3, 2 ].unpack( "n" )
 
      puts "Slave #{slave} [writeRegisters]:"
      puts "  #{count} register(s) written"
    end
  end

  def writeRegMask( slave, address, andMask, orMask )
    tx( slave, 22.chr + (address - 1).to_word + andMask.to_word + orMask.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      puts "Slave #{slave} [writeRegMask]"
    end
  end
end
