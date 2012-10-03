#!/usr/bin/env ruby
# -*- coding: utf-8 -*-
## ---------------------------------------------------------------------------
##
##  Tiny Modbus Master
##  Copyright © 2006,2008  Peter Heinrich
##
##  This program is free software; you can redistribute it and/or
##  modify it under the terms of the GNU General Public License
##  as published by the Free Software Foundation; either version 2
##  of the License, or (at your option) any later version.
##
##  This program is distributed in the hope that it will be useful,
##  but WITHOUT ANY WARRANTY; without even the implied warranty of
##  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##  GNU General Public License for more details.
##
##  You should have received a copy of the GNU General Public License
##  along with this program; if not, write to the Free Software
##  Foundation, Inc., 51 Franklin Street, Boston, MA  02110-1301, USA.
##
## ---------------------------------------------------------------------------
##  This file defines a tiny Modbus master (client) suitable for testing and
##  debugging of the PIC Modbus slave (server) library.  This master is de-
##  signed for interactive use in irb, exposing specific Modbus send/receive
##  actions as public methods.
## ---------------------------------------------------------------------------



require 'rubygems'
require 'serialport'



DEF_BAUD = 19200
DEF_DATABITS = 8
DEF_STOPBITS = 1
DEF_PARITY = SerialPort::EVEN



# Initialize some constants from the modbus.inc include file
File.open( "#{File.dirname( __FILE__ )}/modbus.inc", "r" ).each_line do |line|
  match = /Modbus\.k([A-Za-z0-9]+)\s+equ\s+([0-9]+)/.match( line )
  if match
    key, value = match.captures
    Kernel.const_set( key, value.to_i )
  end
end



class Integer
  def to_word
    return (self >> 8).chr + (0xff & self).chr
  end

  def to_long
    return (self >> 16).to_word + (0xffff & self).to_word
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

  class MockPort
    def initialize
    end

    def calcParity( value )
      case $parity
        when SerialPort::MARK
          true
        when SerialPort::SPACE
          false
        when SerialPort::EVEN, SerialPort::ODD
          parity = false

          while 0 != value
            parity = !parity
            value &= value - 1
          end

          parity ^ (SerialPort::EVEN != $parity)
      end
    end

    def gets
      $adu = "\001\002\003\004\005\006\007\010\011\012\145\143" if $adu.nil?
      $adu
    end

    def puts( adu )
      adu.each_byte do |b|
        if SerialPort::NONE != $parity
          if calcParity( b )
            b |= 0x100 if 8 == $databits
            b |= 0x80 if 7 == $databits
          else
            b &= 0x0ff if 8 == $databits
            b &= 0x7f if 7 == $databits
          end
        end

        print "%02x " % b if $debug
      end
      print "\n" if $debug
    end
  end

  def initialize( port = -1, rtu = true,
  				  baud = DEF_BAUD, stopbits = DEF_STOPBITS, parity = DEF_PARITY,
  				  verbose = false, debug = false )
    $databits = rtu ? 8 : 7
    $parity = parity
    $verbose = verbose
    $debug = debug

    if -1 == port
      @sp = MockPort.new
    else
      begin
        @sp = SerialPort.open( port, baud, $databits, stopbits, $parity );
      rescue StandardError => bang
        puts "Couldn't initialize serial port (#{bang})."
      end
    end
  end

  def port
    @sp
  end

  def ascii!
    $databits = 7
  end

  def rtu!
    $databits = 8
  end

  def is_rtu?
    8 == $databits
  end

  def setVerbose( verbose )
    $verbose = verbose
  end

  def setDebug( debug )
    $debug = debug
  end

  def setParity( parity )
    $parity = parity
  end

  def is_error?( pdu )
    if pdu && 0 != (0x80 & pdu[ 0 ].to_i)
       puts "Error: #{@@errors[ pdu[ 1 ] ]}" if $verbose
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
    0xff & -pdu.sum( 128 )
  end

  def tx( slave, pdu )
    if is_rtu?
      adu  = slave.chr
      adu += pdu
      sum  = crc( adu )
      adu += (0xff & sum).chr + (sum >> 8).chr
    else
      adu  = ':'
      pdu = slave.chr + pdu
      pdu.each_byte { |b| adu += "%02x" % b }
      adu += "%02x\r\n" % lrc( adu[ 1..-1 ] )
    end

    if $debug
      puts "Sending \"#{adu}\""
      adu.each_byte { |b| print "%02x " % b }
      puts
    end

    @sp.puts( adu ) if @sp
  end

  def rx
    adu = @sp.gets if @sp
    puts "Receiving \"#{adu}\"" if $debug
 
    if is_rtu?
      slave = adu[ 0 ]
      pdu = adu[ 1..-3 ]

      sum = crc( adu[ 0..-3 ] )
      if sum != adu[ -2 ] + (adu[ -1 ] << 8)
        puts( "CRC incorrect! (Calculated 0x%04x, found 0x%04x)" % [sum, adu[ -2 ] + (adu[ -1 ] << 8)] )
      end
    else
      slave = adu[ 1..2 ].hex
      pdu = ""
 
      sum = lrc( adu[ 1..-5 ] )
      if sum != adu[ -4..-3 ].hex
        puts( "LRC incorrect! (Calculated 0x%02x, found 0x%02x)" % [sum, adu[ -4..-3 ].hex] )
      end
 
      adu = adu[ 3..-5]
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
        print "Checksum/" if 0 != (0x02 & e)
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
    tx( slave, Diagnostics.chr + DiagClear.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        puts "Slave #{slave} [diagClear]" if $verbose
      end
    end
  end

  def diagClearOverrun( slave )
    tx( slave, Diagnostics.chr + DiagClearOverrun.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        puts "Slave #{slave} [diagClearOverrun]" if $verbose
      end
    end
  end

  def diagGetBusyCount( slave )
    tx( slave, Diagnostics.chr + DiagGetBusyCount.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        busy = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

        puts "Slave #{slave} [diagGetBusyCount]:\n  busy: #{busy}" if $verbose
        busy
      end
    end
  end

  def diagGetErrorCount( slave )
    tx( slave, Diagnostics.chr + DiagGetErrorCount.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        errors = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

        puts "Slave #{slave} [diagGetErrorCount]:\n  bus errors: #{errors}" if $verbose
        errors
      end
    end
  end

  def diagGetExceptCount( slave )
    tx( slave, Diagnostics.chr + DiagGetExceptCount.to_word )
    slave, pdu = rx()

    unless is_error?( pdu )
      errors = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

      puts "Slave #{slave} [diagGetExceptCount]\n  exceptions: #{errors}" if $verbose
      errors
    end
  end

  def diagGetMsgCount( slave )
    tx( slave, Diagnostics.chr + DiagGetMsgCount.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        messages = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

        puts "Slave #{slave} [diagGetMsgCount]\n  messages: #{messages}" if $verbose
        messages
      end
    end
  end

  def diagGetNAKCount( slave )
    tx( slave, Diagnostics.chr + DiagGetNAKCount.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        naks = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

        puts "Slave #{slave} [diagGetNAKCount]\n  NAKs: #{naks}" if $verbose
        naks
      end
    end
  end

  def diagGetNoRespCount( slave )
    tx( slave, Diagnostics.chr + DiagGetNoRespCount.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        noResp = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

        puts "Slave #{slave} [diagGetNoRespCount]\n  no response: #{noResp}" if $verbose
        noResp
      end
    end
  end

  def diagGetOverrunCount( slave )
    tx( slave, Diagnostics.chr + DiagGetOverrunCount.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        overruns = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

        puts "Slave #{slave} [diagGetOverrunCount]\n  overruns: #{overruns}" if $verbose
        overruns
      end
    end
  end

  def diagGetRegister( slave )
    tx( slave, Diagnostics.chr + DiagGetRegister.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        register = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

        puts "Slave #{slave} [diagGetRegister]\n  register: #{register}" if $verbose
        register
      end
    end
  end

  def diagGetSlaveMsgCount( slave )
    tx( slave, Diagnostics.chr + DiagGetSlaveMsgCount.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        messages = pdu[ 3, 2 ].unpack( "n" )[ 0 ]

        puts "Slave #{slave} [diagGetSlaveMsgCount]\n  messages: #{messages}" if $verbose
        messages
      end
    end
  end

  def diagRestartComm( slave, clearLog = false )
    tx( slave, Diagnostics.chr + DiagRestartComm.to_word + (clearLog ? 0 : 0xff00).to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        puts "Slave #{slave} [diagRestartComm]:\n  log: #{clearLog ? "cleared" : "preserved"}" if $verbose
      end
    end
  end

  def diagReturnQuery( slave, data )
    tx( slave, Diagnostics.chr + DiagReturnQuery.to_word + data.pack( "c*" ) )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        puts "Slave #{slave} [diagReturnQuery]" if $verbose
        pdu[ 3..-1 ].unpack( "c*" )
      end
    end
  end

  def diagSetDelim( slave, delim )
    tx( slave, Diagnostics.chr + DiagSetDelim.to_word + delim + 0.chr )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        puts "Slave #{slave} [diagSetDelim]:\n  delimiter: \"" + delim + "\"" if $verbose
      end
    end
  end

  def diagSetListenOnly( slave )
    tx( slave, Diagnostics.chr + DiagSetListenOnly.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        puts "Slave #{slave} [diagSetListenOnly]" if $verbose
      end
    end
  end

  def encapCANopen( slave )
    puts "Not Yet Implemented"
  end

  def encapGetDeviceId( slave, idCode, objectId, array = nil )
    tx( slave, MEITransport.chr + EncapGetDeviceId.chr + idCode.chr + objectId.chr )
    if slave > 0
      slave, pdu = rx()
 
      unless is_error?( pdu )
        conformity = pdu[ 3 ]
        moreFollows = (0 != pdu[ 4 ].ord)
        nextObjectId = pdu[ 5 ]
        objectCount = pdu[ 6 ]

        continuation = !array.nil?
        array = [] if array.nil?
        list = pdu[ 7..-1 ]

        while 0 < list.length do
          length = list[ 1 ].ord
          array << [ list[ 0 ].ord, list[ 2, length ] ]
          list = list[ 2+length..-1 ]
        end
 
        encapGetDeviceId( slave, idCode, nextObjectId, array ) if moreFollows
 
        if !continuation && $verbose
          puts "Slave #{slave} [encapGetDeviceId]"
          puts "  conformity  : 0x%02x" % conformity
          puts "  object count: #{array.length}"

          array.each {|o| puts "  Object #{o[ 0 ]}: #{o[ 1 ]}" }
        end

        array
      end
    end
  end

  def getEventCount( slave )
    tx( slave, GetEventCount.chr )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        status, events = pdu[1, 4].unpack( "n2" )

        if $verbose
          puts "Slave #{slave} [getEventCount]:"
          puts "  status: #{0 == status ? "READY" : "BUSY"}"
          puts "  events:  #{events}"
        end
        [status, events]
      end
    end
  end

  def getEventLog( slave )
    tx( slave, GetEventLog.chr )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        status, events, messages = pdu[2, 6].unpack( "n3" )
        log = pdu[8..-1].unpack( "c*" )

        if $verbose
          puts "Slave #{slave} [getEventLog]:"
          puts "  status:   #{0 == status ? "READY" : "BUSY"}"
          puts "  events:   #{events}"
          puts "  messages: #{messages}"
        end
        log
      end
    end
  end

  def getExceptions( slave )
    tx( slave, GetExceptions.chr )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        if $verbose
          puts "Slave #{slave} [getExceptions]:"
          8.times { |i| puts "  exception ##{i}: #{0 == (1 << i) & pdu[ 1 ] ? "NO" : "YES"}" }
        end
        # FIX return array of exception values
      end
    end
  end

  def getSlaveId( slave )
    tx( slave, GetSlaveId.chr )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        puts "Slave #{slave} [getSlaveId]" if $verbose
        pdu[ 2..-1 ]
      end
    end
  end

  def readCoils( slave, address, count )
    tx( slave, ReadCoils.chr + address.to_word + count.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        coils = []
        pdu[ 2..-1 ].unpack( "b*" ).join.each_byte { |b| coils << b - ?0 }

        puts "Slave #{slave} [readCoils]" if $verbose
        coils[ 0, count ]
      end
    end
  end

  def readDiscretes( slave, address, count )
    tx( slave, ReadDiscretes.chr + address.to_word + count.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        discretes = []
        pdu[ 2..-1 ].unpack( "b*" ).join.each_byte { |b| discretes << b - ?0 }

        puts "Slave #{slave} [readDiscretes]" if $verbose
        discretes[ 0, count ]
      end
    end
  end

  def readFIFOQueue( slave, queue )
    tx( slave, ReadFIFOQueue.chr + queue.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        queue = pdu[ 5..-1 ].unpack( "n*" )

        puts "Slave #{slave} [readFIFOQueue]" if $verbose
        queue
      end
    end
  end

  # [[file1, start1, count1], [file2, start2, count2], ... ]
  def readFileRecord( slave, subreqs )
    pdu = ""
    subreqs.each { |sr| pdu << 6.chr << sr.pack( "nnn" ) }

    tx( slave, ReadFileRecord.chr + pdu.length.chr + pdu )
    if slave > 0
      slave, pdu = rx()
 
      unless is_error?( pdu )
        records = []
        offset = 2

        while offset < pdu.length - 3 do
          records << pdu[ 2 + offset, pdu[ offset ] - 1 ].unpack( "n*" )
          offset += pdu[ offset ] + 1
        end
 
        if $verbose
          puts "Slave #{slave} [readFileRecord]:"
          puts "  records: #{records.length}"
          puts "  total:   #{pdu[ 1 ]} bytes"
        end
        records
      end
    end
  end

  def readInputs( slave, address, count )
    tx( slave, ReadInputs.chr + address.to_word + count.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        puts "Slave #{slave} [readInputs]" if $verbose
        inputs = pdu[ 2..-1 ].unpack( "n*" )
      end
    end
  end

  def readRegisters( slave, address, count )
    tx( slave, ReadRegisters.chr + address.to_word + count.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        puts "Slave #{slave} [readRegisters]" if $verbose
        registers = pdu[ 2..-1 ].unpack( "n*" )
      end
    end
  end

  def readWriteRegs( slave, readAddr, count, writeAddr, values )
    length = values.length

    tx( slave, ReadWriteRegs.chr + readAddr.to_word + count.to_word +
        writeAddr.to_word + length.to_word + (length << 1).chr + values.pack( "n*" ) )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        puts "Slave #{slave} [readWriteRegs]" if $verbose
        inputs = pdu[ 2..-1 ].unpack( "n*" )
      end
    end
  end

  def writeCoil( slave, address, value )
    tx( slave, WriteCoil.chr + address.to_word + (0 == value ? 0 : 0xff00).to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        value = pdu[ 3, 2 ].unpack( "n" )
 
        puts "Slave #{slave} [writeCoil]:\n  coil #{address}: #{0 == value[ 0 ] ? "RESET" : "SET"}" if $verbose
        value
      end
    end
  end

  def writeCoils( slave, address, values )
    count = values.length
    pdu = ""
    0.step( count, 8 ) { |i| pdu << values[ i...i+8 ].join.reverse.to_i( 2 ).chr }

    tx( slave, WriteCoils.chr + address.to_word + count.to_word + pdu.length.chr + pdu )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        count = pdu[ 3, 2 ].unpack( "n" )
 
        puts "Slave #{slave} [writeCoils]:\n  #{count} coil(s) written" if $verbose
        count
      end
    end
  end

  # [[file1, start1, [data1]], [file2, start2, [data2]], ... ]
  def writeFileRecord( slave, subreqs )
    pdu = ""
    subreqs.each { |sr| pdu << 6.chr << sr.pack( "nn" ) << sr[ 2 ].pack( "n*" ) }

    tx( slave, WriteFileRecord.chr + pdu.length.chr + pdu )
    if slave > 0
      slave, pdu = rx()
 
      unless is_error?( pdu )
        if $verbose
          puts "Slave #{slave} [writeFileRecord]:"
          puts "  records: #{subreqs.length}"
          puts "  total:   #{pdu[ 1 ]} bytes"
        end
      end
    end
  end

  def writeRegister( slave, address, value )
    tx( slave, WriteRegister.chr + address.to_word + value.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        value = pdu[ 3, 2 ].unpack( "n" )

        puts "Slave #{slave} [writeRegister]:\n  register #{address}: 0x%04x" % value if $verbose
        value
      end
    end
  end

  def writeRegisters( slave, address, values )
    length = values.length

    tx( slave, WriteRegisters.chr + address.to_word + length.to_word + (length << 1).chr + values.pack( "n*" ) )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        count = pdu[ 3, 2 ].unpack( "n" )
 
        puts "Slave #{slave} [writeRegisters]:\n  #{count} register(s) written" if $verbose
        count
      end
    end
  end

  def writeRegMask( slave, address, andMask, orMask )
    tx( slave, WriteRegMask.chr + address.to_word + andMask.to_word + orMask.to_word )
    if slave > 0
      slave, pdu = rx()

      unless is_error?( pdu )
        puts "Slave #{slave} [writeRegMask]" if $verbose
      end
    end
  end
end
