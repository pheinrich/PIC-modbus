#!/usr/bin/ruby

require "serialport"

DEF_BAUD = 19200
DEF_DATABITS = 8
DEF_STOPBITS = 1
DEF_PARITY = SerialPort::EVEN

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
    pdu.each do |b|
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
      adu  = [ slave ]
      adu += pdu.unpack( "c#{pdu.length}" )
      sum  = crc( adu )
      adu += [ 0xff & sum, sum >> 8 ]
      adu  = adu.pack( "c#{adu.length}" )
    else
      adu  = ":%02x" % slave
      pdu.each_byte { |b| adu += "%02x" % b }
      adu += "%02x\r\n" % lrc( adu[ 1..-1 ] )
    end

    @sp.puts( adu ) if @sp
    puts( adu )
  end

  def rx
  end
end

modbus = Modbus.new( "COM4" )
modbus.tx( 1, "ABCDEF0123456789" )
modbus.ascii!
modbus.tx( 1, "ABCDEF0123456789" )
