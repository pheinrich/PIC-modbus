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
    puts( "Tx: " + adu )
  end

  def rx
    adu = @sp.gets if @sp
    puts( "Rx: " + adu )

    if is_rtu?
      slave = adu[ 0 ]
      adu[ 1..-3 ].step( 2 ) { |i| pdu += adu[ i..i+1 ].hex }
      pdu = adu[ 1..-3 ]

      sum = crc( adu[ 0..-3 ] )
      if sum != adu[ -1 ] + (adu[ -2 ] << 8)
        puts( "CRC incorrect! (Calculated 0x%04x)" % sum )
      end
    else
      slave = adu[1..2].hex
      pdu = adu[ 3..-5 ]

      sum = lrc( adu[ 1..-5 ] )
      if sum != adu[ -4..-3 ].hex
        puts( "LRC incorrect! (Calculated 0x%02x)" % sum )
      end

    end

    slave, pdu
  end
end

modbus = Modbus.new( "COM4" )
modbus.tx( 1, "ABCDEF0123456789" )
modbus.ascii!
modbus.tx( 1, "ABCDEF0123456789" )
