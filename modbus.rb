require "serialport"

class Modbus
  def initialize( port, baud = 19200, databits = 8, stopbits = 1, parity = SerialPort::EVEN )
    @sp = SerialPort.open( port, baud, databits, stopbits, parity );
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
    (sum >> 8) + ((0xff & sum) << 8)
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
      adu += "%04x" % crc( adu )
    else
      adu  = ":%02x" % slave
      pdu.each_byte { |b| adu += "%02x" % b }
      adu += "%02x\r\n" % lrc( adu[1..-1] )
    end

#    sp.puts( adu )
    puts( adu )
  end

  def rx
  end
end

modbus = Modbus.new( "COM3" )
modbus.tx( 1, "ABCDEF0123456789" )
modbus.ascii!
modbus.tx( 1, "ABCDEF0123456789" )
