import javax.comm.SerialPort;
import net.wimpi.modbus.Modbus;
import net.wimpi.modbus.facade.ModbusSerialMaster;
import net.wimpi.modbus.procimg.Register;
import net.wimpi.modbus.procimg.SimpleRegister;
import net.wimpi.modbus.util.SerialParameters;


/**
 *  Simple test to determine if jamod and Ifos speak the same language.
 *  Takes a port name, slave id, and a value to write to register 0.
 */
public class JamodTest
{
   private static void printUsage()
   {
      System.out.println( "java -cp .:jamod.jar JamodTest <port> <slave> <value>" );
   }

   public static void main( String[] args )
   {
      ModbusSerialMaster msm = null;
      String port = "/dev/ttyIfos";
      int slave = 3;
      int value = 1;

      try
      {
	 //  Initialize some parameters from the command line.
         if( 3 != args.length )
         {
            printUsage();
            System.exit( 1 );
         }
         else
	 {
            try
            {
               port  = args[ 0 ];
               slave = Integer.parseInt( args[ 1 ] );
               value = Integer.parseInt( args[ 2 ] );
            }
            catch( NumberFormatException nfe )
            {
               nfe.printStackTrace();
               printUsage();
               System.exit( 1 );
            }
         }

	 //  Get ready to initialize the hardware serial port and write a register.
         Register reg = new SimpleRegister( value );
         SerialParameters params = new SerialParameters( port,
                                                         19200,
                                                         SerialPort.FLOWCONTROL_NONE,
                                                         SerialPort.FLOWCONTROL_NONE,
                                                         SerialPort.DATABITS_7,
                                                         SerialPort.STOPBITS_1,
                                                         SerialPort.PARITY_EVEN,
                                                         false );

	 //  Create a master in ASCII mode, attach it to the serial port, then
	 //  write out the value.  Since Ifos maps its registers directly to the
	 //  LEDs, writing most non-zero values to register 0 will cause the first
	 //  multicolor LED to light up.
         params.setEncoding( Modbus.SERIAL_ENCODING_ASCII );
         msm = new ModbusSerialMaster( params );
         msm.connect();
         msm.writeSingleRegister( slave, 0, reg );
      }
      catch( Exception e )
      {
         e.printStackTrace();
      }
      finally
      {
	 //  Make sure we relinquish control of the harware port.
	 if( null != msm ) 
            msm.disconnect();
      }
   }
}
