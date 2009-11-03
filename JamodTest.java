// ---------------------------------------------------------------------------
//
//  PIC Modbus
//  Copyright (c) 2006,2008  Peter Heinrich
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.
//
//  Linking this library statically or dynamically with other modules
//  is making a combined work based on this library. Thus, the terms
//  and conditions of the GNU General Public License cover the whole
//  combination.
//
//  As a special exception, the copyright holders of this library give
//  you permission to link this library with independent modules to
//  produce an executable, regardless of the license terms of these
//  independent modules, and to copy and distribute the resulting
//  executable under terms of your choice, provided that you also meet,
//  for each linked independent module, the terms and conditions of the
//  license of that module. An independent module is a module which is
//  not derived from or based on this library. If you modify this
//  library, you may extend this exception to your version of the
//  library, but you are not obligated to do so. If you do not wish to
//  do so, delete this exception statement from your version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin Street, Boston, MA  02110-1301, USA.
//
// ---------------------------------------------------------------------------



import gnu.io.SerialPort;
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
      String port = "/dev/tty.Ifos";
      int slave = 4;
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
