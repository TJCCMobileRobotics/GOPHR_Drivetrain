using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using System;
using System.Threading;

namespace GOPHR_Drivetrain
{
    public static class Comms
    {
        /*Create Serial Port for UART*/
        public static System.IO.Ports.SerialPort _uart = new System.IO.Ports.SerialPort(CTRE.HERO.IO.Port1.UART, 9600);

        /*Some variables for UART*/
        private static int byteRead;
        private static int bytesInBuffer;
        private static int bytesOutBuffer;

        /*Configure UART settings*/
        public static void UartSetup()
        {
            Comms._uart.ReadTimeout = 50;
            Comms._uart.WriteTimeout = 50;
        }

        public static int UartReadObsWatchdog()
        {
            bytesInBuffer = _uart.BytesToRead;

            if (bytesInBuffer > 0)
            {
                byteRead = Comms._uart.ReadByte();
            }
            //Debug.Print("Read: " + byteRead);
            return byteRead;
        }

        public static void UartReadWaypoints()
        {
            byte[] numWaypointsByte = new byte[2];
            char[] numWaypointsChars;
            int numWaypoints = 0;

            int i = 0;

            while (true)
            {
                bytesInBuffer = _uart.BytesToRead;
                //Debug.Print("Bytes in Buffer: " + bytesInBuffer);

                if (bytesInBuffer > 0)
                {
                    byteRead = Comms._uart.Read(numWaypointsByte, 0, bytesInBuffer);
                    numWaypointsChars = System.Text.Encoding.UTF8.GetChars(numWaypointsByte);
                    string numWaypointsString = new string(numWaypointsChars);
                    numWaypoints = int.Parse(numWaypointsString);
                    Debug.Print("Number Waypoints: " + numWaypoints);
                    break;
                }

                Thread.Sleep(1000);
            }

            Var.waypointArray = new float[3 * numWaypoints];

            Comms._uart.DiscardInBuffer();

            while (i < (numWaypoints * 3))
            {
                Comms._uart.WriteByte(1);
                byte[] waypointByte = new byte[40];
                char[] waypointChars;

                bytesInBuffer = _uart.BytesToRead;

                if (bytesInBuffer > 0)
                {
                    Comms._uart.DiscardOutBuffer();
                    Comms._uart.WriteByte(0);
                    byteRead = Comms._uart.Read(waypointByte, 0, bytesInBuffer);
                    waypointChars = System.Text.Encoding.UTF8.GetChars(waypointByte);
                    string waypointString = new string(waypointChars);
                    int waypointInt = int.Parse(waypointString);
                    float waypointFloat = (float)waypointInt / 1000000;
                    Var.waypointArray[i] = waypointFloat;
                    i = i + 1;
                }             
                Thread.Sleep(100);
            }
            i = 0;
            Debug.Print("Waypoint Fully Received");

            while (i < numWaypoints)
            {
                Debug.Print("X Coordinate: " + Var.waypointArray[i * 3 + 0]);
                Var.waypointArray[i * 3 + 0] = Config.MetersToFeet(Var.waypointArray[i * 3 + 0]);
                Debug.Print("Y Coordinate: " + Var.waypointArray[i * 3 + 1]);
                Var.waypointArray[i * 3 + 1] = Config.MetersToFeet(Var.waypointArray[i * 3 + 1]);
                Debug.Print("Omega Rotation: " + Var.waypointArray[i * 3 + 2]);
                i = i + 1;
            }
            Debug.Print(Var.waypointArray.Length / 3 + "waypoints recieved"); 
        }

        public static void UartWriteOdom()
        {
            int vxOdomWhole = (int)(System.Math.Truncate(Var.vxOdom));
            int vxOdomDecimals = (int)((Var.vxOdom - vxOdomWhole)*1000);
            int vyOdomWhole = (int)(System.Math.Truncate(Var.vyOdom));
            int vyOdomDecimals = (int)((Var.vyOdom - vyOdomWhole)*1000);
            int vOmegaOdomWhole = (int)(System.Math.Truncate(Var.vOmegaOdom));
            int vOmegaOdomDecimals = (int)((Var.vOmegaOdom - vOmegaOdomWhole)*1000);

            /*declare byte variables and convert odom integers to 4 byte arrays*/
            byte[] vxOdomWholeByte;
            byte[] vxOdomDecimalsByte;
            byte[] vyOdomWholeByte;
            byte[] vyOdomDecimalsByte;
            byte[] vOmegaOdomWholeByte;
            byte[] vOmegaOdomDecimalsByte;
            vxOdomWholeByte = BitConverter.GetBytes(vxOdomWhole);
            vxOdomDecimalsByte = BitConverter.GetBytes(vxOdomDecimals);
            vyOdomWholeByte = BitConverter.GetBytes(vyOdomWhole);
            vyOdomDecimalsByte = BitConverter.GetBytes(vyOdomDecimals);
            vOmegaOdomWholeByte = BitConverter.GetBytes(vOmegaOdomWhole);
            vOmegaOdomDecimalsByte = BitConverter.GetBytes(vOmegaOdomDecimals);

            vxOdomDecimals = BitConverter.ToInt32(vxOdomDecimalsByte, 0);

            Debug.Print("Test: " + vxOdomDecimals);

            /*Concatenate odom byte arrays to one 24 byte array*/
            byte[] odomBytesToWrite = new byte[24];
            vxOdomWholeByte.CopyTo(odomBytesToWrite, 0);
            vxOdomDecimalsByte.CopyTo(odomBytesToWrite, 4);
            vyOdomWholeByte.CopyTo(odomBytesToWrite, 8);
            vyOdomDecimalsByte.CopyTo(odomBytesToWrite, 12);
            vOmegaOdomWholeByte.CopyTo(odomBytesToWrite, 16);
            vOmegaOdomDecimalsByte.CopyTo(odomBytesToWrite, 20);

            /*Write odom byte array to UART buffer*/

            bytesOutBuffer = _uart.BytesToWrite;

            Comms._uart.Write(odomBytesToWrite, 0, 24);
        }


    }
}
