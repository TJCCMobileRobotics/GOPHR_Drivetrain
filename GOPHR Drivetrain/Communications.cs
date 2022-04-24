﻿using Microsoft.SPOT;
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
            bytesInBuffer = _uart.BytesToRead;

            if (bytesInBuffer > 0)
            {
                byteRead = Comms._uart.ReadByte();
            }

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
