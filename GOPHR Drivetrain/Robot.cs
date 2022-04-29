using Microsoft.SPOT;
using System.Threading;

namespace GOPHR_Drivetrain
{
    public class Robot
    {
        public static void Main()
        {
            /*Configure CAN bus hardware*/
            Config.ConfigCoders();
            Config.ConfigSteerTalons();
            Config.ConfigDriveTalons();

            /*Setup UART for Comms with Pi*/
            Comms.UartSetup();

            /*Initialize global variables for heading angle and coordinates*/
            Var.startAngle = HW.pigeon.GetFusedHeading();
            Var.xCurrent = 0;
            Var.yCurrent = 0;
            Var.omegaCurrent = 0;

            Debug.Print("Select Robot Mode");
            Debug.Print("Hold A for Autonomous");
            Debug.Print("Hold X for Teleoperated");

            /*Begin main loop*/
            while (true)
            {
                Var.startAngle = HW.pigeon.GetFusedHeading();
                Var.xCurrent = 0;
                Var.yCurrent = 0;
                Var.omegaCurrent = 0;

                /*Use X Button to Enter Teleop Mode*/
                if (HW.myGamepad.GetButton(1) == true)
                {
                    Debug.Print("Teleoperated mode enabled, hold B to return to selection.");
                    /*Initialize current angle as desired heading for angle holding purposes*/
                    Var.currentAngle = HW.pigeon.GetFusedHeading();
                    Var.targetAngle = Var.currentAngle;

                    /*Initialize obstacle watchdog and open UART Comms*/
                    int obsWatchdog;
                    Comms._uart.Open();
                    while (true)
                    {
                        /*Get UART Comms*/
                        obsWatchdog = Comms.UartReadObsWatchdog();                        

                        //Debug.Print("obswatchdog: " + obsWatchdog);

                        if (HW.myGamepad.GetButton(7) == true && HW.myGamepad.GetButton(5) == true)
                        {
                            Var.maxSpeed = 1600f * 3;
                        }
                        //else if (HW.myGamepad.GetButton(7) == true)
                        //{
                        //    Var.maxSpeed = 1600f * 3;
                        //}
                        else
                        {
                            Var.maxSpeed = 1600f;
                        }

                            /*If gamepad connected: enable motor control*/
                            if (HW.myGamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected /*&& obsWatchdog == 49*/)
                        {
                            CTRE.Phoenix.Watchdog.Feed();
                        }

                        /*Get gamepad inputs and translate to module states*/
                        Kinematics.SetModuleStatesTeleop();

                        /*Output module states to steering and driving motors*/
                        steer.Steer();
                        Drive.Velocity();

                        /*Get chassis Kinematics for odometry*/
                        //Kinematics.GetChassisSpeeds();

                        /*Write odometry to UART buffer*/
                        //Comms.UartWriteOdom();

                        /*Get Chassis Position*/
                        //Kinematics.GetChassisPosition();

                        /*Use B Button to Exit Teleop Mode*/
                        if (HW.myGamepad.GetButton(3) == true)
                        {
                            Comms._uart.Close();
                            Debug.Print("Select Robot Mode");
                            Debug.Print("Hold A for Autonomous");
                            Debug.Print("Hold X for Teleoperated");
                            break;
                        }
                    }
                }

                /*Use A Button to Enter Auto Mode*/
                else if (HW.myGamepad.GetButton(2) == true)
                {

                    Comms._uart.Open();
                    Debug.Print("Auto mode enabled, waiting for waypoints...");

                    /*Use this to receive waypoints from pi:*/
                    Comms.UartReadWaypoints();

                    /*Use this to hardcode waypoints:*/
                    float[]testArray = {6.562f, 0, 0, 8.202f, -82.349f, 0, 10.499f, -82.349f, 0};
                    float[] testArrayBack = {6.562f, 0, 180, 8.702f, -82.349f, 100, 10.499f, -82.349f, 0 };
                    Var.waypointArray = testArray;

                    while (true)
                    {
                        int obsWatchdog = Comms.UartReadObsWatchdog();

                        /*If gamepad connected and obstacle watchdog == GO: enable motor control*/
                        if (HW.myGamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected /*&& obsWatchdog == 49*/)
                        {
                            CTRE.Phoenix.Watchdog.Feed();
                        }

                        Debug.Print("Following Path...");

                        int i = 0;

                        while (i < Var.waypointArray.Length / 3)
                        {
                            Kinematics.WaypointTracker(-Var.waypointArray[i * 3 + 1], Var.waypointArray[i * 3 + 0], Var.waypointArray[i * 3 + 2]);
                            Debug.Print("Waypoint " + (i + 1) +  " reached");
                            Thread.Sleep(500);
                            i = i + 1;
                            if (HW.myGamepad.GetButton(3) == true)
                            {
                                Debug.Print("Pathing cancelled");
                                break;
                            }
                        }

                        int t = 0;

                        Comms._uart.DiscardOutBuffer();

                        Thread.Sleep(1000);

                        Comms._uart.DiscardOutBuffer();

                        while (t < 1000)
                        {
                            Comms._uart.WriteByte(3);
                            t = t + 1;
                        }
                        

                        Thread.Sleep(1000);

                        Debug.Print("Endpoint reached");

                        Comms._uart.DiscardInBuffer();

                        while (true)
                        {
                            Comms._uart.WriteByte(3);
                            byte[] goByte = new byte[1];
                            
                            int bytesInBuffer = Comms._uart.BytesToRead;
                            Debug.Print("Bytes in Buffer: " + bytesInBuffer);

                            if (bytesInBuffer > 0)
                            {
                                Comms._uart.Read(goByte, 0, 1);
                                char[] goChar = System.Text.Encoding.UTF8.GetChars(goByte);
                                string goString = new string(goChar);
                                int goInt = int.Parse(goString);
                                Debug.Print("" + goInt);
                                if (goInt == 1)
                                {
                                    break;
                                }
                            }
                        }

                        i = i - 2;

                        Debug.Print("Returning home...");

                        Var.waypointArray = testArrayBack;

                        while (i >= 0)
                        {
                            Kinematics.WaypointTracker(-Var.waypointArray[i * 3 + 1], Var.waypointArray[i * 3 + 0], Var.waypointArray[i * 3 + 2]);
                            Debug.Print("Waypoint " + (i + 1) + " reached");
                            Thread.Sleep(500);
                            i = i - 1;
                            if (HW.myGamepad.GetButton(3) == true)
                            {
                                Debug.Print("Pathing cancelled");
                                break;
                            }
                        }

                        Kinematics.WaypointTracker(0, 2, 0);

                        t = 0;

                        Comms._uart.DiscardOutBuffer();

                        while (t < 600)
                        {
                            Comms._uart.WriteByte(2);
                            if (HW.myGamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                            {
                                CTRE.Phoenix.Watchdog.Feed();
                            }
                            Kinematics.SetModuleStatesAuto(0, -0.5f, 0);
                            steer.Steer();
                            Drive.Velocity();
                            t = t + 1;
                        }

                        

                        Debug.Print("Pathing Complete!");
                        Debug.Print("Hold B to return to mode selection.");

                        while (true)
                        {
                            Comms._uart.WriteByte(4);
                            /*Use B Button to Exit Auto Mode*/
                            if (HW.myGamepad.GetButton(3) == true)
                            {
                                Comms._uart.Close();
                                Debug.Print("Select Robot Mode");
                                Debug.Print("Hold A for Autonomous");
                                Debug.Print("Hold X for Teleoperated");
                                break;
                            }
                        }
                        break;
                    }
                }

                Thread.Sleep(1000);
            }
        }
    }
}