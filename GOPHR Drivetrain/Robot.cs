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
                            Var.maxSpeed = 1600f * 8;
                        }
                        else if (HW.myGamepad.GetButton(7) == true)
                        {
                            Var.maxSpeed = 1600f * 3;
                        }
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
                    //Comms.UartReadWaypoints();

                    /*Use this to hardcode waypoints:*/
                    float[]testArray = {0, 3, 0, 3, 3, 0};
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
                            Kinematics.WaypointTracker(Var.waypointArray[i * 3 + 0], Var.waypointArray[i * 3 + 1], Var.waypointArray[i * 3 + 2]);
                            Debug.Print("Waypoint " + (i + 1) +  " reached");
                            Thread.Sleep(500);
                            i = i + 1;
                        }

                        Debug.Print("Pathing Complete!");
                        Debug.Print("Hold B to return to mode selection.");

                        while (true)
                        {   /*Use B Button to Exit Auto Mode*/
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