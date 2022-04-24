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
                    Debug.Print("Teleop Mode Enabled");
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

                        Debug.Print("obswatchdog: " + obsWatchdog);

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
                            break;
                        }
                    }
                }

                /*Use A Button to Enter Auto Mode*/
                else if (HW.myGamepad.GetButton(2) == true)
                {
                    Debug.Print("Auto Mode Enabled");

                    //Comms.UartReadWaypoints();

                    while (true)
                    {
                        int obsWatchdog = Comms.UartReadObsWatchdog();

                        /*If gamepad connected and obstacle watchdog == GO: enable motor control*/
                        if (HW.myGamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected /*&& obsWatchdog == 49*/)
                        {
                            CTRE.Phoenix.Watchdog.Feed();
                        }

                        Kinematics.WaypointTracker(0, 10, 0);

                        Thread.Sleep(250);

                        Kinematics.WaypointTracker(10, 10, 180);

                        Thread.Sleep(250);

                        Kinematics.WaypointTracker(10, 5, 0);

                        Thread.Sleep(250);

                        Kinematics.WaypointTracker(10, 10, 0);

                        Thread.Sleep(250);
//
                        Kinematics.WaypointTracker(0, 10, 180);

                        Thread.Sleep(250);

                        Kinematics.WaypointTracker(0, 0, 0);

                        while (true)
                        {   /*Use B Button to Exit Auto Mode*/
                            if (HW.myGamepad.GetButton(3) == true)
                            {
                                break;
                            }
                        }
                        break;
                    }
                }

                Debug.Print("Select Robot Mode");

                Thread.Sleep(1000);

                //Kinematics.WaypointTracker(0, 10, 0);

                //Var.currentAngle = HW.pigeon.GetFusedHeading();

                //float pidOutput = Controllers.TurnPidController(Var.currentAngle, Var.targetAngle, 0.1, 0.001, 0);

                //Kinematics.SetModuleStatesAuto(0, 0, pidOutput);

                //steer.Steer();

                //Drive.Velocity();


            }
        }
    }
}