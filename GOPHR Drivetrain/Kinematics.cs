using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using System;
using System.Threading;

using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Tasking;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;
using CTRE.Phoenix.Motion.Profile;

namespace GOPHR_Drivetrain
{
    public static class Kinematics
    {

        private static float VxActual;
        private static float VyActual;
        private static float omegaActual;

        private static float VxTarget;
        private static float VyTarget;

        private static float Vx0;
        private static float Vy0;

        private static float Vx1;
        private static float Vy1;

        private static float Vx2;
        private static float Vy2;

        private static float Vx3;
        private static float Vy3;

        /*public static void GetChassisSpeeds()
        {
            float coder03Pos = HW.talon02.GetSelectedSensorPosition();
            float talon01Vel = HW.talon01.GetSelectedSensorVelocity();
            if (true == HW.talon01.GetInverted())
            {
                talon01Vel *= -1;
            }
            Vx0 = (float)System.Math.Sin(coder03Pos / 4096 * System.Math.PI * 2) * talon01Vel;
            Vy0 = (float)System.Math.Cos(coder03Pos / 4096 * System.Math.PI * 2) * talon01Vel;

            float coder13Pos = HW.talon12.GetSelectedSensorPosition();
            float talon11Vel = HW.talon11.GetSelectedSensorVelocity();
            if (false == HW.talon11.GetInverted())
            {
                talon11Vel *= -1;
            }
            Vx1 = (float)System.Math.Sin(coder13Pos / 4096 * System.Math.PI * 2) * talon11Vel;
            Vy1 = (float)System.Math.Cos(coder13Pos / 4096 * System.Math.PI * 2) * talon11Vel;

            float coder23Pos = HW.talon22.GetSelectedSensorPosition();
            float talon21Vel = HW.talon21.GetSelectedSensorVelocity();
            if (true == HW.talon21.GetInverted())
            {
                talon21Vel *= -1;
            }
            Vx2 = (float)System.Math.Sin(coder23Pos / 4096 * System.Math.PI * 2) * talon21Vel;
            Vy2 = (float)System.Math.Cos(coder23Pos / 4096 * System.Math.PI * 2) * talon21Vel;

            float coder33Pos = HW.talon32.GetSelectedSensorPosition();
            float talon31Vel = HW.talon31.GetSelectedSensorVelocity();
            if (false == HW.talon31.GetInverted())
            {
                talon31Vel *= -1;
            }
            Vx3 = (float)System.Math.Sin(coder33Pos / 4096 * System.Math.PI * 2) * talon31Vel;
            Vy3 = (float)System.Math.Cos(coder33Pos / 4096 * System.Math.PI * 2) * talon31Vel;

            VxActual = (float)(-(0.25 * Vx0 + 0.25 * Vx1 + 0.25 * Vx2 + 0.25 * Vx3));
            VyActual = (float)(0.25 * Vy0 + 0.25 * Vy1 + 0.25 * Vy2 + 0.25 * Vy3);
            omegaActual = (float)(0.0143 * Vx0 + -0.0143 * Vy0 + 0.0143 * Vx1 + 0.0143 * Vy1 + -0.0143 * Vx2 + -0.0143 * Vy2 + 0.0143 * Vx3 + -0.0143 * Vy3);

            Var.vxOdom = Config.TicksToFeet(VxActual) * 10;
            Var.vyOdom = Config.TicksToFeet(VyActual) * 10;
            Var.vOmegaOdom = omegaActual;

            //Debug.Print("Vx Odometry (fps): " + Var.vxOdom);
            //Debug.Print("Vy Odometry (fps): " + Var.vyOdom);
            //Debug.Print("Vomega Odometry (???): " + Var.vOmegaOdom);
        }

        /*private static float coder01Last = 0;
        private static float coder11Last = 0;
        private static float coder21Last = 0;
        private static float coder31Last = 0;

        private static float x0Pos = 0;
        private static float y0Pos = 0;
        private static float x1Pos = 0;
        private static float y1Pos = 0;
        private static float x2Pos = 0;
        private static float y2Pos = 0;
        private static float x3Pos = 0;
        private static float y3Pos = 0;

        public static void GetChassisPosition()
        {
            float coder03Pos = HW.talon02.GetSelectedSensorPosition();
            float coder01Pos = HW.talon01.GetSelectedSensorPosition();
            if (true == HW.talon01.GetInverted())
            {
                coder01Pos *= -1;
            }

            x0Pos = x0Pos + System.Math.Sin(coder03Pos / 4096 * System.Math.PI * 2) * (coder01Pos - coder01Last);
            y0Pos = y0Pos + System.Math.Cos(coder03Pos / 4096 * System.Math.PI * 2) * (coder01Pos - coder01Last);

            float coder13Pos = HW.talon12.GetSelectedSensorPosition();
            float coder11Pos = HW.talon11.GetSelectedSensorPosition();
            if (true == HW.talon11.GetInverted())
            {
                coder11Pos *= -1;
            }

            x1Pos = x1Pos + System.Math.Sin(coder13Pos / 4096 * System.Math.PI * 2) * (coder11Pos - coder11Last);
            y1Pos = y1Pos + System.Math.Cos(coder13Pos / 4096 * System.Math.PI * 2) * (coder11Pos - coder11Last);

            float coder23Pos = HW.talon22.GetSelectedSensorPosition();
            float coder21Pos = HW.talon21.GetSelectedSensorPosition();
            if (true == HW.talon21.GetInverted())
            {
                coder21Pos *= -1;
            }

            x2Pos = x2Pos + System.Math.Sin(coder23Pos / 4096 * System.Math.PI * 2) * (coder21Pos - coder21Last);
            y2Pos = y2Pos + System.Math.Cos(coder23Pos / 4096 * System.Math.PI * 2) * (coder21Pos - coder21Last);

            float coder33Pos = HW.talon32.GetSelectedSensorPosition();
            float coder31Pos = HW.talon31.GetSelectedSensorPosition();
            if (true == HW.talon31.GetInverted())
            {
                coder31Pos *= -1;
            }

            x3Pos = x3Pos + System.Math.Sin(coder33Pos / 4096 * System.Math.PI * 2) * (coder31Pos - coder31Last);
            y3Pos = y3Pos + System.Math.Cos(coder33Pos / 4096 * System.Math.PI * 2) * (coder31Pos - coder31Last);

            Var.xCurrent = 0.25 * x0Pos + 0.25 * x1Pos + 0.25 * x2Pos + 0.25 * x3Pos;
            Var.yCurrent = 0.25 * y0Pos + 0.25 * y1Pos + 0.25 * Vy2 + 0.25 * y3Pos;
            Var.omegaCurrent = -0.0143 * x0Pos + 0.0143 * y0Pos + 0.0143 * x1Pos + 0.0143 * y1Pos + -0.0143 * x2Pos + -0.0143 * y2Pos + 0.0143 * x3Pos + -0.0143 * y3Pos;

            Var.xCurrent = Config.TicksToFeet(Var.xCurrent);
            Var.yCurrent = Config.TicksToFeet(Var.yCurrent);
            Var.omegaCurrent = Config.TicksToFeet(Var.omegaCurrent);

            coder01Last = coder01Pos;
            coder11Last = coder11Pos;
            coder21Last = coder21Pos;
            coder31Last = coder31Pos;

            Debug.Print("last: " + coder01Last);

            Debug.Print("X Position: " + Var.xCurrent);
            Debug.Print("Y Position: " + Var.yCurrent);
            Debug.Print("Omega Position: " + Var.omegaCurrent);

        }*/
        public static void SetModuleStatesTeleop()
        {
            /*Get Gamepad Axes for Teleop Driving*/
            Var.axis0 = HW.myGamepad.GetAxis(0);
            Var.axis1 = -HW.myGamepad.GetAxis(1);
            Var.axis2 = HW.myGamepad.GetAxis(2);

            Var.axis0 = Var.axis0 * Var.axis0 * System.Math.Sign(Var.axis0);
            Var.axis1 = Var.axis1 * Var.axis1 * System.Math.Sign(Var.axis1);

            Var.currentAngle = HW.pigeon.GetFusedHeading();

            /*Fix robot orientation to correct front (rotate 90 degrees)*/
            float newaxis0 = Var.axis0 * (float)System.Math.Cos(-90 * System.Math.PI / 180) + Var.axis1 * (float)System.Math.Sin(-90 * System.Math.PI / 180);
            float newaxis1 = Var.axis1 * (float)System.Math.Cos(-90 * System.Math.PI / 180) - Var.axis0 * (float)System.Math.Sin(-90 * System.Math.PI / 180);
            Var.axis0 = (float)newaxis0;
            Var.axis1 = (float)newaxis1;

            /*Field Centric*/
            newaxis0 = Var.axis0 * (float)System.Math.Cos(Var.currentAngle * System.Math.PI / 180) + Var.axis1 * (float)System.Math.Sin(Var.currentAngle * System.Math.PI / 180);
            newaxis1 = Var.axis1 * (float)System.Math.Cos(Var.currentAngle * System.Math.PI / 180) - Var.axis0 * (float)System.Math.Sin(Var.currentAngle * System.Math.PI / 180);
            Var.axis0 = (float)newaxis0;
            Var.axis1 = (float)newaxis1;

            /*Set Vx and Vy Chassis targets using axis inputs and scale by finding hypot, max, min, and scaling by max speed setting*/
            if (System.Math.Abs(Var.axis0) != System.Math.Abs(Var.axis1))
            {
                VxTarget = Var.axis0 * Var.maxSpeed;
                VyTarget = Var.axis1 * Var.maxSpeed;
            }
            else if (Var.axis0 !=0 && Var.axis1 !=0)
            {
                VxTarget = Var.axis0 * Var.maxSpeed;
                VyTarget = Var.axis1 * Var.maxSpeed;
            }
            else
            {
                VxTarget = 0;
                VyTarget = 0;
            }

            /*Set omegea chassis rotation speed target using axis value*/
            Var.omegaTarget = (float)(System.Math.Sign(Var.axis2) * Var.maxSpeed * Var.axis2 * Var.axis2 * 0.05);

            /*Hold current angle using pigeon heading*/
            if (Var.axis2 > 0.1f || Var.axis2 < -0.1f)
            {
                Var.currentAngle = HW.pigeon.GetFusedHeading();
                Var.targetAngle = Var.currentAngle;
            }
            else
            {
                HoldAngle();
            }

            /*Calculate Module velocities*/
            Vx0 = (float)(-8.75 * Var.omegaTarget + VxTarget);
            Vy0 = (float)(8.75 * Var.omegaTarget + VyTarget);
            Vx1 = (float)(8.75 * Var.omegaTarget + VxTarget);
            Vy1 = (float)(8.75 * Var.omegaTarget + VyTarget);
            Vx2 = (float)(-8.75 * Var.omegaTarget + VxTarget);
            Vy2 = (float)(-8.75 * Var.omegaTarget + VyTarget);
            Vx3 = (float)(8.75 * Var.omegaTarget + VxTarget);
            Vy3 = (float)(-8.75 * Var.omegaTarget + VyTarget);

            /*Set module 0 speed and steer angle*/
            Var.drive01 = (float)System.Math.Sqrt(Vx0 * Vx0 + Vy0 * Vy0);
            if (Vx0 == 0 && Vy0 == 0)
            {
                Var.steer02 = 0;
            }
            else
            {
                Var.steer02 = (float)(System.Math.Atan2(Vy0, Vx0) / (2 * System.Math.PI) * 360);
            }

            /*Set module 1 speed and steer angle*/
            Var.drive11 = (float)System.Math.Sqrt(Vx1 * Vx1 + Vy1 * Vy1);
            if (Vx1 == 0 && Vy1 == 0)
            {
                Var.steer12 = 0;
            }
            else
            {
                Var.steer12 = (float)(System.Math.Atan2(Vy1, Vx1) / (2 * System.Math.PI) * 360);
            }

            /*Set module 2 speed and steer angle*/
            Var.drive21 = (float)System.Math.Sqrt(Vx2 * Vx2 + Vy2 * Vy2);
            if (Vx2 == 0 && Vy2 == 0)
            {
                Var.steer22 = 0;
            }
            else
            {
                Var.steer22 = (float)(System.Math.Atan2(Vy2, Vx2) / (2 * System.Math.PI) * 360);
            }

            /*Set module 3 speed and steer angle*/
            Var.drive31 = (float)System.Math.Sqrt(Vx3 * Vx3 + Vy3 * Vy3);
            if (Vx3 == 0 && Vy3 == 0)
            {
                Var.steer32 = 0;
            }
            else
            {
                Var.steer32 = (float)(System.Math.Atan2(Vy3, Vx3) / (2 * System.Math.PI) * 360);
            }
        }
        public static void SetModuleStatesAuto(float VxTarget, float VyTarget, float omegaTarget)
        {
            VxTarget *= Var.maxSpeed;
            VyTarget *= Var.maxSpeed;
            Var.omegaTarget = omegaTarget * Var.maxSpeed * 0.1f;

            //Debug.Print("omega Target: " + Var.omegaTarget);

            if (Var.omegaTarget > 1000)
            {
                Var.omegaTarget = 1000;
            }
            else if (Var.omegaTarget < -1000)
            {
                Var.omegaTarget = -1000;
            }

            //if (Var.omegaTarget < 0.5f && Var.omegaTarget > -0.5f)
            //{
            //    HoldAngle();
            //}

            /*Fix robot orientation to correct front (rotate 90 degrees)*/
            float newVxTarget = VxTarget * (float)System.Math.Cos(-90 * System.Math.PI / 180) + VyTarget * (float)System.Math.Sin(-90 * System.Math.PI / 180);
            float newVyTarget = VyTarget * (float)System.Math.Cos(-90 * System.Math.PI / 180) - VxTarget * (float)System.Math.Sin(-90 * System.Math.PI / 180);
            VxTarget = newVxTarget;
            VyTarget = newVyTarget;

            /*Calculate Module velocities*/
            Vx0 = (float)(-8.75 * Var.omegaTarget + VxTarget);
            Vy0 = (float)(8.75 * Var.omegaTarget + VyTarget);
            Vx1 = (float)(8.75 * Var.omegaTarget + VxTarget);
            Vy1 = (float)(8.75 * Var.omegaTarget + VyTarget);
            Vx2 = (float)(-8.75 * Var.omegaTarget + VxTarget);
            Vy2 = (float)(-8.75 * Var.omegaTarget + VyTarget);
            Vx3 = (float)(8.75 * Var.omegaTarget + VxTarget);
            Vy3 = (float)(-8.75 * Var.omegaTarget + VyTarget);

            /*Set module 0 steer angle*/
            Var.drive01 = (float)System.Math.Sqrt(Vx0 * Vx0 + Vy0 * Vy0);
            if (Vx0 == 0 && Vy0 == 0)
            {
                Var.steer02 = 0;
            }
            else
            {
                Var.steer02 = (float)(System.Math.Atan2(Vy0, Vx0) / (2 * System.Math.PI) * 360);
            }

            /*Set module 1 steer angle*/
            Var.drive11 = (float)System.Math.Sqrt(Vx1 * Vx1 + Vy1 * Vy1);
            if (Vx1 == 0 && Vy1 == 0)
            {
                Var.steer12 = 0;
            }
            else
            {
                Var.steer12 = (float)(System.Math.Atan2(Vy1, Vx1) / (2 * System.Math.PI) * 360);
            }

            /*Set module 2 steer angle*/
            Var.drive21 = (float)System.Math.Sqrt(Vx2 * Vx2 + Vy2 * Vy2);
            if (Vx2 == 0 && Vy2 == 0)
            {
                Var.steer22 = 0;
            }
            else
            {
                Var.steer22 = (float)(System.Math.Atan2(Vy2, Vx2) / (2 * System.Math.PI) * 360);
            }

            /*Set module 3 and steer angle*/
            Var.drive31 = (float)System.Math.Sqrt(Vx3 * Vx3 + Vy3 * Vy3);
            if (Vx3 == 0 && Vy3 == 0)
            {
                Var.steer32 = 0;
            }
            else
            {
                Var.steer32 = (float)(System.Math.Atan2(Vy3, Vx3) / (2 * System.Math.PI) * 360);
            }
        }

        public static void WaypointTracker(float xTarget, float yTarget, float omegaTarget)
        {
            float distanceX = xTarget - Var.xCurrent;
            float distanceY = yTarget - Var.yCurrent;

            float heading;

            float hypot = (float)(System.Math.Sqrt(distanceX * distanceX + distanceY * distanceY));

            float distanceTicks = Config.FeetToTicks(hypot);

            if (distanceX != 0)
            {
                heading = (float)(90 - (System.Math.Atan2(distanceY, distanceX) * 180 / System.Math.PI));
            }
            else
            {
                heading = (System.Math.Sign(distanceY) * 90) - 90;
            }

            Var.targetAngle = (float)(Var.startAngle - heading);

            Var.currentAngle = HW.pigeon.GetFusedHeading();
            float initialAngle = Var.currentAngle;

            while (System.Math.Abs(Var.currentAngle - Var.targetAngle) > 0.1)
            {
                Var.currentAngle = HW.pigeon.GetFusedHeading();
                //Debug.Print("initial angle: " + initialAngle);
                //Debug.Print("target angle: " + Var.targetAngle);
                //Debug.Print("current angle: " + Var.currentAngle);
                float pidOutput = (float)Controllers.TurnPidController(initialAngle, Var.currentAngle, Var.targetAngle, 0.1f, 0.001f, 0f);
                Kinematics.SetModuleStatesAuto(0, 0, pidOutput);
                steer.Steer();
                Drive.Velocity();
                //Debug.Print("target angle: " + Var.targetAngle);
                //Debug.Print("current angle: " + Var.currentAngle);

                if (HW.myGamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                {
                    CTRE.Phoenix.Watchdog.Feed();
                }

                if (HW.myGamepad.GetButton(3) == true)
                {
                    break;
                }
            }

            Thread.Sleep(250);

            float distanceTraveled;
            float velocity;
            HW.talon01.SetSelectedSensorPosition(0);
            Var.targetAngle = HW.pigeon.GetFusedHeading();
            Var.pathProgress = 0;
            Var.omegaTarget = 0;

            Thread.Sleep(250);

            while (Var.pathProgress < 1)
            {
                distanceTraveled = HW.talon01.GetSelectedSensorPosition();
                Var.pathProgress = distanceTraveled / distanceTicks;
                velocity = Drive.Trapezoid(distanceTicks, distanceTraveled);
                Kinematics.SetModuleStatesAuto(0, velocity, -Var.omegaTarget);
                HoldAngle();
                steer.Steer();
                Drive.Velocity();

                if (HW.myGamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                {
                    CTRE.Phoenix.Watchdog.Feed();
                }
            }

            Var.targetAngle = (float)(Var.startAngle + omegaTarget);

            Var.currentAngle = HW.pigeon.GetFusedHeading();
            initialAngle = Var.currentAngle;

            Thread.Sleep(250);

            while (System.Math.Abs(Var.currentAngle - Var.targetAngle) > 0.1)
            {
                Var.currentAngle = HW.pigeon.GetFusedHeading();
                //Debug.Print("initial angle: " + initialAngle);
                //Debug.Print("target angle: " + Var.targetAngle);
                //Debug.Print("current angle: " + Var.currentAngle);
                float pidOutput = (float)Controllers.TurnPidController(initialAngle, Var.currentAngle, Var.targetAngle, 0.1f, 0.001f, 0f);
                Kinematics.SetModuleStatesAuto(0, 0, pidOutput);
                steer.Steer();
                Drive.Velocity();

                if (HW.myGamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                {
                    CTRE.Phoenix.Watchdog.Feed();
                }
            }

            Thread.Sleep(250);

            Var.xCurrent = xTarget;
            Var.yCurrent = yTarget;
            Var.omegaCurrent = omegaTarget;

        }

        public static void HoldAngle()
        {
            Var.currentAngle = HW.pigeon.GetFusedHeading();

            Var.angleError = Var.currentAngle - Var.targetAngle;

            Var.omegaTarget = -Var.angleError * 0.05f;
        }
    }
}
