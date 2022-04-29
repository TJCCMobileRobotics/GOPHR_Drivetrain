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
    public static class Drive
    {

        public static void Velocity()
        {
            int velocityDb = 200;

            if (Var.drive01 > velocityDb || Var.drive01 < -velocityDb)
            {
                HW.talon01.Set(ControlMode.Velocity, Var.drive01);
            }
            else
            {
                HW.talon01.Set(ControlMode.PercentOutput, 0);
            }
            if (Var.drive11 > velocityDb || Var.drive11 < -velocityDb)
            {
                HW.talon11.Set(ControlMode.Velocity, Var.drive11);
            }
            else
            {
                HW.talon11.Set(ControlMode.PercentOutput, 0);
            }
            if (Var.drive21 > velocityDb || Var.drive21 < -velocityDb)
            {
                HW.talon21.Set(ControlMode.Velocity, Var.drive21);
            }
            else
            {
                HW.talon21.Set(ControlMode.PercentOutput, 0);
            }
            if (Var.drive31 > velocityDb || Var.drive31 < -velocityDb)
            {
                HW.talon31.Set(ControlMode.Velocity, Var.drive31);
            }
            else
            {
                HW.talon31.Set(ControlMode.PercentOutput, 0);
            }

        }

        readonly private static float rampDist = Config.FeetToTicks(2);

        public static float Trapezoid(float distanceTarget, float distanceTraveled)
        {
            float output;
            if (Var.pathProgress <= 0.5)
            {
                if (distanceTraveled <= rampDist)
                {
                    output = (float)(0.2 + (distanceTraveled * 0.8) / rampDist);
                }
                else
                {
                    output = 1;
                }
            }
            else if (distanceTraveled >= distanceTarget - rampDist)
            {
                //output = (float)(0.2 + (((distanceTarget - distanceTraveled) * 0.8) / rampDist));
                output = (float)(0.2 + (((distanceTarget - distanceTraveled) * 0.8) / rampDist));
            }
            else
            {
                output = 1;
            }

            //Debug.Print("Output: " + output);
            return output;
        }
    }
}