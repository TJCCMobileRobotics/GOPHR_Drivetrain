
using CTRE.Phoenix.MotorControl;

namespace GOPHR_Drivetrain
{
    public static class steer
    {
        public static float coder03Val;
        public static float coder13Val;
        public static float coder23Val;
        public static float coder33Val;

        public static float coder03Target;
        public static float coder13Target;
        public static float coder23Target;
        public static float coder33Target;

        public static float targetAngleTicks;
        public static float newTargetAngle;

        public static void Steer()
        {
            /*Get Steer CANcoder positions*/
            coder03Val = HW.talon02.GetSelectedSensorPosition();
            coder13Val = HW.talon12.GetSelectedSensorPosition();
            coder23Val = HW.talon22.GetSelectedSensorPosition();
            coder33Val = HW.talon32.GetSelectedSensorPosition();

            /*Use wrap handler function to ensure continous rotation and efficient angle finding*/
            coder03Target = WrapHandler((Var.steer02 / 360 * 4096), coder03Val);
            coder13Target = WrapHandler((Var.steer12 / 360 * 4096), coder13Val);
            coder23Target = WrapHandler((Var.steer22 / 360 * 4096), coder23Val);
            coder33Target = WrapHandler((Var.steer32 / 360 * 4096), coder33Val);

            /*Turn to position*/
            HW.talon02.Set(ControlMode.Position, coder03Target/1.25f); /*<--- I have no idea why this scaling by 1/1.25 needs to occur, it just does*/
            HW.talon12.Set(ControlMode.Position, coder13Target/1.25f);
            HW.talon22.Set(ControlMode.Position, coder23Target/1.25f);
            HW.talon32.Set(ControlMode.Position, coder33Target/1.25f);
        }
               
        /*This function handles all of the oddities that occur with the modules' rotation*/
        /* - wraps encoders once they exceed one rotation either direction*/
        /* - define new target if target is over half a rotation away, ensuring the module turns the optimal direction*/
        /* - define a new target and flip motor direction if target is over a quarter a rotation away, ensuring the module never turns more than 90 degrees*/
        public static float WrapHandler(float targetAngleTicks, float coderX3Val)
        {
            if (coderX3Val < 4096 && coderX3Val >= 0) /*If CANcoder is reading value before wrapping: 0-4096, use target angle ticks without modifying*/
            {
                newTargetAngle = targetAngleTicks;
            }
            else /*Else find out how many wraps have occured, which direction they've occurred in, and recalculate target tick position*/
            {
                newTargetAngle = (float)(targetAngleTicks + 4096 * System.Math.Truncate((float)(coderX3Val / 4096)));
            }

            if (newTargetAngle - coderX3Val > 2048) /*If target angle is over half a rotation away from the current angle in the + direction, subtract a rotation from the target*/
            {
                newTargetAngle -= 4096;
            }
            if (newTargetAngle - coderX3Val < -2048) /*If target angle is over half a rotation away from current angle in the - direction, add a rotation to the target*/
            {
                newTargetAngle += 4096;
            }
            else /*If target angle is within half a rotation of current angle, make no adjustment*/
            {
                
            }

            if (System.Math.Abs(newTargetAngle - coderX3Val) > 1024)
            {
                if (coderX3Val == coder03Val)
                {
                    HW.talon01.SetInverted(false);
                }
                if (coderX3Val == coder13Val)
                {
                    HW.talon11.SetInverted(true);
                }
                if (coderX3Val == coder23Val)
                {
                    HW.talon21.SetInverted(false);
                }
                if (coderX3Val == coder33Val)
                {
                    HW.talon31.SetInverted(true);
                }
                return newTargetAngle - (2048 * System.Math.Sign(newTargetAngle - coderX3Val));
            }
            else
            {
                if (coderX3Val == coder03Val)
                {
                    HW.talon01.SetInverted(true);
                }
                if (coderX3Val == coder13Val)
                {
                    HW.talon11.SetInverted(false);
                }
                if (coderX3Val == coder23Val)
                {
                    HW.talon21.SetInverted(true);
                }
                if (coderX3Val == coder33Val)
                {
                    HW.talon31.SetInverted(false);
                }
                return newTargetAngle;

            }
        }
    }
}