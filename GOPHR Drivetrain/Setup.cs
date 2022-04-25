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
using CTRE.Phoenix.Motion;

namespace GOPHR_Drivetrain
{
    public static class HW /*Hardware*/
    {
        /*Create Gamepad*/
        public static CTRE.Phoenix.Controller.GameController myGamepad = new CTRE.Phoenix.Controller.GameController(new CTRE.Phoenix.UsbHostDevice(0));

        /*Create Pigeon IMU*/
        public static PigeonIMU pigeon = new PigeonIMU(00);

        /*Create Drive Motors*/
        public static TalonFX talon01 = new TalonFX(01);
        public static TalonFX talon11 = new TalonFX(11);
        public static TalonFX talon21 = new TalonFX(21);
        public static TalonFX talon31 = new TalonFX(31);

        /*Create Steer Motors*/
        public static TalonFX talon02 = new TalonFX(02);
        public static TalonFX talon12 = new TalonFX(12);
        public static TalonFX talon22 = new TalonFX(22);
        public static TalonFX talon32 = new TalonFX(32);

        /*Create CANcoders*/
        public static CANcoder coder03 = new CANcoder(03);
        public static CANcoder coder13 = new CANcoder(13);
        public static CANcoder coder23 = new CANcoder(23);
        public static CANcoder coder33 = new CANcoder(33);
    }

    public static class Config
    {
        public static void ConfigCoders()
        {
            /*Reset CANcoders to default*/
            HW.coder03.ConfigFactoryDefault();
            HW.coder13.ConfigFactoryDefault();
            HW.coder23.ConfigFactoryDefault();
            HW.coder33.ConfigFactoryDefault();

            /*Create CANcoder configuration*/
            CANcoderConfiguration configCoderAll = new CANcoderConfiguration();
            configCoderAll.unitString = "deg";
            configCoderAll.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            configCoderAll.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

            /*Config all CANcoders to Config*/
            HW.coder03.ConfigAllSettings(configCoderAll);
            HW.coder13.ConfigAllSettings(configCoderAll);
            HW.coder23.ConfigAllSettings(configCoderAll);
            HW.coder33.ConfigAllSettings(configCoderAll);

            /*Apply unique CANcoder offsets based on absolute zeros*/
            HW.coder03.ConfigMagnetOffset(-61.875f);
            HW.coder13.ConfigMagnetOffset(-151.260f);
            HW.coder23.ConfigMagnetOffset(-199.073f);
            HW.coder33.ConfigMagnetOffset(-334.248f);

            /*Clear sticky faults?*/
            HW.coder03.ClearStickyFaults();
            HW.coder13.ClearStickyFaults();
            HW.coder23.ClearStickyFaults();
            HW.coder33.ClearStickyFaults();
        }
        public static void ConfigSteerTalons()
        {
            /*Create Steer Talon configuration */
            TalonFXConfiguration configTalon02 = new TalonFXConfiguration();
            configTalon02.slot_0.kP = 2.0f;
            configTalon02.slot_0.kI = 0.0f;
            configTalon02.slot_0.kD = 0.0f;
            configTalon02.slot_0.kF = 0.5f;
            configTalon02.filter_0.remoteSensorDeviceID = 03;
            configTalon02.filter_0.remoteSensorSource = RemoteSensorSource.RemoteSensorSource_CANCoder;
            configTalon02.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
            configTalon02.slot_0.closedLoopPeakOutput = 0.5f;
            configTalon02.supplyCurrLimit.enable = true;
            configTalon02.supplyCurrLimit.triggerThresholdCurrent = 20; // the peak supply current, in amps
            configTalon02.supplyCurrLimit.triggerThresholdTime = 0.5f; // the time at the peak supply current before the limit triggers, in sec
            configTalon02.supplyCurrLimit.currentLimit = 15;
            configTalon02.feedbackNotContinuous = false;
            configTalon02.neutralDeadband = 0.01f;

            /*Configure Talon02*/
            HW.talon02.ConfigFactoryDefault();
            HW.talon02.ConfigAllSettings(configTalon02);

            /*Adjust Talon12 for unique CANcoder input configuration and configure*/
            TalonFXConfiguration configTalon12 = configTalon02;
            configTalon12.filter_0.remoteSensorDeviceID = 13;
            HW.talon12.ConfigAllSettings(configTalon12);

            /*Adjust Talon22 for unique CANcoder input configuration and configure*/
            TalonFXConfiguration configTalon22 = configTalon02;
            configTalon22.filter_0.remoteSensorDeviceID = 23;
            HW.talon22.ConfigAllSettings(configTalon22);

            /*Adjust Talon32 for unique CANcoder input configuration and configure*/
            TalonFXConfiguration configTalon32 = configTalon02;
            configTalon32.filter_0.remoteSensorDeviceID = 33;
            HW.talon32.ConfigAllSettings(configTalon32);
        }
        public static void ConfigDriveTalons()
        {
            TalonFXConfiguration configTalon01 = new TalonFXConfiguration();
            configTalon01.slot_0.kP = 0.1f;
            configTalon01.slot_0.kI = 0.0001f;
            configTalon01.slot_0.kD = 0.0f;
            configTalon01.slot_0.kF = 0.0f;
            configTalon01.supplyCurrLimit.enable = true;
            configTalon01.supplyCurrLimit.triggerThresholdCurrent = 10; // the peak supply current, in amps
            configTalon01.supplyCurrLimit.triggerThresholdTime = 0.5f; // the time at the peak supply current before the limit triggers, in sec
            configTalon01.slot_0.closedLoopPeakOutput = 1.0f;
            configTalon01.supplyCurrLimit.currentLimit = 10;
            configTalon01.neutralDeadband = 0.01f;
            configTalon01.initializationStrategy = SensorInitializationStrategy.BootToZero;
            configTalon01.peakOutputForward = 1.0f;
            configTalon01.peakOutputReverse = -1.0f;
            configTalon01.motionAcceleration = 3200;
            configTalon01.motionCruiseVelocity = 3200;

            HW.talon01.ConfigFactoryDefault();
            HW.talon01.ConfigAllSettings(configTalon01);
            HW.talon01.ConfigSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
            HW.talon01.SetSelectedSensorPosition(0);

            TalonFXConfiguration configTalon11 = configTalon01;

            HW.talon11.ConfigFactoryDefault();
            HW.talon11.ConfigAllSettings(configTalon11);
            HW.talon11.ConfigSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
            HW.talon11.SetSelectedSensorPosition(0);

            TalonFXConfiguration configTalon21 = configTalon01;

            HW.talon21.ConfigFactoryDefault();
            HW.talon21.ConfigAllSettings(configTalon21);
            HW.talon21.ConfigSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
            HW.talon21.SetSelectedSensorPosition(0);

            TalonFXConfiguration configTalon31 = configTalon01;

            HW.talon31.ConfigFactoryDefault();
            HW.talon31.ConfigAllSettings(configTalon31);
            HW.talon31.ConfigSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
            HW.talon31.SetSelectedSensorPosition(0);

            HW.talon01.SetInverted(false);
            HW.talon11.SetInverted(true);
            HW.talon21.SetInverted(false);
            HW.talon31.SetInverted(true);
        }
        public static float FeetToTicks(float feet)
        {
            return (float)(feet * 12 * 8.14 * 2048 / (3.65 * System.Math.PI));
        }

        public static float DegreesToFeet(float degrees)
        {
            return (float)(degrees * 0.01799778968);
        }

        public static float TicksToFeet(float ticks)
        {
            return (float)(ticks * (3.65 * System.Math.PI) / (12 * 8.14 * 2048));
        }

        public static float MetersToFeet(float meters)
        {
            return (float) meters * 3.28084f;
        }

    }

    public static class Controllers
    {
        readonly private static float maxIntegral = 0.1f;

        private static float curError;
        private static float velError;
        private static float prevError;
        private static float totalError = 0;

        private static DateTime prevTime = System.DateTime.Now;
        private static DateTime curTime = System.DateTime.Now;
        private static TimeSpan span;
        private static int ticks;
        private static float period;

        public static float TurnPidController(float initialAngle, float currentAngle, float targetAngle, float kP, float kI, float kD)
        {
            int turnRamp = 30;
            float output;
            if (System.Math.Abs(targetAngle - currentAngle) < 5)
            {
                /*Make current error from last cycle previous error*/
                prevError = curError;

                /*Make current error setpoint - process variable*/
                curError = targetAngle - currentAngle;

                /*Set current time from last cycle as previous time*/
                prevTime = curTime;

                /*Set current time from system time*/
                curTime = System.DateTime.Now;

                /*Calculate timespan between current and previous time*/
                span = curTime - prevTime;

                /*Convert timespan to integer number of ticks and then to double number of ms*/
                ticks = (int)span.Ticks;
                period = ticks / 10000;

                /*Calculate Velocity error using current and previous errors and period*/
                velError = -(curError - prevError) / period;

                /*If kI !=0, allow kI to build until max kI contribution is reached*/
                if (kI != 0)
                {
                    totalError += curError * period;

                    if (totalError >= maxIntegral / kI)
                    {
                        totalError = maxIntegral / kI;
                    }
                    else if (totalError <= -maxIntegral / kI)
                    {
                        totalError = -maxIntegral / kI;
                    }
                }
                output = (kP * curError + kI * totalError + kD * velError);
                //Debug.Print("PID Output: " + output);
            }
            else if ((currentAngle - initialAngle) / (targetAngle - initialAngle) <= 0.5)
            {
                if (System.Math.Abs(currentAngle - initialAngle) <= turnRamp)
                {
                    output = System.Math.Sign(targetAngle - initialAngle) * 0.2f + (float)System.Math.Abs((currentAngle - initialAngle) * 0.8f / turnRamp) * System.Math.Sign(targetAngle - initialAngle);
                    //Debug.Print("Ramping up");
                }
                else
                {
                    output = System.Math.Sign(targetAngle - initialAngle) * 1;
                    //Debug.Print("Cruise under half");
                }
            }
            else
            {
                if (System.Math.Abs(targetAngle - currentAngle) <= turnRamp)
                {
                    output = System.Math.Sign(targetAngle - initialAngle) * 0.2f + ((targetAngle - currentAngle) * 0.8f / (turnRamp));
                    //Debug.Print("Ramping down");
                }
                else
                {
                    output = System.Math.Sign(targetAngle - initialAngle) * 1;
                    //Debug.Print("cruise over half");
                }
            }
            /*Return PID control value*/
            //Debug.Print("Ramping Output: " + output);
            return -output;
        }
    }

    public static class Var
    {
        public static float targetAngle;
        public static float currentAngle;
        public static float angleError;
        public static float startAngle;

        public static float targetSpeed;

        public static float axis0;
        public static float axis1;
        public static float axis2;

        public static float wheelbase = 17.5f;
        public static float trackwidth = 17.5f;
        public static float radius = (float)(System.Math.Sqrt(wheelbase * wheelbase + trackwidth * trackwidth) / 2);

        public static float drive01;
        public static float drive11;
        public static float drive21;
        public static float drive31;

        public static float steer02;
        public static float steer12;
        public static float steer22;
        public static float steer32;

        public static float omegaTarget;

        public static float maxSpeed = 1600f;

        public static float xCurrent;
        public static float yCurrent;
        public static float omegaCurrent;

        public static float vxOdom;
        public static float vyOdom;
        public static float vOmegaOdom;

        public static float pathProgress;

        public static float[] waypointArray = new float[60];
    }
}

