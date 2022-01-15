/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.Season_Setup;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.Locale;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcIntake;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcServo;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcMotorActuator;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcRevBlinkin;
import TrcFtcLib.ftclib.FtcRobotBattery;
import TrcFtcLib.ftclib.FtcServo;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    // Global objects
    public static final String ROBOT_NAME = "Ducky";
    public FtcOpMode opMode;
    public FtcDashboard dashboard;
    public TrcDbgTrace globalTracer;

    // Vision subsystems
    public Freight_Frenzy_Pipeline vision;

    // Sensors and indicators
    public FtcRevBlinkin blinkin;
    public FtcRobotBattery battery;

    // Subsystems
    public RobotDrive robotDrive;

    public TrcIntake collector;
    public FtcServo armExtender;
    public TrcPidActuator armRotator;
    public TrcPidActuator armPlatformRotator;

    public FtcServo carouselSpinner;
    public TrcPidActuator carouselSpinnerRotator;

    public FtcServo tapeMeasure;


    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *                specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        // Initialize global objects.
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTextView(
            ((FtcRobotControllerActivity)opMode.hardwareMap.appContext)
                .findViewById(com.qualcomm.ftcrobotcontroller.R.id.textOpMode));
        globalTracer = TrcDbgTrace.getGlobalTracer();

        // Voice Telemetry Update
        speak("Init starting");


        // Initialize vision subsystems



        // If visionOnly is true, the robot controller is disconnected from the robot for testing
        // vision.
        // In this case, we should not instantiate any robot hardware.
        if (!RobotParams.Preferences.visionOnly)
        {
            // Create and initialize sensors and indicators.
            if (RobotParams.Preferences.useBlinkin )
            {
//                blinkin = new FtcRevBlinkin(RobotParams.HWNAME_BLINKIN);
//                // Vision uses Blinkin as an indicator, so set it up.
//                if (vision != null)
//                {
//                    vision.setupBlinkin();
//                }
            }

            // Allows the program to access battery voltage information.
            if (RobotParams.Preferences.useBatteryMonitor)
            {
                battery = new FtcRobotBattery();
            }

            // Create and initialize RobotDrive.
            robotDrive = new RobotDrive(this);

            // Create and initialize other subsystems.
            if (RobotParams.Preferences.initSubsystems)
            {

                // Arm System
                if (RobotParams.Preferences.useArmSystem)
                {



                    // Arm Rotator
                    final TrcPidActuator.Parameters armParams = new TrcPidActuator.Parameters()
                            .setPosRange(RobotParams.ARM_ROTATOR_MIN_POS, RobotParams.ARM_ROTATOR_MAX_POS)
                            .setScaleOffset(RobotParams.ARM_ROTATOR_DEG_PER_COUNT, RobotParams.ARM_ROTATOR_OFFSET)
                            .setPidParams(new TrcPidController.PidParameters(
                                    RobotParams.ARM_ROTATOR_KP, RobotParams.ARM_ROTATOR_KI, RobotParams.ARM_ROTATOR_KD, RobotParams.ARM_ROTATOR_TOLERANCE))
                            .setMotorParams(
                                    RobotParams.ARM_ROTATOR_MOTOR_INVERTED,
                                    RobotParams.ARM_ROTATOR_HAS_LOWER_LIMIT_SWITCH, RobotParams.ARM_ROTATOR_LOWER_LIMIT_INVERTED,
                                    RobotParams.ARM_ROTATOR_HAS_UPPER_LIMIT_SWITCH, RobotParams.ARM_ROTATOR_UPPER_LIMIT_INVERTED,
                                    RobotParams.ARM_ROTATOR_CAL_POWER)
                            .setStallProtectionParams(
                                    RobotParams.ARM_ROTATOR_STALL_MIN_POWER, RobotParams.ARM_ROTATOR_STALL_TIMEOUT, RobotParams.ARM_ROTATOR_RESET_TIMEOUT)
                            .setPosPresets(RobotParams.ARM_ROTATOR_PRESET_LEVELS);
                    armRotator = new FtcMotorActuator(RobotParams.HWNAME_ARM_ROTATOR, armParams).getPidActuator();
                    armRotator.setMsgTracer(globalTracer);
                    armRotator.zeroCalibrate();
                }

                // Arm System
                if (RobotParams.Preferences.useDuckSystem)
                {

                }
            }
        }

        // Voice Telemetry Update
        speak("Init complete");
    }   // Robot

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return ROBOT_NAME;
    }   //toString

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        if (robotDrive != null)
        {
            // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to
            // cartesian converter.
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(true);
            }

            // Enable odometry only for autonomous or test modes.
            if (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE)
            {
                robotDrive.setOdometryEnabled(true);
            }
        }


        if (!RobotParams.Preferences.competitionMode)
        {
            // The following are performance counters, could be disabled for competition if you want.
            // But it might give you some insight if somehow autonomous wasn't performing as expected.
            if (robotDrive != null && robotDrive.gyro != null)
            {
                robotDrive.gyro.setElapsedTimerEnabled(true);
            }
            TrcDigitalInput.setElapsedTimerEnabled(true);
            TrcMotor.setElapsedTimerEnabled(true);
            TrcServo.setElapsedTimerEnabled(true);
        }
    }   // startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        final String funcName = "stopMode";

        if (!RobotParams.Preferences.competitionMode) {
            // Print all performance counters if there are any.
            if (robotDrive != null && robotDrive.gyro != null) {
                robotDrive.gyro.printElapsedTime(globalTracer);
                robotDrive.gyro.setElapsedTimerEnabled(false);
            }
            TrcDigitalInput.printElapsedTime(globalTracer);
            TrcDigitalInput.setElapsedTimerEnabled(false);
            TrcMotor.printElapsedTime(globalTracer);
            TrcMotor.setElapsedTimerEnabled(false);
            TrcServo.printElapsedTime(globalTracer);
            TrcServo.setElapsedTimerEnabled(false);
        }

        // Disable vision
//        if (vision != null)
//        {
//            if (RobotParams.Preferences.useVuforia)
//            {
//                globalTracer.traceInfo(funcName, "Disabling Vuforia.");
//                vision.setVuforiaEnabled(false);
//            }
//
//            if (RobotParams.Preferences.useTensorFlow)
//            {
//                globalTracer.traceInfo(funcName, "Shutting down TensorFlow.");
//                vision.tensorFlowShutdown();
//            }
//        }

        if (robotDrive != null)
        {
            // Disable odometry
            robotDrive.setOdometryEnabled(false);

            // Disable gyro task
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(false);
            }
        }
    }   // stopMode

    /**
     * This method is typically called in the autonomous state machine to log the autonomous state info as a state
     * event in the trace log file. The logged event can be used to play back autonomous path movement.
     *
     * @param state specifies the current state of the state machine.
     */
    public void traceStateInfo(Object state)
    {
        final String funcName = "traceStateInfo";

        if (robotDrive != null)
        {
            StringBuilder msg = new StringBuilder();

            msg.append(String.format(Locale.US, "tag=\">>>>>\" state=\"%s\"", state));
            if (robotDrive.pidDrive.isActive())
            {
                TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();
                TrcPose2D targetPose = robotDrive.pidDrive.getAbsoluteTargetPose();
                msg.append(" RobotPose=" + robotPose + " TargetPose=" + targetPose);
            }
            else if (robotDrive.purePursuitDrive.isActive())
            {
                TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();
                TrcPose2D robotVel = robotDrive.driveBase.getFieldVelocity();
                TrcPose2D targetPose = robotDrive.purePursuitDrive.getTargetFieldPosition();
                msg.append(" RobotPose=" + robotPose +
                        " TargetPose=" + targetPose +
                        " vel=" + robotVel +
                        " Path=" + robotDrive.purePursuitDrive.getPath());
            }

            if (battery != null)
            {
                msg.append(String.format(
                        Locale.US, " volt=\"%.2fV(%.2fV)\"", battery.getVoltage(), battery.getLowestVoltage()));
            }

            globalTracer.logEvent(funcName, "StateInfo", "%s", msg);
        }
    }   // traceStateInfo

    /**
     * This method sends the text string to the Driver Station to be spoken using text to speech.
     *
     * @param sentence specifies the sentence to be spoken by the Driver Station.
     */
    public void speak(String sentence)
    {
        opMode.telemetry.speak(sentence);
    }   //speak

}   // class Robot
