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

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Locale;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
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
    public static boolean isRedAlliance;

    // Vision subsystems
    public Freight_Frenzy_Pipeline vision;
    public OpenCvCamera webcam;
    public int cameraMonitorViewId;

    // Sensors and indicators
//    public FtcRevBlinkin blinkin;
    public FtcRobotBattery battery;

    // Subsystems
    public RobotDrive robotDrive;

    public FtcServo collector;
    public TrcPidActuator armExtender;
    public TrcPidActuator armRotator;
    public TrcPidActuator armPlatformRotator;

    public FtcServo carouselSpinner;
    public FtcServo carouselExtenderOne; // Extender closer to center of robot
    public FtcServo carouselExtenderTwo; // Extender at outer edge of robot


    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *                specific sensors and subsystems if necessary.
     */
    @SuppressWarnings("unused")
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
        if ((RobotParams.Preferences.useVision) &&
                (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            vision = new Freight_Frenzy_Pipeline();

            cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM), cameraMonitorViewId);
            webcam.setPipeline(vision);

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        }


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
                    // Collector
                    collector = new FtcServo(RobotParams.HWNAME_COLLECTOR);
                    collector.setPosition(RobotParams.COLLECTOR_STOP_POWER);

                    // Arm Extender
                    final FtcMotorActuator.MotorParams armExtenderMotorParams = new FtcMotorActuator.MotorParams(
                            RobotParams.ARM_EXTENDER_MOTOR_INVERTED,
                            RobotParams.ARM_EXTENDER_HAS_LOWER_LIMIT_SWITCH, RobotParams.ARM_EXTENDER_LOWER_LIMIT_INVERTED,
                            RobotParams.ARM_EXTENDER_HAS_UPPER_LIMIT_SWITCH, RobotParams.ARM_EXTENDER_UPPER_LIMIT_INVERTED);
                    final TrcPidActuator.Parameters armExtenderParams = new TrcPidActuator.Parameters()
                            .setPosRange(RobotParams.ARM_EXTENDER_MIN_POS, RobotParams.ARM_EXTENDER_MAX_POS)
                            .setScaleOffset(RobotParams.ARM_EXTENDER_INCHES_PER_COUNT, RobotParams.ARM_EXTENDER_OFFSET)
                            .setPidParams(new TrcPidController.PidParameters(
                                    RobotParams.ARM_EXTENDER_KP, RobotParams.ARM_EXTENDER_KI, RobotParams.ARM_EXTENDER_KD, RobotParams.ARM_EXTENDER_TOLERANCE))
                            .setStallProtectionParams(
                                    RobotParams.ARM_EXTENDER_STALL_MIN_POWER, RobotParams.ARM_EXTENDER_STALL_TOLERANCE,
                                    RobotParams.ARM_EXTENDER_STALL_TIMEOUT, RobotParams.ARM_EXTENDER_RESET_TIMEOUT)
                            .setZeroCalibratePower(RobotParams.ARM_EXTENDER_CAL_POWER)
                            .setPosPresets(RobotParams.ARM_EXTENDER_PRESET_LENGTH);
                    armExtender = new FtcMotorActuator(RobotParams.HWNAME_ARM_EXTENDER, armExtenderMotorParams, armExtenderParams).getPidActuator();
                    armExtender.setMsgTracer(globalTracer);
                    armExtender.zeroCalibrate();

                    // Arm Rotator
                    final FtcMotorActuator.MotorParams armRotatorMotorParams = new FtcMotorActuator.MotorParams(
                            RobotParams.ARM_ROTATOR_MOTOR_INVERTED,
                            RobotParams.ARM_ROTATOR_HAS_LOWER_LIMIT_SWITCH, RobotParams.ARM_ROTATOR_LOWER_LIMIT_INVERTED,
                            RobotParams.ARM_ROTATOR_HAS_UPPER_LIMIT_SWITCH, RobotParams.ARM_ROTATOR_UPPER_LIMIT_INVERTED);
                    final TrcPidActuator.Parameters armRotatorParams = new TrcPidActuator.Parameters()
                            .setPosRange(RobotParams.ARM_ROTATOR_MIN_POS, RobotParams.ARM_ROTATOR_MAX_POS)
                            .setScaleOffset(RobotParams.ARM_ROTATOR_DEG_PER_COUNT, RobotParams.ARM_ROTATOR_OFFSET)
                            .setPidParams(new TrcPidController.PidParameters(
                                    RobotParams.ARM_ROTATOR_KP, RobotParams.ARM_ROTATOR_KI, RobotParams.ARM_ROTATOR_KD, RobotParams.ARM_ROTATOR_TOLERANCE))
                            .setPowerCompensation(this::gravityCompensation)
                            .setStallProtectionParams(
                                    RobotParams.ARM_ROTATOR_STALL_MIN_POWER, RobotParams.ARM_ROTATOR_STALL_TOLERANCE,
                                    RobotParams.ARM_ROTATOR_STALL_TIMEOUT, RobotParams.ARM_ROTATOR_RESET_TIMEOUT)
                            .setZeroCalibratePower(RobotParams.ARM_ROTATOR_CAL_POWER)
                            .setPosPresets(RobotParams.ARM_ROTATOR_PRESET_LEVELS);
                    armRotator = new FtcMotorActuator(RobotParams.HWNAME_ARM_ROTATOR, armRotatorMotorParams, armRotatorParams).getPidActuator();
                    armRotator.getPidController().setOutputLimit(0.5);
                    armRotator.setMsgTracer(globalTracer);

                    if(runMode == TrcRobot.RunMode.TELEOP_MODE)
                    {
                        armRotator.setPositionScale(RobotParams.ARM_ROTATOR_DEG_PER_COUNT, RobotParams.ARM_ROTATOR_OFFSET_TELEOP);
                    }
                    armRotator.zeroCalibrate();

                    // Arm Platform Rotator
                    final FtcMotorActuator.MotorParams armPlatformRotatorMotorParams = new FtcMotorActuator.MotorParams(
                            RobotParams.ARM_PLATFORM_ROTATOR_MOTOR_INVERTED,
                            RobotParams.ARM_PLATFORM_ROTATOR_HAS_LOWER_LIMIT_SWITCH, RobotParams.ARM_PLATFORM_ROTATOR_LOWER_LIMIT_INVERTED,
                            RobotParams.ARM_PLATFORM_ROTATOR_HAS_UPPER_LIMIT_SWITCH, RobotParams.ARM_PLATFORM_ROTATOR_UPPER_LIMIT_INVERTED);
                    final TrcPidActuator.Parameters armPlatformRotatorParams = new TrcPidActuator.Parameters()
                            .setPosRange(RobotParams.ARM_PLATFORM_ROTATOR_MIN_POS, RobotParams.ARM_PLATFORM_ROTATOR_MAX_POS)
                            .setScaleOffset(RobotParams.ARM_PLATFORM_ROTATOR_DEG_PER_COUNT, RobotParams.ARM_PLATFORM_ROTATOR_OFFSET)
                            .setPidParams(new TrcPidController.PidParameters(
                                    RobotParams.ARM_PLATFORM_ROTATOR_KP, RobotParams.ARM_PLATFORM_ROTATOR_KI, RobotParams.ARM_PLATFORM_ROTATOR_KD, RobotParams.ARM_PLATFORM_ROTATOR_TOLERANCE))
                            .setStallProtectionParams(
                                    RobotParams.ARM_PLATFORM_ROTATOR_STALL_MIN_POWER, RobotParams.ARM_PLATFORM_ROTATOR_STALL_TOLERANCE,
                                    RobotParams.ARM_PLATFORM_ROTATOR_STALL_TIMEOUT, RobotParams.ARM_PLATFORM_ROTATOR_RESET_TIMEOUT)
                            .setZeroCalibratePower(RobotParams.ARM_PLATFORM_ROTATOR_CAL_POWER)
                            .setPosPresets(RobotParams.ARM_PLATFORM_ROTATOR_PRESET_LEVELS);
                    armPlatformRotator = new FtcMotorActuator(RobotParams.HWNAME_ARM_PLATFORM_ROTATOR, armPlatformRotatorMotorParams, armPlatformRotatorParams).getPidActuator();
                    armPlatformRotator.getPidController().setOutputLimit(0.5);
                    armPlatformRotator.setMsgTracer(globalTracer);
                    if(runMode == TrcRobot.RunMode.TELEOP_MODE)
                    {
                        armPlatformRotator.setPositionScale(RobotParams.ARM_PLATFORM_ROTATOR_DEG_PER_COUNT, RobotParams.ARM_PLATFORM_ROTATOR_OFFSET_TELEOP);
                    }
                    armPlatformRotator.zeroCalibrate();
                }

                // Duck System
                if (RobotParams.Preferences.useDuckSystem)
                {
                    // Carousel Spinner
                    carouselSpinner = new FtcServo(RobotParams.HWNAME_CAROUSEL_SPINNER);
                    carouselSpinner.setPosition(RobotParams.CAROUSEL_SPINNER_STOP_POWER);

                    // Carousel Extender One
                    carouselExtenderOne = new FtcServo(RobotParams.HWNAME_CAROUSEL_SPINNER_EXTENDER_ONE);
                    carouselExtenderOne.setPosition(RobotParams.CAROUSEL_SPINNER_STOP_POWER);

                    // Carousel Spinner
                    carouselExtenderTwo = new FtcServo(RobotParams.HWNAME_CAROUSEL_SPINNER_EXTENDER_TWO);
                    carouselExtenderTwo.setPosition(RobotParams.CAROUSEL_SPINNER_STOP_POWER);
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
    @SuppressWarnings("unused")
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
    @SuppressWarnings("unused")
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
                msg.append(" RobotPose=").append(robotPose).append(" TargetPose=").append(targetPose);
            }
            else if (robotDrive.purePursuitDrive.isActive())
            {
                TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();
                TrcPose2D robotVel = robotDrive.driveBase.getFieldVelocity();
                TrcPose2D targetPose = robotDrive.purePursuitDrive.getTargetFieldPosition();
                msg.append(" RobotPose=").append(robotPose).append(" TargetPose=").append(targetPose)
                        .append(" vel=").append(robotVel).append(" Path=").append(robotDrive.purePursuitDrive.getPath());
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
        if (RobotParams.Preferences.speakEnabled)
        {
            opMode.telemetry.speak(sentence);
        }
    }   // speak

    /**
     * This method allows the armRotator to hold its position in the air at all times.
     * @return The gravity compensation for the armRotator depending on the angle.
     */
    private double gravityCompensation ()
    {
        return RobotParams.ARM_ROTATOR_MAX_GRAVITY_COMPENSATION * Math.sin(Math.toRadians(armRotator.getPosition()));
    }

}   // class Robot
