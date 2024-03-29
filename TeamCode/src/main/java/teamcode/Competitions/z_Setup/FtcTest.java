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

package teamcode.Competitions.z_Setup;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import TrcCommonLib.command.CmdDriveMotorsTest;
import TrcCommonLib.command.CmdPidDrive;
import TrcCommonLib.command.CmdPurePursuitDrive;
import TrcCommonLib.command.CmdTimedDrive;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcElapsedTimer;
import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcGyro;
import TrcCommonLib.trclib.TrcPath;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcChoiceMenu;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcMenu;
import TrcFtcLib.ftclib.FtcPidCoeffCache;
import TrcFtcLib.ftclib.FtcTensorFlow;
import TrcFtcLib.ftclib.FtcValueMenu;
import teamcode.Competitions.TeleOp.FtcTeleOp;
import teamcode.Season_Setup.Robot;
import teamcode.Season_Setup.RobotParams;


/**
 * This class contains the Test Mode program.
 */
@TeleOp(name="FtcTest", group="Setup")

public class FtcTest extends FtcTeleOp
{
    private static final boolean logEvents = true;
    private static final boolean debugPid = true;

    private enum Test
    {
        SENSORS_TEST,
        SUBSYSTEMS_TEST,
        DRIVE_SPEED_TEST,
        DRIVE_MOTORS_TEST,
        Y_TIMED_DRIVE,
        PID_DRIVE,
        TUNE_X_PID,
        TUNE_Y_PID,
        TUNE_TURN_PID,
        PURE_PURSUIT_DRIVE,
        X_TIMED_DRIVE
    }   // enum Test

    /**
     * This class stores the test menu choices.
     */
    private static class TestChoices
    {
        Test test = Test.SENSORS_TEST;
        double xTarget = 0.0;
        double yTarget = 0.0;
        double turnTarget = 0.0;
        double driveTime = 0.0;
        double drivePower = 0.0;
        TrcPidController.PidCoefficients tunePidCoeff = null;
        double tuneDistance = 0.0;
        double tuneHeading = 0.0;
        double tuneDrivePower = 0.0;

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "test=\"%s\" " +
                "xTarget=%.1f " +
                "yTarget=%.1f " +
                "turnTarget=%1f " +
                "driveTime=%.1f " +
                "drivePower=%.1f " +
                "tunePidCoeff=%s " +
                "tuneDistance=%.1f " +
                "tuneHeading=%.1f " +
                "tuneDrivePower=%.1f",
                test, xTarget, yTarget, turnTarget, driveTime, drivePower, tunePidCoeff, tuneDistance, tuneHeading,
                tuneDrivePower);
        }   // toString
    }   // class TestChoices

    private final FtcPidCoeffCache pidCoeffCache =
        new FtcPidCoeffCache("PIDTuning", RobotParams.LOG_PATH_FOLDER);
    private final TestChoices testChoices = new TestChoices();
    private TrcElapsedTimer elapsedTimer = null;
    private FtcChoiceMenu<Test> testMenu = null;

    private TrcRobot.RobotCommand testCommand = null;
    private double maxDriveVelocity = 0.0;
    private double maxDriveAcceleration = 0.0;
    private double prevTime = 0.0;
    private double prevVelocity = 0.0;


    // Overrides FtcOpMode abstract method.

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void initRobot()
    {

        // TeleOp initialization.
        super.initRobot();
        if (RobotParams.Preferences.useLoopPerformanceMonitor)
        {
            elapsedTimer = new TrcElapsedTimer("TestLoopMonitor", 2.0);
        }

        // Test menus.
        doTestMenus();

        // Create the robot command for the tests that need one.
        switch (testChoices.test)
        {
            case DRIVE_MOTORS_TEST:
                if (!RobotParams.Preferences.visionOnly)
                {
                    testCommand = new CmdDriveMotorsTest(
                        new FtcDcMotor[] {robot.robotDrive.leftWheels, robot.robotDrive.rightWheels},
                        5.0, 0.5);
                }
                break;

            case X_TIMED_DRIVE:
                if (!RobotParams.Preferences.visionOnly)
                {
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.driveTime,
                        testChoices.drivePower, 0.0, 0.0);
                }
                break;

            case Y_TIMED_DRIVE:
                if (!RobotParams.Preferences.visionOnly)
                {
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.driveTime,
                        0.0, testChoices.drivePower, 0.0);
                }
                break;

            case PID_DRIVE:
                if (!RobotParams.Preferences.visionOnly)
                {
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.drivePower, null,
                        new TrcPose2D(testChoices.xTarget*12.0, testChoices.yTarget*12.0, testChoices.turnTarget));
                }
                break;

            case TUNE_X_PID:
                if (!RobotParams.Preferences.visionOnly)
                {
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.tuneDrivePower,
                        testChoices.tunePidCoeff, new TrcPose2D(testChoices.tuneDistance*12.0, 0.0, 0.0));
                }
                break;

            case TUNE_Y_PID:
                if (!RobotParams.Preferences.visionOnly)
                {
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.tuneDrivePower,
                        testChoices.tunePidCoeff, new TrcPose2D(0.0, testChoices.tuneDistance*12.0, 0.0));
                }
                break;

            case TUNE_TURN_PID:
                if (!RobotParams.Preferences.visionOnly)
                {
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.tuneDrivePower,
                        testChoices.tunePidCoeff, new TrcPose2D(0.0, 0.0, testChoices.tuneHeading));
                }
                break;

            case PURE_PURSUIT_DRIVE:
                if (!RobotParams.Preferences.visionOnly)
                {
                    testCommand = new CmdPurePursuitDrive(
                        robot.robotDrive.driveBase, null, robot.robotDrive.yPosPidCoeff,
                        robot.robotDrive.turnPidCoeff, robot.robotDrive.velPidCoeff);
                }
                break;
        }

//        // Only SENSORS_TEST and SUBSYSTEMS_TEST need TensorFlow, shut it down for all other tests.
//        //
//        if (robot.vision != null && RobotParams.Preferences.useTensorFlow &&
//            testChoices.test != Test.SENSORS_TEST && testChoices.test != Test.SUBSYSTEMS_TEST)
//        {
//            robot.globalTracer.traceInfo("TestInit", "Shutting down TensorFlow.");
//            robot.vision.tensorFlowShutdown();
//        }
    }   // initRobot

    // Overrides TrcRobot.RobotMode methods.

    /**
     * This method is called before test mode is about to start so it can initialize appropriate subsystems for the
     * test.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        @SuppressWarnings("unused")
        final String funcName = "startMode";

        super.startMode(prevMode, nextMode);
        switch (testChoices.test)
        {
            case SENSORS_TEST:
            case SUBSYSTEMS_TEST:
                if (robot.vision != null)
                {
//                    //
//                    // Vision generally will impact performance, so we only enable it if it's needed.
//                    //
//                    if (RobotParams.Preferences.useVision)
//                    {
//                        robot.globalTracer.traceInfo(funcName, "Enabling Vuforia.");
//                        robot.vision.setVuforiaEnabled(true);
//                    }
//
//                    if (RobotParams.Preferences.useTensorFlow)
//                    {
//                        robot.globalTracer.traceInfo(funcName, "Enabling TensorFlow.");
//                        robot.vision.setTensorFlowEnabled(true);
//                    }
                }
                break;

            case TUNE_TURN_PID:
                robot.robotDrive.pidDrive.setMsgTracer(robot.globalTracer, logEvents, debugPid);
                break;

            case PURE_PURSUIT_DRIVE:
                robot.robotDrive.purePursuitDrive.setMsgTracer(robot.globalTracer, logEvents, debugPid);

                // Set the current position as the absolute field origin so the path can be an absolute path.
                robot.robotDrive.driveBase.setFieldPosition(new TrcPose2D(0.0, 0.0, 0.0));

                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);

                // Turn 45 degrees to the Left
//                robot.robotDrive.purePursuitDrive.start(
//                        robot.robotDrive.driveBase.getFieldPosition(), true,
//                        new TrcPose2D(0.0,12.0, -15.0),
//                        new TrcPose2D(0.0,12.0, -15.0),
//                        new TrcPose2D(0.0,12.0, -15.0),
//                        new TrcPose2D(0.0,12.0, -15.0));

//                // Turn 45 degrees to the Right
//                robot.robotDrive.purePursuitDrive.start(
//                        robot.robotDrive.driveBase.getFieldPosition(), true,
//                        new TrcPose2D(0.0,12.0, 15.0),
//                        new TrcPose2D(0.0,12.0, 15.0),
//                        new TrcPose2D(0.0,12.0, 15.0),
//                        new TrcPose2D(0.0,12.0, 15.0));

                // Turn 45 degrees to the Right (smaller increments)
//                robot.robotDrive.purePursuitDrive.start(
//                        robot.robotDrive.driveBase.getFieldPosition(), true,
//                        new TrcPose2D(0.0,5.0, 15.0),
//                        new TrcPose2D(0.0,5.0, 15.0),
//                        new TrcPose2D(0.0,5.0, 15.0),
//                        new TrcPose2D(0.0,5.0, 15.0));

                // Pure Pursuit Path
//                robot.robotDrive.purePursuitDrive.start(
//                        robot.robotDrive.driveBase.getFieldPosition(), false,
//                        new TrcPose2D(12.0, 24.0,45.0),
//                        new TrcPose2D(-12.0, 36.0, -90.0));

                // Create and run Incremental Path
//                TrcPath path = robot.robotDrive.buildPath(
//                        15.0, false,
//                        robot.robotDrive.pathPoint(0.0, 2.0, 90.0),
//                        robot.robotDrive.pathPoint(1.0, 2.0, 45.0));
//                TrcDbgTrace.getGlobalTracer().traceInfo(funcName,">>>>>>>>>>>> Path = %s", path);
//                robot.robotDrive.purePursuitDrive.start(path, null, 10.0);

                // Pure Pursuit Path Point
                robot.robotDrive.purePursuitDrive.start(
                        robot.robotDrive.driveBase.getFieldPosition(), false,
                        robot.robotDrive.pathPoint(0.0, 2.0, 90.0),
                        robot.robotDrive.pathPoint(1.0, 2.0, 90.0));
                break;
        }
    }   // startMode

    /**
     * This method is called before test mode is about to exit so it can do appropriate cleanup.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        final String funcName = "stopMode";

        if (testCommand != null)
        {
            testCommand.cancel();
        }

        if (robot.vision != null)
        {
//            // Vision generally will impact performance, so we only enable it if it's needed.
//            //
//            if (RobotParams.Preferences.useVuforia)
//            {
//                robot.globalTracer.traceInfo(funcName, "Disabling Vuforia.");
//                robot.vision.setVuforiaEnabled(false);
//            }
//
//            if (RobotParams.Preferences.useTensorFlow)
//            {
//                robot.globalTracer.traceInfo(funcName, "Shutting down TensorFlow.");
//                robot.vision.tensorFlowShutdown();
//            }
        }

        super.stopMode(prevMode, nextMode);
    }   // stopMode

    /**
     * This method is called periodically during test mode to perform low frequency tasks such as teleop control or
     * displaying status or test results.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runPeriodic(double elapsedTime)
    {
        if (robot.robotDrive.pidDrive.isActive())
        {
//            robot.robotDrive.pidDrive.getYPidCtrl().printPidInfo(robot.globalTracer, true);
            robot.robotDrive.pidDrive.getTurnPidCtrl().printPidInfo(robot.globalTracer, true);

        }

        if (allowTeleOp())
        {
            // Allow TeleOp to run so we can control the robot in subsystem test or drive speed test modes.
            super.runPeriodic(elapsedTime);
        }

        switch (testChoices.test)
        {
            case SENSORS_TEST:
            case SUBSYSTEMS_TEST:
                doSensorsTest();
                doVisionTest();
                break;
        }
    }   // runPeriodic

    /**
     * This method is called continuously during test mode to execute the test command.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runContinuous(double elapsedTime)
    {
        // Run the testCommand if any.
        if (testCommand != null)
        {
            testCommand.cmdPeriodic(elapsedTime);
        }

        // Display test status.
        switch (testChoices.test)
        {
            case DRIVE_SPEED_TEST:
                if (!RobotParams.Preferences.visionOnly)
                {
                    double currTime = TrcUtil.getCurrentTime();
                    TrcPose2D velPose = robot.robotDrive.driveBase.getFieldVelocity();
                    double velocity = TrcUtil.magnitude(velPose.x, velPose.y);
                    double acceleration = 0.0;

                    if (prevTime != 0.0)
                    {
                        acceleration = (velocity - prevVelocity)/(currTime - prevTime);
                    }

                    if (velocity > maxDriveVelocity)
                    {
                        maxDriveVelocity = velocity;
                    }

                    if (acceleration > maxDriveAcceleration)
                    {
                        maxDriveAcceleration = acceleration;
                    }

                    prevTime = currTime;
                    prevVelocity = velocity;

                    robot.dashboard.displayPrintf(8, "Drive Vel: (%.1f/%.1f)", velocity, maxDriveVelocity);
                    robot.dashboard.displayPrintf(9, "Drive Accel: (%.1f/%.1f)", acceleration, maxDriveAcceleration);
                }
                break;

            case Y_TIMED_DRIVE:
                if (!RobotParams.Preferences.visionOnly)
                {
                    robot.dashboard.displayPrintf(8, "Timed Drive: %.0f sec", testChoices.driveTime);
                    robot.dashboard.displayPrintf(
                        9, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                        robot.robotDrive.driveBase.getXPosition(), robot.robotDrive.driveBase.getYPosition(),
                        robot.robotDrive.driveBase.getHeading());
                    robot.dashboard.displayPrintf(
                        10, "raw=left:%.0f,right:%.0f",
                        robot.robotDrive.leftWheels.getPosition(), robot.robotDrive.rightWheels.getPosition());
                }
                break;

            case TUNE_Y_PID:
            case TUNE_TURN_PID:
                if (!RobotParams.Preferences.visionOnly && testChoices.tunePidCoeff != null)
                {
                    robot.dashboard.displayPrintf(7, "TunePid=%s", testChoices.tunePidCoeff);
                }
                //
                // Intentionally falling through.
                //
            case PID_DRIVE:
                if (!RobotParams.Preferences.visionOnly)
                {
                    robot.dashboard.displayPrintf(
                        8, "xPos=%.1f,yPos=%.1f,heading=%.1f,raw=lw:%.0f,rw:%.0f",
                        robot.robotDrive.driveBase.getXPosition(), robot.robotDrive.driveBase.getYPosition(),
                        robot.robotDrive.driveBase.getHeading(),
                        robot.robotDrive.leftWheels.getPosition(), robot.robotDrive.rightWheels.getPosition());
                    if (robot.robotDrive.encoderYPidCtrl != null)
                    {
                        robot.robotDrive.encoderYPidCtrl.displayPidInfo(11);
                    }
                    if (robot.robotDrive.gyroPidCtrl != null)
                    {
                        robot.robotDrive.gyroPidCtrl.displayPidInfo(13);
                    }
                }
                break;

            case PURE_PURSUIT_DRIVE:
                if (!RobotParams.Preferences.visionOnly)
                {
                    robot.dashboard.displayPrintf(
                        8, "xPos=%.1f,yPos=%.1f,heading=%.1f,rawEnc=left:%.0f,right:%.0f",
                        robot.robotDrive.driveBase.getXPosition(), robot.robotDrive.driveBase.getYPosition(),
                        robot.robotDrive.driveBase.getHeading(),
                        robot.robotDrive.leftWheels.getPosition(), robot.robotDrive.rightWheels.getPosition());
                }
                break;
        }

        if (elapsedTimer != null)
        {
            elapsedTimer.recordPeriodTime();
            robot.dashboard.displayPrintf(
                15, "Period: %.3f(%.3f/%.3f)",
                elapsedTimer.getAverageElapsedTime(), elapsedTimer.getMinElapsedTime(),
                elapsedTimer.getMaxElapsedTime());
        }
    }   // runContinuous


    // Overrides TrcGameController.ButtonHandler in TeleOp.

    /**
     * This method is called when a driver gamepad button event occurs.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    public void driverButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (allowTeleOp())
        {
            boolean processed = false;

            // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
            // FtcTeleOp gamepad actions.

            robot.dashboard.displayPrintf(
                7, "%s: %04x->%s", gamepad, button, pressed ? "Pressed" : "Released");
            switch (button)
            {
                case FtcGamepad.GAMEPAD_DPAD_UP:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;
            }

            // If the control was not processed by this method, pass it back to TeleOp.

            if (!processed)
            {
                super.driverButtonEvent(gamepad, button, pressed);
            }
        }
    }   // driverButtonEvent

    /**
     * This method is called when an operator gamepad button event occurs.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    public void operatorButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (allowTeleOp())
        {
            boolean processed = false;

            // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
            // FtcTeleOp gamepad actions.

            robot.dashboard.displayPrintf(
                7, "%s: %04x->%s", gamepad, button, pressed ? "Pressed" : "Released");
            switch (button)
            {
                case FtcGamepad.GAMEPAD_DPAD_UP:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;
            }

            // If the control was not processed by this method, pass it back to TeleOp.

            if (!processed)
            {
                super.operatorButtonEvent(gamepad, button, pressed);
            }
        }
    }   // operatorButtonEvent

    /**
     * This method creates and displays the test menus and record the selected choices.
     */
    private void doTestMenus()
    {
        // Create menus.

        testMenu = new FtcChoiceMenu<>("Tests:", null);
        FtcValueMenu xTargetMenu = new FtcValueMenu(
            "xTarget:", testMenu, -10.0, 10.0, 0.5, 0.0, " %.1f ft");
        FtcValueMenu yTargetMenu = new FtcValueMenu(
            "yTarget:", testMenu, -10.0, 10.0, 0.5, 8.0, " %.1f ft");
        FtcValueMenu turnTargetMenu = new FtcValueMenu(
            "turnTarget:", testMenu, -180.0, 180.0, 5.0, 0.0, " %.0f deg");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
            "Drive time:", testMenu, 1.0, 10.0, 1.0, 4.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
            "Drive power:", testMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");

        // PID Tuning menus.

        FtcValueMenu tuneKpMenu = new FtcValueMenu(
            "Kp:", testMenu, 0.0, 1.0, 0.001, this::getTuneKp, " %f");
        FtcValueMenu tuneKiMenu = new FtcValueMenu(
            "Ki:", tuneKpMenu, 0.0, 1.0, 0.001, this::getTuneKi, " %f");
        FtcValueMenu tuneKdMenu = new FtcValueMenu(
            "Kd:", tuneKiMenu, 0.0, 1.0, 0.001, this::getTuneKd, " %f");
        FtcValueMenu tuneKfMenu = new FtcValueMenu(
            "Kf:", tuneKdMenu, 0.0, 1.0, 0.001, this::getTuneKf, " %f");
        FtcValueMenu tuneDistanceMenu = new FtcValueMenu(
            "PID Tune distance:", tuneKfMenu, -10.0, 10.0, 0.5, 8.0,
            " %.1f ft");
        FtcValueMenu tuneHeadingMenu = new FtcValueMenu(
            "PID Tune heading:", tuneDistanceMenu, -180.0, 180.0, 5.0, 0.0,
            " %.0f deg");
        FtcValueMenu tuneDrivePowerMenu = new FtcValueMenu(
            "PID Tune drive power:", tuneHeadingMenu, -1.0, 1.0, 0.1, 0.5,
            " %.1f");

        // Populate menus.

        testMenu.addChoice("Sensors test", Test.SENSORS_TEST, true);
        testMenu.addChoice("Subsystems test", Test.SUBSYSTEMS_TEST, false);
        testMenu.addChoice("Drive speed test", Test.DRIVE_SPEED_TEST, false);
        testMenu.addChoice("Drive motors test", Test.DRIVE_MOTORS_TEST, false);
        testMenu.addChoice("X Timed drive", Test.X_TIMED_DRIVE, false, driveTimeMenu);
        testMenu.addChoice("Y Timed drive", Test.Y_TIMED_DRIVE, false, driveTimeMenu);
        testMenu.addChoice("PID drive", Test.PID_DRIVE, false, xTargetMenu);
        testMenu.addChoice("Tune Y PID", Test.TUNE_Y_PID, false, tuneKpMenu);
        testMenu.addChoice("Tune Turn PID", Test.TUNE_TURN_PID, false, tuneKpMenu);
        testMenu.addChoice("Pure Pursuit Drive", Test.PURE_PURSUIT_DRIVE, false);

        xTargetMenu.setChildMenu(yTargetMenu);
        yTargetMenu.setChildMenu(turnTargetMenu);
        turnTargetMenu.setChildMenu(drivePowerMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        tuneKpMenu.setChildMenu(tuneKiMenu);
        tuneKiMenu.setChildMenu(tuneKdMenu);
        tuneKdMenu.setChildMenu(tuneKfMenu);
        tuneKfMenu.setChildMenu(tuneDistanceMenu);
        tuneDistanceMenu.setChildMenu(tuneHeadingMenu);
        tuneHeadingMenu.setChildMenu(tuneDrivePowerMenu);

        // Traverse menus.

        FtcMenu.walkMenuTree(testMenu);

        // Fetch choices.

        testChoices.test = testMenu.getCurrentChoiceObject();
        testChoices.xTarget = xTargetMenu.getCurrentValue();
        testChoices.yTarget = yTargetMenu.getCurrentValue();
        testChoices.turnTarget = turnTargetMenu.getCurrentValue();
        testChoices.driveTime = driveTimeMenu.getCurrentValue();
        testChoices.drivePower = drivePowerMenu.getCurrentValue();
        testChoices.tunePidCoeff = new TrcPidController.PidCoefficients(
            tuneKpMenu.getCurrentValue(), tuneKiMenu.getCurrentValue(),
            tuneKdMenu.getCurrentValue(),tuneKfMenu.getCurrentValue());
        testChoices.tuneDistance = tuneDistanceMenu.getCurrentValue();
        testChoices.tuneHeading = tuneHeadingMenu.getCurrentValue();
        testChoices.tuneDrivePower = tuneDrivePowerMenu.getCurrentValue();

        TrcPidController tunePidCtrl = getTunePidController(testChoices.test);
        if (tunePidCtrl != null)
        {
            // Write the user input PID coefficients to a cache file so tune PID menu can read them as start value
            // next time.
            pidCoeffCache.writeCachedPidCoeff(tunePidCtrl, testChoices.tunePidCoeff);
        }

        // Show choices.
        robot.dashboard.displayPrintf(0, "Test Choices: %s", testChoices);
    }   // doTestMenus

    /**
     * This method returns the PID controller for the tune test.
     *
     * @param test specifies the selected test.
     * @return tune PID controller.
     */
    private TrcPidController getTunePidController(Test test)
    {
        TrcPidController pidCtrl;

        switch (test)
        {
            case TUNE_Y_PID:
                pidCtrl = robot.robotDrive.encoderYPidCtrl;
                break;

            case TUNE_TURN_PID:
                pidCtrl = robot.robotDrive.gyroPidCtrl;
                break;

            default:
                pidCtrl = null;
        }

        return pidCtrl;
    }   // getTunePidController

    /**
     * This method is called by the tuneKpMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Kp value of the PID controller being tuned.
     */
    private double getTuneKp()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kP;
        }

        return value;
    }   // getTuneKp

    /**
     * This method is called by the tuneKiMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Ki value of the PID controller being tuned.
     */
    private double getTuneKi()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kI;
        }

        return value;
    }   // getTuneKi

    /**
     * This method is called by the tuneKdMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Kd value of the PID controller being tuned.
     */
    private double getTuneKd()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kD;
        }

        return value;
    }   // getTuneKd

    /**
     * This method is called by the tuneKfMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Kf value of the PID controller being tuned.
     */
    double getTuneKf()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kF;
        }

        return value;
    }   // getTuneKF

    /**
     * This method reads all sensors and prints out their values. This is a very useful diagnostic tool to check
     * if all sensors are working properly. For encoders, since test sensor mode is also teleop mode, you can
     * operate the gamepads to turn the motors and check the corresponding encoder counts.
     */
    private void doSensorsTest()
    {
        //
        // Read all sensors and display on the dashboard.
        // Drive the robot around to sample different locations of the field.
        //
        if (!RobotParams.Preferences.visionOnly)
        {
            robot.dashboard.displayPrintf(
                8, "Enc: lw=%.0f,rw=%.0f",
                robot.robotDrive.leftWheels.getPosition(), robot.robotDrive.rightWheels.getPosition());
        }

        if (robot.robotDrive.gyro != null)
        {
            Orientation orientation = robot.robotDrive.imu.imu.getAngularOrientation();

            robot.dashboard.displayPrintf(13, "x=%.2f, y=%.2f, z=%.2f",
                    robot.robotDrive.gyro.getRawXData(TrcGyro.DataType.HEADING).value,
                    robot.robotDrive.gyro.getRawYData(TrcGyro.DataType.HEADING).value,
                    robot.robotDrive.gyro.getRawZData(TrcGyro.DataType.HEADING).value);

            robot.dashboard.displayPrintf(
                9, "Gyro - Z: Rate=%.3f,Heading=%.1f",
                robot.robotDrive.gyro.getZRotationRate().value, robot.robotDrive.gyro.getZHeading().value);
            robot.dashboard.displayPrintf(
                    10, "Gyro - Y: Rate=%.3f,Heading=%.1f",
                    robot.robotDrive.gyro.getYRotationRate().value, robot.robotDrive.gyro.getYHeading().value);
            robot.dashboard.displayPrintf(
                    11, "Gyro - X: Rate=%.3f,Heading=%.1f",
                    robot.robotDrive.gyro.getXRotationRate().value, robot.robotDrive.gyro.getXHeading().value);
        }
    }   // doSensorsTest

    /**
     * This method calls vision code to detect target objects and display their info.
     */
    private void doVisionTest()
    {
        if (robot.vision != null)
        {
//            if (RobotParams.Preferences.useTensorFlow)
//            {
//                FtcTensorFlow.TargetInfo[] targetsInfo = robot.vision.getDetectedTargetsInfo(null, null, null);
//                final int maxNumLines = 3;
//                int lineIndex = 10;
//                int endLine = lineIndex + maxNumLines;
//
//                if (targetsInfo != null)
//                {
//                    int numTargets = Math.min(targetsInfo.length, maxNumLines);
//                    for (int i = 0; i < numTargets; i++)
//                    {
//                        robot.dashboard.displayPrintf(lineIndex, "%s", targetsInfo[i]);
//                        lineIndex++;
//                    }
//                }
//
//                while (lineIndex < endLine)
//                {
//                    robot.dashboard.displayPrintf(lineIndex, "");
//                    lineIndex++;
//                }
//            }
//
//            if (RobotParams.Preferences.useVuforia)
//            {
//                TrcPose2D robotPose = robot.vision.getRobotPose(null, false);
//                robot.dashboard.displayPrintf(13, "RobotLoc %s: %s",
//                                              robot.vision.getLastSeenVuforiaImageName(), robotPose);
//            }
        }
    }   //doVisionTest

    /**
     * This method is called to determine if Test mode is allowed to do teleop control of the robot.
     *
     * @return true to allow and false otherwise.
     */
    private boolean allowTeleOp()
    {
        return !RobotParams.Preferences.visionOnly &&
               (testChoices.test == Test.SUBSYSTEMS_TEST || testChoices.test == Test.DRIVE_SPEED_TEST);
    }   // shouldDoTeleOp

}   // class FtcTest
