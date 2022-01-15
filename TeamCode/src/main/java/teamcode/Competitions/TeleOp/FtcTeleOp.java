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

package teamcode.Competitions.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcOpMode;
import teamcode.Season_Setup.Robot;
import teamcode.Season_Setup.RobotParams;

/**
 * This class contains the TeleOp Mode program.
 */
@TeleOp(name="FtcTeleOp", group="Ducky")
public class FtcTeleOp extends FtcOpMode
{
    // Class Setup
    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;

    // Drive Variables
    private boolean invertedDrive = false;
    private double drivePowerScale = 1.0;

    // Arm Variables
    private double armExtender_Powerscale = 1.0;
    private double armSystemPowerScale = 1.0; // Arm Rotator, Arm Platform Rotator

    // Mechanism toggle
    private boolean collector_On;
    private boolean collector_Reversing;
    private boolean armExtender_On;
    private boolean carouselSpinner_On;
    private boolean tapeMeasure_Safety_One;
    private boolean tapeMeasure_Safety_Two;


    // Implements FtcOpMode abstract method


    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void initRobot()
    {
        // Create and initialize robot object.
        robot = new Robot(TrcRobot.getRunMode());

        // Create and initialize Gamepads
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1, this::driverButtonEvent);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2, this::operatorButtonEvent);
        driverGamepad.setYInverted(true);
        operatorGamepad.setYInverted(true);
    }   // initRobot


    // Overrides TrcRobot.RobotMode methods


    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.dashboard.clearDisplay();

        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        robot.startMode(nextMode);
    }   // startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        robot.stopMode(prevMode);

        if (!RobotParams.Preferences.competitionMode) {
            printPerformanceMetrics(robot.globalTracer);
        }
    }   // stopMode

    /**
     * This method is called periodically about 50 times a second. Typically, you put code that doesn't require
     * frequent update here. For example, TeleOp joystick code can be put here since human responses are considered
     * slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runPeriodic(double elapsedTime)
    {
        // DriveBase subsystem.
        if (robot.robotDrive != null)
        {
            invertedDrive = driverGamepad.getLeftTrigger() > 0;
            drivePowerScale = driverGamepad.getRightTrigger() > 0? RobotParams.SLOW_DRIVE_POWER_SCALE: 1.0;

            switch (RobotParams.ROBOT_DRIVE_MODE)
            {
                case TANK_MODE:
                {
                    double leftPower = driverGamepad.getLeftStickY(true)*drivePowerScale;
                    double rightPower = driverGamepad.getRightStickY(true)*drivePowerScale;
                    robot.robotDrive.driveBase.tankDrive(leftPower, rightPower, invertedDrive);
                    robot.dashboard.displayPrintf(1, "Tank:left=%.1f,right=%.1f,inv=%s",
                                                  leftPower, rightPower, invertedDrive);
                    break;
                }

                case ARCADE_MODE:
                {
                    double x = driverGamepad.getRightStickX(true)*drivePowerScale;
                    double y = driverGamepad.getRightStickY(true)*drivePowerScale;
                    double rot = driverGamepad.getLeftStickX(true)*drivePowerScale;
                    robot.robotDrive.driveBase.holonomicDrive(x, y, rot, invertedDrive);
                    robot.dashboard.displayPrintf(1, "Tim:x=%.1f,y=%.1f,rot=%.1f,inv=%s",
                                                  x, y, rot, invertedDrive);
                    break;
                }
            }


            robot.dashboard.displayPrintf(2, "DriveBase: x=%.2f,y=%.2f,heading=%.2f",
                                          robot.robotDrive.driveBase.getXPosition(),
                                          robot.robotDrive.driveBase.getYPosition(),
                                          robot.robotDrive.driveBase.getHeading());
        }

        //// Other subsystems

        // Arm Extender
        if (robot.armExtender != null)
        {
            double armExtenderPower = operatorGamepad.getLeftTrigger(true) -
                    operatorGamepad.getRightTrigger(true);

            robot.armRotator.setPower(armExtenderPower);

            robot.dashboard.displayPrintf(3, "Arm Extender: Pow=%.1f,Pos=%.1f",
                        armExtenderPower, robot.armRotator.getPosition());
        }

        // Arm Rotator
        if (robot.armRotator != null)
        {
            double armRotatorPower = operatorGamepad.getLeftStickY(true);

            robot.armRotator.setPower(armRotatorPower * armSystemPowerScale);
            robot.dashboard.displayPrintf(3, "Arm Rotator: Pow=%.1f,Pos=%.1f",
                    armRotatorPower, robot.armRotator.getPosition());
        }

        // Arm Platform Rotator
        if (robot.armPlatformRotator != null)
        {
            double armPlatformRotatorPower = operatorGamepad.getLeftStickX(true);

            robot.armRotator.setPower(armPlatformRotatorPower * armSystemPowerScale);
            robot.dashboard.displayPrintf(3, "Arm Platform Rotator: Pow=%.1f,Pos=%.1f",
                    armPlatformRotatorPower, robot.armRotator.getPosition());
        }


    }   // runPeriodic

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void driverButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            7, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                tapeMeasure_Safety_One = pressed;

                if (tapeMeasure_Safety_One && tapeMeasure_Safety_Two)
                {
                    robot.tapeMeasure.setPosition(RobotParams.TAPE_MEASURE_EXTENDED);
                }

                break;

            case FtcGamepad.GAMEPAD_B:
                break;

            case FtcGamepad.GAMEPAD_X:
                break;

            case FtcGamepad.GAMEPAD_Y:
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                if (pressed)
                {
                    // Toggle inverted drive.
                    invertedDrive = !invertedDrive;
                }
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                // Press and hold for slow drive.
                drivePowerScale = pressed? RobotParams.SLOW_DRIVE_POWER_SCALE: 1.0;

                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                tapeMeasure_Safety_Two = pressed;

                break;
        }

    }   // driverButtonEvent

    /**
     * This method is called when operator gamepad button event is detected.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void operatorButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            7, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                break;

            case FtcGamepad.GAMEPAD_B:
                break;

            case FtcGamepad.GAMEPAD_X:
                // Collector On

                break;

            case FtcGamepad.GAMEPAD_Y:
                // Collector Reverse

                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                break;

            // Arm System Slow Button
            case FtcGamepad.GAMEPAD_RBUMPER:
                if (robot.armRotator != null)
                {
                    armSystemPowerScale = pressed? RobotParams.ARM_ROTATOR_SLOW_POWER_SCALE : 1.0;
                }

                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                // Arm Collecting

                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                // Arm Bottom level

                break;

            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                // Arm Top level

                break;

            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                // Arm Mid level

                break;
        }
    }   //operatorButtonEvent

}   //class FtcTeleOp
