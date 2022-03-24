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
@TeleOp(name="Competition TeleOp", group="Competition")

public class FtcTeleOp extends FtcOpMode
{
    // Class Setup
    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;

    // Drivebase Variables
    private boolean alreadyStopped;

    // Arm Variables
    private double armExtenderPowerScale = 1.0;
    private double armRotatorPowerScale = 0.75;
    private double armPlatformRotatorPowerScale = 0.5;
    private boolean armAlreadyStopped;

    @SuppressWarnings("FieldCanBeLocal")
    private String armExtenderLevel = "N/A";
    @SuppressWarnings("FieldCanBeLocal")
    private String armRotatorLevel = "N/A";
    @SuppressWarnings("FieldCanBeLocal")
    private String armPlatformRotatorLevel = "N/A";


    // Mechanism toggle
    private boolean collectorOn;
    private boolean collectorReverse;

    // System Toggle
    private boolean manualOverrideOn;
    private boolean adaptiveArmExtension;

    // Alliance change toggle
    @SuppressWarnings("FieldCanBeLocal")
    private boolean allianceChange_Safety_One;
    private boolean allianceChange_Safety_Two;


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
            // Drive Variables
            boolean invertedDrive = driverGamepad.getLeftTrigger() > 0;
            double drivePowerScale = driverGamepad.getRightTrigger() > 0 ? RobotParams.SLOW_DRIVE_POWER_SCALE : 1.0;

            switch (RobotParams.ROBOT_DRIVE_MODE)
            {
                case TANK_MODE:
                {
                    double leftPower = driverGamepad.getLeftStickY(true)* drivePowerScale;
                    double rightPower = driverGamepad.getRightStickY(true)* drivePowerScale;

                    // If the driver is not controlling the drivebase, allow other systems to control the drivebase.
                    if (leftPower != 0 || rightPower != 0)
                    {
                        robot.robotDrive.driveBase.tankDrive(leftPower, rightPower, invertedDrive);
                        alreadyStopped = false;

                    }
                    else if (!alreadyStopped)
                    {
                        robot.robotDrive.driveBase.stop();
                        alreadyStopped = true;
                    }

                    robot.dashboard.displayPrintf(1, "Tank: L = %.1f, R = %.1f, Inv = %s, Slow = %s",
                            robot.robotDrive.leftWheels.getMotorPower(), robot.robotDrive.rightWheels.getMotorPower(),
                            invertedDrive, drivePowerScale);
                    break;
                }

                case ARCADE_MODE:
                {
                    double x = driverGamepad.getRightStickX(true)* drivePowerScale;
                    double y = driverGamepad.getRightStickY(true)* drivePowerScale;
                    double rot = driverGamepad.getLeftStickX(true)* drivePowerScale;
                    robot.robotDrive.driveBase.holonomicDrive(x, y, rot, invertedDrive);
                    robot.dashboard.displayPrintf(1, "Arcade: x = %.1f, y = %.1f, Rot = %.1f, Inv = %s",
                                                  x, y, rot, invertedDrive);
                    break;
                }
            }


            robot.dashboard.displayPrintf(2, "DriveBase: x = %.2f, y = %.2f, Heading = %.2f",
                                          robot.robotDrive.driveBase.getXPosition(),
                                          robot.robotDrive.driveBase.getYPosition(),
                                          robot.robotDrive.driveBase.getHeading());

            robot.dashboard.displayPrintf(15, "isRedAlliance = %s", Robot.isRedAlliance);
        }

        //// Other subsystems

        // Arm Extender
        // Arm Extender
        if (robot.armExtender != null)
        {
            double armExtenderPower = operatorGamepad.getLeftTrigger(true) -
                    operatorGamepad.getRightTrigger(true);

            robot.armExtender.setPower(armExtenderPower * armExtenderPowerScale);

            robot.dashboard.displayPrintf(4, "Arm Extender: Pow = %.1f, Pos = %.1f",
                    robot.armExtender.getMotor().getMotorPower(), robot.armExtender.getPosition());
        }

        // Arm Rotator
        if (robot.armRotator != null)
        {
            double armRotatorPower = operatorGamepad.getLeftStickY(true);

            if (armRotatorPower < 0)
            {
                armRotatorPower /= Math.sin(Math.toRadians(robot.armRotator.getPosition()))*
                        RobotParams.ARM_ROTATOR_LOWERING_ARM_POWER_SCALE;
            }

            robot.armRotator.setPower(armRotatorPower * armRotatorPowerScale, true);
            robot.dashboard.displayPrintf(5, "Arm Rotator: Pow = %.3f, Pos = %.1f",
                    robot.armRotator.getMotor().getMotorPower(), robot.armRotator.getPosition());
        }

        // Arm Platform Rotator
        if (robot.armPlatformRotator != null)
        {
            double armPlatformRotatorPower = operatorGamepad.getLeftStickX(true);

            robot.armPlatformRotator.setPower(armPlatformRotatorPower * armPlatformRotatorPowerScale);
            robot.dashboard.displayPrintf(6, "Arm Platform Rotator: Pow = %.1f, Pos = %.1f",
                    robot.armPlatformRotator.getMotor().getMotorPower(), robot.armPlatformRotator.getPosition());
        }

    }   // runPeriodic


    // Implements TrcGameController.ButtonHandler interface.


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
            8, "%s: %04x -> %s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            // Alliance change Safety Trigger One
            case FtcGamepad.GAMEPAD_Y:
                allianceChange_Safety_One = pressed;

                if (allianceChange_Safety_One && allianceChange_Safety_Two)
                {
                    Robot.isRedAlliance = !Robot.isRedAlliance;

                    robot.speak(Robot.isRedAlliance?"Alliance changed to Red": "Alliance changed to Blue");
                }

                break;

            // Carousel Spinner On, and rotate to the appropriate side
            case FtcGamepad.GAMEPAD_LBUMPER:
                if (robot.carouselSpinner != null)
                {
                    if (pressed)
                    {
                        robot.carouselSpinner.setPosition(Robot.isRedAlliance?RobotParams.CAROUSEL_SPINNER_RED:
                                RobotParams.CAROUSEL_SPINNER_BLUE);
                        robot.robotDrive.driveBase.tankDrive(-0.2, -0.2);
                    }
                    else
                    {
                        robot.carouselSpinner.setPosition(RobotParams.CAROUSEL_SPINNER_STOP_POWER);
                        robot.robotDrive.driveBase.stop();
                    }
                }
                break;


            // Arm Extender retract (level down)
            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                if (robot.armExtender != null && pressed)
                {
                    robot.armExtender.levelDown();

                    if (robot.armExtender.getLevel() == 0)
                    {
                        armExtenderLevel = "0";

                    } else if (robot.armExtender.getLevel() == 1)
                    {
                        armExtenderLevel = "3";

                    } else if (robot.armExtender.getLevel() == 2)
                    {
                        armExtenderLevel = "6";

                    } else
                    {
                        armExtenderLevel = "8";

                    }

                    robot.speak("Arm Extender Position " + armExtenderLevel);
                    robot.dashboard.displayPrintf(14, "Arm Extender Level = %s, Pos = %.1f",
                            armExtenderLevel, robot.armExtender.getPosition());
                }

                break;

            // Arm Extender extend (level up)
            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                if (robot.armExtender != null && pressed)
                {
                    robot.armExtender.levelUp();

                    if (robot.armExtender.getLevel() == 0)
                    {
                        armExtenderLevel = "0";

                    } else if (robot.armExtender.getLevel() == 1)
                    {
                        armExtenderLevel = "3";

                    } else if (robot.armExtender.getLevel() == 2)
                    {
                        armExtenderLevel = "6";

                    } else
                    {
                        armExtenderLevel = "8";

                    }

                    robot.speak("Arm Extender Position " + armExtenderLevel);
                    robot.dashboard.displayPrintf(14, "Arm Extender Level = %s, Pos = %.1f",
                            armExtenderLevel, robot.armExtender.getPosition());
                    break;
                }
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
            8, "%s: %04x -> %s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            // Collector On (Toggle)
            case FtcGamepad.GAMEPAD_X:
                if (robot.collector != null && pressed)
                {
                    collectorOn = !collectorOn;

                    if (collectorOn)
                    {
                        collectorReverse = false;

                        robot.collector.setPosition(RobotParams.COLLECTOR_PICKUP_POWER);
                    } else
                    {
                        robot.collector.setPosition(RobotParams.COLLECTOR_STOP_POWER);
                    }
                }

                break;

            // Collector Reverse (Toggle)
            case FtcGamepad.GAMEPAD_Y:
                if (robot.collector != null && pressed)
                {
                    collectorReverse = !collectorReverse;

                    if (collectorReverse)
                    {
                        collectorOn = false;

                        robot.collector.setPosition(RobotParams.COLLECTOR_DEPOSIT_POWER);
                    } else
                    {
                        robot.collector.setPosition(RobotParams.COLLECTOR_STOP_POWER);
                    }
                }

                break;

//            // Toggle Adaptive Arm Extension - Keep height off the ground constant as arm extends
//            case FtcGamepad.GAMEPAD_LBUMPER:
//                if (pressed)
//                {
//                    adaptiveArmExtension = !adaptiveArmExtension;
//
//                    String msg = adaptiveArmExtension?"Adaptive Arm Extension On": "Adaptive Arm Extension Off";
//                    robot.speak(msg);
//                    robot.dashboard.displayPrintf(13, msg);
//                }
//
//                break;

            // Arm System Slow Button
            case FtcGamepad.GAMEPAD_RBUMPER:
                armExtenderPowerScale = pressed? RobotParams.ARM_EXTENDER_SLOW_POWER_SCALE: 1.0;
                armRotatorPowerScale = pressed? RobotParams.ARM_ROTATOR_SLOW_POWER_SCALE: 0.75;
                armPlatformRotatorPowerScale = pressed? RobotParams.ARM_PLATFORM_ROTATOR_SLOW_POWER_SCALE: 0.5;

                robot.dashboard.displayPrintf(9, "Arm Slow Button: %s", pressed? "Pressed": "Released");

                break;

            // Arm level up
            case FtcGamepad.GAMEPAD_DPAD_UP:
                if (robot.armRotator != null && pressed)
                {
                    robot.armRotator.levelUp();

                    if (robot.armRotator.getLevel() == 0)
                    {
                        armRotatorLevel = "Collecting";

                    } else if (robot.armRotator.getLevel() == 1)
                    {
                        armRotatorLevel = "Bottom";

                    } else if (robot.armRotator.getLevel() == 2)
                    {
                        armRotatorLevel = "Mid";

                    } else if (robot.armRotator.getLevel() == 3)
                    {
                        armRotatorLevel = "Top";

                    } else
                    {
                        armRotatorLevel = "Unknown";

                    }

                    robot.speak("Arm Level " + armRotatorLevel);
                    robot.dashboard.displayPrintf(10, "Arm Rotator Level = %s",
                            armRotatorLevel);
                }

                break;

            // Arm level down
            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                if (robot.armRotator != null && pressed)
                {
                    robot.armRotator.levelDown();

                    if (robot.armRotator.getLevel() == 0)
                    {
                        armRotatorLevel = "Collecting";

                    } else if (robot.armRotator.getLevel() == 1)
                    {
                        armRotatorLevel = "Bottom";

                    } else if (robot.armRotator.getLevel() == 2)
                    {
                        armRotatorLevel = "Mid";

                    } else if (robot.armRotator.getLevel() == 3)
                    {
                        armRotatorLevel = "Top";

                    } else
                    {
                        armRotatorLevel = "Unknown";

                    }

                    robot.speak("Arm Level " + armRotatorLevel);
                    robot.dashboard.displayPrintf(10, "Arm Rotator Level = %s",
                            armRotatorLevel);
                }

                break;

            // Arm Platform Rotator rotate towards Front (level down)
            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                if (robot.armPlatformRotator != null && pressed)
                {
                    robot.armPlatformRotator.levelDown();

                    if (robot.armPlatformRotator.getLevel() == 0)
                    {
                        armPlatformRotatorLevel = "Front";

                    } else if (robot.armPlatformRotator.getLevel() == 1)
                    {
                        armPlatformRotatorLevel = "Middle";

                    } else if (robot.armPlatformRotator.getLevel() == 2)
                    {
                        armPlatformRotatorLevel = "Back";

                    } else
                    {
                        armPlatformRotatorLevel = "Unknown";

                    }

                    robot.speak("Arm Platform Rotator Position " + armPlatformRotatorLevel);
                    robot.dashboard.displayPrintf(11, "Arm Platform Rotator Level = %s",
                            armPlatformRotatorLevel);
                }

                break;

            // Arm Platform Rotator rotate towards Back (level up)
            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                if (robot.armPlatformRotator != null && pressed)
                {
                    robot.armPlatformRotator.levelUp();

                    if (robot.armPlatformRotator.getLevel() == 0)
                    {
                        armPlatformRotatorLevel = "Front";

                    } else if (robot.armPlatformRotator.getLevel() == 1)
                    {
                        armPlatformRotatorLevel = "Middle";

                    } else if (robot.armPlatformRotator.getLevel() == 2)
                    {
                        armPlatformRotatorLevel = "Back";

                    } else
                    {
                        armPlatformRotatorLevel = "Unknown";

                    }

                    robot.speak("Arm Platform Rotator Position " + armPlatformRotatorLevel);
                    robot.dashboard.displayPrintf(11, "Arm Platform Rotator Level = %s",
                            armPlatformRotatorLevel);
                }

                break;

            // Zero Calibrate
            case FtcGamepad.GAMEPAD_GUIDE:
                if (robot.armExtender != null && robot.armRotator != null &&
                        robot.armPlatformRotator != null)
                {
                    // Disable Manual Override
                    manualOverrideOn = false;
                    robot.armExtender.setManualOverride(false);
                    robot.armRotator.setManualOverride(false);
                    robot.armPlatformRotator.setManualOverride(false);
                    String msg = "Manual Override Off";
                    robot.speak(msg);
                    robot.dashboard.displayPrintf(12, msg);

                    // Zero Calibrate all mechanism motors
                    robot.armExtender.zeroCalibrate();
                    robot.armRotator.zeroCalibrate();
                    robot.armPlatformRotator.zeroCalibrate();
                }

                break;

            // Toggle Manual Override
            case FtcGamepad.GAMEPAD_BACK:
                if (robot.armExtender != null && robot.armRotator != null &&
                        robot.armPlatformRotator != null && pressed)
                {
                    manualOverrideOn = !manualOverrideOn;

                    robot.armExtender.setManualOverride(manualOverrideOn);
                    robot.armRotator.setManualOverride(manualOverrideOn);
                    robot.armPlatformRotator.setManualOverride(manualOverrideOn);

                    String msg = manualOverrideOn?"Manual Override On": "Manual Override Off";
                    robot.speak(msg);
                    robot.dashboard.displayPrintf(12, msg);
                }

                break;
        }
    }   // operatorButtonEvent

}   // class FtcTeleOp
