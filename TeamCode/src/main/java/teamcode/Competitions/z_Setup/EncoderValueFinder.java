package teamcode.Competitions.z_Setup;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcOpMode;
import teamcode.Season_Setup.Robot;
import teamcode.Season_Setup.RobotParams;


@TeleOp(name="Encoder Value Finder", group="Setup")

public class EncoderValueFinder extends FtcOpMode
{
    // Class Setup
    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;


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
        robot.dashboard.displayPrintf(2, "Back Left Encoder Pulses: %.3f", robot.robotDrive.leftWheels.getMotorPosition());
        robot.dashboard.displayPrintf(3, "Back Right Encoder Pulses: %.3f",   robot.robotDrive.rightWheels.getMotorPosition());
        robot.dashboard.displayPrintf(4, "Arm Extender Encoder Pulses: %.3f",   robot.armExtender.getMotor().getMotorPosition());
        robot.dashboard.displayPrintf(5, "Arm Rotator Encoder Pulses: %.3f",   robot.armRotator.getMotor().getMotorPosition());
        robot.dashboard.displayPrintf(6, "Arm Platform Rotator Encoder Pulses: %.3f",   robot.armPlatformRotator.getMotor().getMotorPosition());
    }


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
                7, "%s: %04x->%s", gamepad, button, pressed ? "Pressed" : "Released");

        // Reset Encoder Values
        if (button == FtcGamepad.GAMEPAD_A)
        {
            robot.robotDrive.leftWheels.resetPosition(true);
            robot.robotDrive.rightWheels.resetPosition(true);
            robot.armExtender.getMotor().resetPosition(true);
            robot.armRotator.getMotor().resetPosition(true);
            robot.armPlatformRotator.getMotor().resetPosition(true);
//            robot.carouselSpinnerRotator.getMotor().resetPosition(true);
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
                7, "%s: %04x->%s", gamepad, button, pressed ? "Pressed" : "Released");

        // Reset Encoder Values
        if (button == FtcGamepad.GAMEPAD_A)
        {
            robot.robotDrive.leftWheels.resetPosition(true);
            robot.robotDrive.rightWheels.resetPosition(true);
            robot.armExtender.getMotor().resetPosition(true);
            robot.armRotator.getMotor().resetPosition(true);
            robot.armPlatformRotator.getMotor().resetPosition(true);
//            robot.carouselSpinnerRotator.getMotor().resetPosition(true);
        }
    }   // operatorButtonEvent
}
