package teamcode.Test;

import android.annotation.SuppressLint;

import java.util.Locale;

import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcMatchInfo;
import TrcFtcLib.ftclib.FtcOpMode;
import teamcode.Competitions.Autonomous.FtcAuto;
import teamcode.Season_Setup.Freight_Frenzy_Pipeline;
import teamcode.Season_Setup.Robot;
import teamcode.Season_Setup.RobotParams;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="EasyOpenCV", group="Test")

public class EasyOpenCV_Test extends FtcOpMode
{
    private static final String moduleName = "FtcAuto";
    private static final boolean logEvents = true;
    private static final boolean debugPid = true;

    private Robot robot;
    private FtcMatchInfo matchInfo;
    private final FtcAuto.AutoChoices autoChoices = new FtcAuto.AutoChoices();
    private TrcRobot.RobotCommand autoCommand = null;


    // Implements FtcOpMode abstract method

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void initRobot()
    {
        @SuppressWarnings("unused")
        final String funcName = "initRobot";

        // Create and initialize robot object
        robot = new Robot(TrcRobot.getRunMode());

        // Open trace log
        if (RobotParams.Preferences.useTraceLog)
        {
            matchInfo = FtcMatchInfo.getMatchInfo();
            String filePrefix = String.format(Locale.US, "%s%02d", matchInfo.matchType, matchInfo.matchNumber);
            robot.globalTracer.openTraceLog(RobotParams.LOG_PATH_FOLDER, filePrefix);
        }

        if (robot.vision != null)
        {
            // Telemetry Data
            telemetry.addData("Frame Count", robot.webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", robot.webcam.getFps()));
            telemetry.addData("Total frame time ms", robot.webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", robot.webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", robot.webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", robot.webcam.getCurrentPipelineMaxFps());

            // Telemetry Ring Data
            telemetry.addData("Analysis - Left",   robot.vision.leftValue);
            telemetry.addData("Analysis - Center", robot.vision.centerValue);
            telemetry.addData("Analysis - Right",  robot.vision.rightValue);

            // Telemetry Update
            telemetry.update();
        }
    }   // initRobot


    // Overrides TrcRobot.RobotMode methods.

    /**
     * This method is called periodically after initRobot() is called but before competition starts. For this season,
     * we are detecting the duck's barcode position before the match starts.
     */
    @Override
    public void initPeriodic()
    {
    }   // initPeriodic

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

        if (RobotParams.Preferences.useTraceLog)
        {
            robot.globalTracer.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(moduleName, "***** Starting autonomous *****");
        if (matchInfo != null)
        {
            robot.globalTracer.logInfo(moduleName, "MatchInfo", "%s", matchInfo);
        }
        robot.globalTracer.logInfo(moduleName, "AutoChoices", "%s", autoChoices);

        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode
        robot.startMode(nextMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(true);
        }

        robot.robotDrive.pidDrive.setMsgTracer(robot.globalTracer, logEvents, debugPid, robot.battery);
        robot.robotDrive.purePursuitDrive.setMsgTracer(robot.globalTracer, logEvents, debugPid, robot.battery);
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
        // Opmode is about to stop, cancel autonomous command in progress if any.
        if (autoCommand != null)
        {
            autoCommand.cancel();
        }

        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode
        robot.stopMode(prevMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(false);
        }

        printPerformanceMetrics(robot.globalTracer);

        if (robot.globalTracer.tracerLogIsOpened())
        {
            robot.globalTracer.closeTraceLog();
        }
    }   // stopMode

    /**
     * This method is called periodically as fast as the control system allows. Typically, you put code that requires
     * servicing at a higher frequency here. To make the robot as responsive and as accurate as possible especially
     * in autonomous mode, you will typically put that code here.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoCommand != null)
        {
            // Run the autonomous command
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   // runContinuous
}
