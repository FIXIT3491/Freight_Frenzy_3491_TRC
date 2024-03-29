package teamcode.Test;

import android.annotation.SuppressLint;


import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcOpMode;
import teamcode.Season_Setup.Freight_Frenzy_Pipeline;
import teamcode.Season_Setup.Robot;
import teamcode.Season_Setup.RobotParams;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="EasyOpenCV", group="Test")

@SuppressWarnings("unused")
public class EasyOpenCV_Test extends FtcOpMode {
    private static final String moduleName = "EasyOpenCV Test";
    private static final boolean logEvents = true;
    private static final boolean debugPid = true;

    private Robot robot;

    // Create Instance of Pipeline Class
    Freight_Frenzy_Pipeline freight_frenzy_pipeline = new Freight_Frenzy_Pipeline();


    // Implements FtcOpMode abstract method

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void initRobot() {
        @SuppressWarnings("unused") final String funcName = "initRobot";

        // Create and initialize robot object
        robot = new Robot(TrcRobot.getRunMode());
    }   // initRobot


    // Overrides TrcRobot.RobotMode methods.

    /**
     * This method is called periodically after initRobot() is called but before competition starts. For this season,
     * we are detecting the duck's barcode position before the match starts.
     */
    @Override
    public void initPeriodic() {
        if (robot.vision != null) {
            robot.dashboard.displayPrintf(2, "Frame Count: %d", robot.webcam.getFrameCount());
            robot.dashboard.displayPrintf(3, "FPS: %.2f", robot.webcam.getFps());
            robot.dashboard.displayPrintf(4, "Total frame time ms: %d", robot.webcam.getTotalFrameTimeMs());
            robot.dashboard.displayPrintf(5, "Pipeline time ms: %d", robot.webcam.getPipelineTimeMs());
            robot.dashboard.displayPrintf(6, "Overhead time ms: %d", robot.webcam.getOverheadTimeMs());
            robot.dashboard.displayPrintf(7, "Theoretical max FPS: %d", robot.webcam.getCurrentPipelineMaxFps());

            robot.dashboard.displayPrintf(9, "Analysis - Left: %.2f", Freight_Frenzy_Pipeline.analysisLeft);
            robot.dashboard.displayPrintf(10, "Analysis - Center: %.2f", Freight_Frenzy_Pipeline.analysisCenter);
            robot.dashboard.displayPrintf(11, "Analysis - Right: %.2f", Freight_Frenzy_Pipeline.analysisRight);
            robot.dashboard.displayPrintf(12, "Position - int: %d", freight_frenzy_pipeline.getPosition().value);

            robot.dashboard.displayPrintf(14, "Position: %s", freight_frenzy_pipeline.getPosition());


        }
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
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode) {
        robot.dashboard.clearDisplay();

        if (RobotParams.Preferences.useTraceLog) {
            robot.globalTracer.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(moduleName, "***** Starting autonomous *****");

        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode
        robot.startMode(nextMode);

        if (robot.battery != null) {
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
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode) {
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode
        robot.stopMode(prevMode);

        if (robot.battery != null) {
            robot.battery.setEnabled(false);
        }

        printPerformanceMetrics(robot.globalTracer);

        if (robot.globalTracer.tracerLogIsOpened()) {
            robot.globalTracer.closeTraceLog();
        }
    }   // stopMode
}