package Old_Repo.Test.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;
import org.firstinspires.ftc.teamcode.Season_Setup.Freight_Frenzy_Pipeline;

@TeleOp(name="EasyOpenCV", group="Test")

public class EasyOpenCV_Test extends LinearOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();
    Freight_Frenzy_Pipeline freight_frenzy_pipeline = new Freight_Frenzy_Pipeline();

    /**
     * Initializing the Program
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        // Initialize all motors/ servos
        ducky.init(hardwareMap, telemetry);

        // Indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Waiting for the program to start
        waitForStart();

        while (opModeIsActive()) {
            // Telemetry Data
            telemetry.addData("Frame Count", ducky.webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", ducky.webcam.getFps()));
            telemetry.addData("Total frame time ms", ducky.webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", ducky.webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", ducky.webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", ducky.webcam.getCurrentPipelineMaxFps());

            // Telemetry Ring Data
            telemetry.addData("Analysis - Left",   Freight_Frenzy_Pipeline.analysisLeft);
            telemetry.addData("Analysis - Center", Freight_Frenzy_Pipeline.analysisCenter);
            telemetry.addData("Analysis - Right",  Freight_Frenzy_Pipeline.analysisRight);
            telemetry.addData("Position", Freight_Frenzy_Pipeline.positionOfTeamShippingElement);

            // Telemetry Update
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if (gamepad1.a) {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                freight_frenzy_pipeline.webcam.stopStreaming();
            }
        }
    }
}
