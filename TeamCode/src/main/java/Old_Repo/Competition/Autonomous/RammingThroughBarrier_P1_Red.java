package Old_Repo.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;


@Autonomous(name="Ramming through Barrier - P1, Red", group="Competition - Red")

public class RammingThroughBarrier_P1_Red extends LinearOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all motors/ servos
        ducky.init(hardwareMap, telemetry);

        // Reset Encoders, and Telemetry Update
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // Encoder Position Update
        telemetry.addData("Encoder Position",  "Starting Encoder Position",
                ducky.backLeft.getCurrentPosition(),
                ducky.backRight.getCurrentPosition());
        telemetry.update();

        // Setting Alliance Colour for TeleOp
        ducky.writeAndRead("Red");

        // Telemetry Update
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("Alliance", Ducky.alliance);
        telemetry.update();

        // Waiting for the program to start
        waitForStart();

        // Autonomous Pathing
        ducky.driveBackward_Encoder(6,0.5,5000);
        ducky.turn_P(-90,5000, 1000);
        ducky.driveForward_Power(1);
        Thread.sleep(2500);
        ducky.stop_Power();
        Thread.sleep(1000);
        ducky.driveForward_Power(0.2);
        Thread.sleep(500);
        ducky.stop_Power();
    }
}
