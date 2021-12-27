package Old_Repo.Test.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;


@Autonomous(name="IMU Test - Turn", group="Test")

public class IMU_Turn_Test extends LinearOpMode {

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
        telemetry.addData("Encoder Position",  "Encoder Pulses Left, Right",
                ducky.backLeft.getCurrentPosition(),
                ducky.backRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Waiting for the program to start
        waitForStart();

        // Autonomous Pathing

        // Turning Left
        ducky.turn_P(-90.0,3000, 0);
        ducky.turn_P(-45.0,3000, 1000);

        // Program taking a 2 second break
        Thread.sleep(2000);

        // Turning Right
        ducky.turn_P(45.0,3000, 0);
        ducky.turn_P(90.0,3000, 1000);
    }
}
