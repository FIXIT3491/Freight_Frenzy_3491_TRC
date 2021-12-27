package Old_Repo.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;

@Disabled
@Autonomous(name="Ramming through Barrier (Power Only) - P1, Red", group="Competition - Red")

public class RammingThroughBarrier_powerOnly_P1_Red extends LinearOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    @Override
    public void runOpMode() throws InterruptedException{

        // Initialize all motors/ servos
        ducky.init(hardwareMap, telemetry);

        // Setting Alliance Colour for TeleOp
        ducky.writeAndRead("Red");

        // Telemetry Update
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("Alliance", Ducky.alliance);
        telemetry.update();

        // Waiting for the program to start
        waitForStart();

        // Autonomous Pathing
        DriveBackward_Power(0.2,600);
        Thread.sleep(2000);
        TurnLeft_Power(0.4,800);
        Thread.sleep(2000);
        DriveForward_Power(0.5, 1500);
        Thread.sleep(2000);
    }

    // Robot Driving (Power and Time only)
    public void DriveForward_Power(double speed, int milliseconds) throws InterruptedException {
        ducky.frontLeft.setPower(speed);
        ducky.backLeft.setPower(speed*Ducky.BACK_WHEEL_POWER_REDUCTION);
        ducky.frontRight.setPower(speed);
        ducky.backRight.setPower(speed*Ducky.BACK_WHEEL_POWER_REDUCTION);
        Thread.sleep(milliseconds);
        Stop_Power();
    }
    public void DriveBackward_Power(double speed, int milliseconds) throws InterruptedException {
        ducky.frontLeft.setPower(-speed);
        ducky.backLeft.setPower(-speed*Ducky.BACK_WHEEL_POWER_REDUCTION);
        ducky.frontRight.setPower(-speed);
        ducky.backRight.setPower(-speed*Ducky.BACK_WHEEL_POWER_REDUCTION);
        Thread.sleep(milliseconds);
        Stop_Power();
    }
    public void TurnLeft_Power(double speed, int milliseconds) throws InterruptedException {
        ducky.frontLeft.setPower(-speed);
        ducky.backLeft.setPower(-speed*Ducky.BACK_WHEEL_POWER_REDUCTION);
        ducky.frontRight.setPower(speed);
        ducky.backRight.setPower(speed*Ducky.BACK_WHEEL_POWER_REDUCTION);
        Thread.sleep(milliseconds);
        Stop_Power();
    }
    public void Stop_Power() {
        ducky.frontLeft.setPower(0);
        ducky.backLeft.setPower(0);
        ducky.frontRight.setPower(0);
        ducky.backRight.setPower(0);
    }
}
