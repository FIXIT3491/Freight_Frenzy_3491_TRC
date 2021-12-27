package Old_Repo.Competition.z_SetUp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;


@TeleOp(name="Encoder Value Finder", group="Setup")

public class EncoderValueFinder extends OpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    /**
     * Initializing the Program
     */
    @Override
    public void init() {

        // Initialize all motors/ servos
        ducky.init(hardwareMap, telemetry);

        // Indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * TeleOp loop once Driver hits "Play."
     */
    @Override
    public void loop() {

        // Reset Encoder Value for motor
        if (gamepad1.a) {
            ducky.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            ducky.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            ducky.armRotator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Telemetry Update
        telemetry.addData("Back Left Encoder Pulses",   ducky.backLeft.getCurrentPosition());
        telemetry.addData("Back Right Encoder Pulses",   ducky.backRight.getCurrentPosition());
        telemetry.addData("Arm Rotator Encoder Pulses",   ducky.armRotator.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Code to run after Driver hits "Stop."
     */
    @Override
    public void stop() {
    }
}