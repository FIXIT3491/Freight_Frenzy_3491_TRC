// Basic Tank Drive program with a power limiter.

package Old_Repo.DriverBots;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Disabled here as it is not wanted when downloading programs to competition robot
@Disabled

@TeleOp
public class TankDrive extends LinearOpMode {

    // Create Instance of ElapsedTime
    private final ElapsedTime runtime = new ElapsedTime();

    // Declaring Motor Variables
    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;

    // Variable Declaration
    boolean slowButton;

    @Override
    public void runOpMode() {

        // Telemetry Update
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Setting Motor Variables
        frontLeft  = hardwareMap.dcMotor.get("frontL");
        frontRight = hardwareMap.dcMotor.get("frontR");
        backLeft  = hardwareMap.dcMotor.get("backL");
        backRight = hardwareMap.dcMotor.get("backR");

        // Setting Motor Direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Wait to start program
        waitForStart();
        runtime.reset();

        // Continues running until program is stopped
        while (opModeIsActive()) {

            // Initializing Joystick Control
            float leftPower = powerLimit(-gamepad1.left_stick_y);
            float rightPower = powerLimit(-gamepad1.right_stick_y);

            //// Driving Controls
            // Slow Button
            if (gamepad1.left_trigger > 0) {
                frontLeft.setPower(leftPower/2);
                frontRight.setPower(rightPower/2);
                backLeft.setPower(leftPower/2);
                backRight.setPower(rightPower/2);

                slowButton = true;

            //Normal Driving
            } else {
                frontLeft.setPower(leftPower);
                frontRight.setPower(rightPower);
                backLeft.setPower(leftPower);
                backRight.setPower(rightPower);

                slowButton = false;
            }

            // Telemetry Update
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Slow Button Enabled", slowButton);
            telemetry.addData("Left Side Power", leftPower);
            telemetry.addData("Right Side Power", rightPower);
            telemetry.update();
        }
    }

    /**
     *     Method to limit motor power if needed
     */
    private float powerLimit(float input)
    {
        // The Minimum and Maximum power of the motors
        final float MIN_POWER = 0.0f;
        final float MAX_POWER = 1.0f;

        // Detect if Inputted power is at limit(s), and limits them if needed
        if (Math.abs(input) >= MIN_POWER) {
            if (input >= 0) {
                return Range.clip(input, MIN_POWER, MAX_POWER);
            } else {
                return Range.clip(input, -MAX_POWER, -MIN_POWER);
            }
        }
        else
        {
            return 0.0f;
        }
    }
}