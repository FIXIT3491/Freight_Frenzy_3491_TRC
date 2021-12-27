// Basic Mecanum Drive program with a power limiter.

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
public class MecanumDrive extends LinearOpMode {

    // Create Instance of ElapsedTime
    private final ElapsedTime runtime = new ElapsedTime();

    // Declaring Motor Variables
    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;

    // Declaring variables
    float pivot;
    float horizontal;
    float vertical;
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
            pivot = powerLimit(-gamepad1.right_stick_x);
            horizontal = powerLimit(gamepad1.left_stick_x);
            vertical = powerLimit(-gamepad1.left_stick_y);

            //// Driving Controls
            // Slow Button
            if (gamepad1.left_trigger > 0) {
                frontLeft.setPower((pivot + (vertical + horizontal)) /2 );
                frontRight.setPower((-pivot + (vertical - horizontal)) /2 );
                backLeft.setPower((pivot + (vertical - horizontal)) /2 );
                backRight.setPower((-pivot + (vertical + horizontal)) /2 );

                slowButton = true;

            //Normal Driving
            } else {
                frontLeft.setPower((pivot + (vertical + horizontal)));
                frontRight.setPower((-pivot + (vertical - horizontal)));
                backLeft.setPower((pivot + (vertical - horizontal)));
                backRight.setPower((-pivot + (vertical + horizontal)));

                slowButton = false;
            }

            // Telemetry Update
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Slow Button Enabled", slowButton);
            telemetry.addData("Motors", "horizontal (%.2f), vertical (%.2f), pivot (%.2f)", horizontal, vertical, pivot);
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
        } else {
            return 0.0f;
        }
    }
}