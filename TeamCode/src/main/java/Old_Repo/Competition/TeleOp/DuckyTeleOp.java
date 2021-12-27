package Old_Repo.Competition.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;


@TeleOp(name="Competition TeleOp", group="Competition")

public class DuckyTeleOp extends OpMode {

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
        telemetry.addData("Alliance", Ducky.alliance);
        telemetry.update();
    }

    /**
     * TeleOp loop once Driver hits "Play."
     */
    @Override
    public void loop() {

        // Initializing Joystick Control
        float leftPower = -gamepad1.left_stick_y;
        float rightPower = -gamepad1.right_stick_y;


        //------------------------------------------------------------------------------------------
        // Driving Controls
        //------------------------------------------------------------------------------------------

        // Reverse Drive
        if (gamepad1.left_trigger > 0) {

            // Telemetry Update
            telemetry.addData("Drive Mode: ", "Reverse");

            // Slow Speed
            if (gamepad1.right_trigger > 0) {
                ducky.frontLeft.setPower(rightPower/-2);
                ducky.backLeft.setPower((rightPower/-2)* Ducky.BACK_WHEEL_POWER_REDUCTION);
                ducky.frontRight.setPower(leftPower/-2);
                ducky.backRight.setPower((leftPower/-2)* Ducky.BACK_WHEEL_POWER_REDUCTION);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Slow");

            // Normal Speed
            } else {
                ducky.frontLeft.setPower(-rightPower);
                ducky.backLeft.setPower((-rightPower)* Ducky.BACK_WHEEL_POWER_REDUCTION);
                ducky.frontRight.setPower(-leftPower);
                ducky.backRight.setPower((-leftPower)* Ducky.BACK_WHEEL_POWER_REDUCTION);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Normal");
            }

        // Forward Drive
        } else {

            // Telemetry Update
            telemetry.addData("Drive Mode: ", "Forward");

            // Slow Speed
            if (gamepad1.right_trigger > 0) {
                ducky.frontLeft.setPower(leftPower/2);
                ducky.backLeft.setPower((leftPower/2)* Ducky.BACK_WHEEL_POWER_REDUCTION);
                ducky.frontRight.setPower(rightPower/2);
                ducky.backRight.setPower((rightPower/2)* Ducky.BACK_WHEEL_POWER_REDUCTION);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Slow");

            // Normal Speed
            } else {
                ducky.frontLeft.setPower(leftPower);
                ducky.backLeft.setPower((leftPower)* Ducky.BACK_WHEEL_POWER_REDUCTION);
                ducky.frontRight.setPower(rightPower);
                ducky.backRight.setPower((rightPower)* Ducky.BACK_WHEEL_POWER_REDUCTION);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Normal");
            }
        }


        //------------------------------------------------------------------------------------------
        // Mechanisms
        //------------------------------------------------------------------------------------------

        // Collector
        if (gamepad2.right_trigger > 0) {
            ducky.collectorOn();
        }
        if (gamepad2.right_bumper) {
            ducky.collectorReverse();
        }
        if (gamepad2.a) {
            ducky.collectorOff();
        }

        // Arm Rotator
        if (gamepad2.left_stick_y == 0) {
            if (gamepad2.dpad_up) {
                ducky.armCollecting();
            } else if (gamepad2.dpad_left) {
                ducky.armTopLevel();
            } else if (gamepad2.dpad_right) {
                ducky.armMidLevel();
            } else if (gamepad2.dpad_down) {
                ducky.armBottomLevel();
            }
        } else {
            ducky.rotateArm(gamepad2.left_stick_y);
        }


        // Carousel Spinner
        if (Ducky.alliance.equals("Blue")) {
            if (gamepad2.left_trigger > 0) {
                ducky.carouselSpinnerBlue();
            } else if (gamepad2.left_bumper) {
                ducky.carouselSpinnerOff();
            }
        } else if (Ducky.alliance.equals("Red"))  {
            if (gamepad2.left_trigger > 0) {
                ducky.carouselSpinnerRed();
            } else if (gamepad2.left_bumper) {
                ducky.carouselSpinnerOff();
            }
        }


        // Telemetry Update
        telemetry.addData("Left Side Power", leftPower);
        telemetry.addData("Right Side Power", rightPower);

        telemetry.addData("Arm Target Position",
                Ducky.ARM_COLLECTING_ENCODER_PULSES);
        telemetry.addData("Arm Encoder Pulses",
                ducky.armRotator.getCurrentPosition());

        telemetry.update();
    }

    /**
     * Code to run after Driver hits "Stop."
     */
    public void stop() {
    }
}
