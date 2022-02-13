package teamcode.Season_Setup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RobotFunctions {

    // Class Setup
    protected Robot robot;


    //// Arm System ////

    // Move arm to Collecting level
//    public void armCollecting() {
//        armRotator.setTargetPosition(ARM_COLLECTING_ENCODER_PULSES);
//        armRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        armRotator.setPower(0.5);
//
//        if (armRotator.isBusy()) {
//            if (armRotator.getCurrentPosition() < 50) {
//                telemetry.addData("Reached Collecting Position, Arm Encoder Pulses",
//                        armRotator.getCurrentPosition());
//                telemetry.update();
//
//                armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            } else {
//                telemetry.addData("Arm Rotating, Target Position",
//                        ARM_COLLECTING_ENCODER_PULSES);
//                telemetry.addData("Arm Encoder Pulses",
//                        armRotator.getCurrentPosition());
//                telemetry.update();
//            }
//        }
//    }

    // Move arm to Bottom Hub Level
//    public void armBottomLevel() {
//        armRotator.setTargetPosition(ARM_BOTTOM_LEVEL_ENCODER_PULSES);
//        armRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        armRotator.setPower(0.5);
//
//        if (armRotator.isBusy()) {
//            telemetry.addData("Arm Rotating, Target Position",
//                    ARM_BOTTOM_LEVEL_ENCODER_PULSES);
//            telemetry.addData("Arm Encoder Pulses",
//                    armRotator.getCurrentPosition());
//            telemetry.update();
//        }
//    }

    // Move arm to Middle Hub Level
//    public void armMidLevel() {
//        armRotator.setTargetPosition(ARM_MID_LEVEL_ENCODER_PULSES);
//        armRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        armRotator.setPower(0.5);
//
//        if (armRotator.isBusy()) {
//            armRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            telemetry.addData("Arm Rotating, Target Position",
//                    ARM_MID_LEVEL_ENCODER_PULSES);
//            telemetry.addData("Arm Encoder Pulses",
//                    armRotator.getCurrentPosition());
//            telemetry.update();
//        }
//    }

    // Move arm to Top Hub Level
//    public void armTopLevel() {
//        armRotator.setTargetPosition(ARM_TOP_LEVEL_ENCODER_PULSES);
//        armRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        armRotator.setPower(0.5);
//
//        if (armRotator.isBusy()) {
//            telemetry.addData("Arm Rotating, Target Position",
//                    ARM_TOP_LEVEL_ENCODER_PULSES);
//            telemetry.addData("Arm Encoder Pulses",
//                    armRotator.getCurrentPosition());
//            telemetry.update();
//        }
//    }

    // Rotate Arm platform to the Front of the robot
//    public void armPlatform_Hub_Front() {
//        armPlatformRotator.setTargetPosition(ARM_PLATFORM_ROTATOR_HUB_FRONT);
//        armPlatformRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        armPlatformRotator.setPower(0.5);
//
//        if (armPlatformRotator.isBusy()) {
//            telemetry.addData("Arm Platform Rotating, Target Position",
//                    ARM_PLATFORM_ROTATOR_HUB_BACK);
//            telemetry.addData("Arm Platform Encoder Pulses",
//                    armPlatformRotator.getCurrentPosition());
//            telemetry.update();
//        }
//    }

    // Rotate Arm platform to the Back of the robot
//    public void armPlatform_Hub_Back() {
//        armPlatformRotator.setTargetPosition(ARM_PLATFORM_ROTATOR_HUB_BACK);
//        armPlatformRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        armPlatformRotator.setPower(0.5);
//
//        if (armPlatformRotator.isBusy()) {
//            telemetry.addData("Arm Platform Rotating, Target Position",
//                    ARM_PLATFORM_ROTATOR_HUB_FRONT);
//            telemetry.addData("Arm Platform Encoder Pulses",
//                    armPlatformRotator.getCurrentPosition());
//            telemetry.update();
//        }
//    }


    //// Duck System ////

    // Carousel Spinner
    public void carouselSpinnerBlue(){
        robot.carouselSpinner.setPosition(RobotParams.CAROUSEL_SPINNER_BLUE);
        robot.robotDrive.driveBase.tankDrive(0.2, 0.2);
    }
    public void carouselSpinnerRed(){
        robot.carouselSpinner.setPosition(RobotParams.CAROUSEL_SPINNER_RED);
        robot.robotDrive.driveBase.tankDrive(0.2, 0.2);
    }
    public void carouselSpinnerOff(){
        robot.carouselSpinner.setPosition(RobotParams.CAROUSEL_SPINNER_STOP_POWER);
    }

    // Carousel Spinner Rotator
//    public void rotateToBlue(){
//        carouselSpinnerRotator.setTargetPosition(DUCKY_SPINNER_ROTATOR_BLUE);
//        carouselSpinnerRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        carouselSpinnerRotator.setPower(0.5);
//
//        if (carouselSpinnerRotator.isBusy()) {
//            telemetry.addData("Ducky Spinner Platform Rotating, Target Position",
//                    DUCKY_SPINNER_ROTATOR_BLUE);
//            telemetry.addData("Ducky Spinner Rotator Encoder Pulses",
//                    carouselSpinnerRotator.getCurrentPosition());
//            telemetry.update();
//        }
//    }
//    public void rotateToRed(){
//        carouselSpinnerRotator.setTargetPosition(DUCKY_SPINNER_ROTATOR_RED);
//        carouselSpinnerRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        carouselSpinnerRotator.setPower(0.5);
//
//        if (carouselSpinnerRotator.isBusy()) {
//            telemetry.addData("Ducky Spinner Platform Rotating, Target Position",
//                    DUCKY_SPINNER_ROTATOR_RED);
//            telemetry.addData("Ducky Spinner Rotator Encoder Pulses",
//                    carouselSpinnerRotator.getCurrentPosition());
//            telemetry.update();
//        }
//    }
//    public void rotateToMiddle(){
//        carouselSpinnerRotator.setTargetPosition(DUCKY_SPINNER_ROTATOR_MIDDLE);
//        carouselSpinnerRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        carouselSpinnerRotator.setPower(0.5);
//
//        if (carouselSpinnerRotator.isBusy()) {
//            telemetry.addData("Ducky Spinner Platform Rotating, Target Position",
//                    DUCKY_SPINNER_ROTATOR_MIDDLE);
//            telemetry.addData("Ducky Spinner Rotator Encoder Pulses",
//                    carouselSpinnerRotator.getCurrentPosition());
//            telemetry.update();
//        }
//    }

    //----------------------------------------------------------------------------------------------
    // Miscellaneous Functions
    //----------------------------------------------------------------------------------------------

    public void writeAndRead (String allianceColour) {
        File_WriteAndRead.writeToFile(allianceColour);
        File_WriteAndRead.readFromFile();
    }
}
