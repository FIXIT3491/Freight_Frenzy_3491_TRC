package teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import TrcCommonLib.trclib.TrcPidActuator;
import TrcFtcLib.ftclib.FtcServo;


@TeleOp(name="Test Specific Mechanism", group="Test")

public class Test_Specific_Mechanism extends OpMode
{
    // Initializing motor/ servo
    public CRServo collector;
    public DcMotorEx armExtender;
    public DcMotorEx armRotator;
    public DcMotorEx armPlatformRotator;

    public CRServo carouselSpinner;
    public CRServo carouselExtenderOne;
    public CRServo carouselExtenderTwo;


    /**
     * Initializing the Program
     */
    @Override
    public void init()
    {
//        collector = hardwareMap.crservo.get("collector");
//        armExtender = hardwareMap.get(DcMotorEx.class,"armExtender.motor");
//        armRotator = hardwareMap.get(DcMotorEx.class,"armRotator.motor");
//        armPlatformRotator = hardwareMap.get(DcMotorEx.class,"armPlatformRotator.motor");
//
        carouselSpinner = hardwareMap.crservo.get("carouselSpinner");
//        carouselExtenderOne = hardwareMap.crservo.get("carouselExtenderOne");
        carouselExtenderTwo = hardwareMap.crservo.get("carouselExtenderTwo");

    }

    /**
     * TeleOp loop once Driver hits "Play."
     */
    @Override
    public void loop() 
    {
        if (gamepad1.a)
        {
            carouselSpinner.setPower(1);
//            carouselExtenderOne.setPower(1);
//            carouselExtenderTwo.setPower(1);
        }

        telemetry.addData("Carousel Power:", carouselSpinner.getPower());
        telemetry.update();

//        armRotator.setPower(gamepad2.left_stick_y);
    }
}
