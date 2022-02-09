package teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test Specific Mechanism")
public class Test_Specific_Mechanism extends OpMode {

    private CRServo carouselSpinner = null;


    @Override
    public void init() {
        carouselSpinner = hardwareMap.crservo.get("carouselSpinner");
    }

    /**
     * TeleOp loop once Driver hits "Play."
     */
    @Override
    public void loop() {
        carouselSpinner.setPower(1);
    }

}
