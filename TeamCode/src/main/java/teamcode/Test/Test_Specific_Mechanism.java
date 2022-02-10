package teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="Test Specific Mechanism", group="Test")

public class Test_Specific_Mechanism extends OpMode {

    // Initializing motor/ servo
    private CRServo carouselSpinner = null;

    /**
     * Initializing the Program
     */
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
