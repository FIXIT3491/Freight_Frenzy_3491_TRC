package Old_Repo.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;
import org.firstinspires.ftc.teamcode.Season_Setup.Freight_Frenzy_Pipeline;


@Disabled
@Autonomous(name="HubDuckWarehouse - P2, Blue", group="Competition - Blue")

public class HubDuckWarehouse_P2_Blue extends LinearOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all motors/ servos
        ducky.init(hardwareMap, telemetry);
        Freight_Frenzy_Pipeline freight_frenzy_pipeline = new Freight_Frenzy_Pipeline();

        // Reset Encoders, and Telemetry Update
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // Encoder Position Update
        telemetry.addData("Encoder Position",  "Starting Encoder Position",
                ducky.backLeft.getCurrentPosition(),
                ducky.backRight.getCurrentPosition());
        telemetry.update();

        // Setting Alliance Colour for TeleOp
        ducky.writeAndRead("Blue");

        // Telemetry Update
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("Alliance", Ducky.alliance);
        telemetry.update();

        // Waiting for the program to start
        waitForStart();

        //----------------------------------------------------------------------------------------------
        // Autonomous Pathing
        //----------------------------------------------------------------------------------------------

        // Detecting which autonomous state to run
        if (Freight_Frenzy_Pipeline.positionOfTeamShippingElement == Freight_Frenzy_Pipeline.Pipeline.ElementPosition.LEFT) {
            ducky.leftState = true;
        } else if (Freight_Frenzy_Pipeline.positionOfTeamShippingElement == Freight_Frenzy_Pipeline.Pipeline.ElementPosition.CENTER) {
            ducky.centerState = true;
        } else if (Freight_Frenzy_Pipeline.positionOfTeamShippingElement == Freight_Frenzy_Pipeline.Pipeline.ElementPosition.RIGHT) {
            ducky.rightState = true;
        }

        // Drive backwards
        ducky.driveBackward_Encoder(1,-0.3, 5000);

        // Move arm to position
        if (ducky.leftState) {
            ducky.armBottomLevel();
        } else if (ducky.centerState) {
            ducky.armMidLevel();
        } else if (ducky.rightState) {
            ducky.armTopLevel();
        }

        // Turn Towards Alliance Specific Shipping Hub
        ducky.turn_P(-90,3000, 1000);

        // Move closer to Shipping Hub and score
        ducky.driveBackward_Encoder(2,0.5, 5000);
        ducky.collectorReverse();
        Thread.sleep(1000);
        ducky.collectorOff();

        // Move arm back to collecting position
        ducky.armCollecting();

        // Turn and drive to carousel
        ducky.driveForward_Encoder(10,0.5,5000);
        ducky.armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ducky.turn_P(-90,3000, 1000);
        ducky.driveBackward_Encoder(10,0.5,5000);

        // Spin Carousel
        ducky.carouselSpinnerBlue();
        Thread.sleep(4000);

        // Turn and drive into Warehouse unit
        ducky.driveForward_Encoder(10,1,5000);
        ducky.turn_P(-90,3000, 1000);
        ducky.driveForward_Encoder(50,1,6000);
        ducky.turn_P(-45,3000, 1000);
        ducky.driveForward_Power(1);
        Thread.sleep(3000);
    }
}
