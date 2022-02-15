package teamcode.Season_Setup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RobotFunctions {

    // Class Setup
    protected Robot robot;


    //// Arm System ////


    //// Duck System ////

    // Carousel Spinner
    public void carouselSpinnerBlue()
    {
        robot.carouselSpinner.setPosition(RobotParams.CAROUSEL_SPINNER_BLUE);
        robot.robotDrive.driveBase.tankDrive(0.2, 0.2);
    }
    public void carouselSpinnerRed()
    {
        robot.carouselSpinner.setPosition(RobotParams.CAROUSEL_SPINNER_RED);
        robot.robotDrive.driveBase.tankDrive(0.2, 0.2);
    }
    public void carouselSpinnerOff()
    {
        robot.carouselSpinner.setPosition(RobotParams.CAROUSEL_SPINNER_STOP_POWER);
    }


    //----------------------------------------------------------------------------------------------
    // Miscellaneous Functions
    //----------------------------------------------------------------------------------------------

    public void writeAndRead (String allianceColour) {
        File_WriteAndRead.writeToFile(allianceColour);
        File_WriteAndRead.readFromFile();
    }
}
