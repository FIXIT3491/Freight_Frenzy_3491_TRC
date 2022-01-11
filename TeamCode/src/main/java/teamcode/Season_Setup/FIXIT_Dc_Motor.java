package teamcode.Season_Setup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import TrcFtcLib.ftclib.FtcDcMotor;

public class FIXIT_Dc_Motor extends FtcDcMotor
{
    private final FtcDcMotor slaveMotor;
    private static final double BACK_WHEEL_POWER_REDUCTION = 0.9449;

    public FIXIT_Dc_Motor(String motorName, String slaveWheel)
    {
        super(motorName);
        slaveMotor = new FtcDcMotor(slaveWheel);
    }

    @Override
    public void set(double power)
    {
        super.set(power*BACK_WHEEL_POWER_REDUCTION);
        slaveMotor.set(power);
    }

    public void setMode(DcMotor.RunMode runMode)
    {
        super.motor.setMode(runMode);
        slaveMotor.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void setInverted (boolean inverted)
    {
        super.setInverted(inverted);
        slaveMotor.setInverted(inverted);
    }

    @Override
    public void setBrakeModeEnabled (boolean enabled)
    {
        super.setBrakeModeEnabled(enabled);
        slaveMotor.setBrakeModeEnabled(enabled);
    }
}
