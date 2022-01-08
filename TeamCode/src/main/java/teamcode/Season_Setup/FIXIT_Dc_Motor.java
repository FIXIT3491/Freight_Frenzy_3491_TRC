package teamcode.Season_Setup;

import TrcFtcLib.ftclib.FtcDcMotor;

public class FIXIT_Dc_Motor extends FtcDcMotor
{
    private FtcDcMotor slaveMotor;
    private static final double BACK_WHEEL_POWER_REDUCTION = 0.7559;

    public FIXIT_Dc_Motor(String motorName, String slaveWheel)
    {
        super(motorName);
        slaveMotor = new FtcDcMotor(slaveWheel);
    }

    @Override
    public void set(double power)
    {
        super.set(power);
        slaveMotor.set(power*BACK_WHEEL_POWER_REDUCTION);
    }
}
