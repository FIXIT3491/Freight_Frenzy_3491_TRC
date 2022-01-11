package teamcode.Season_Setup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import TrcFtcLib.ftclib.FtcDcMotor;

public class FIXIT_Dc_Motor extends FtcDcMotor
{
    private final FtcDcMotor slaveMotor;
    private static final double BACK_WHEEL_POWER_REDUCTION = 0.9449;


    /**
     * Create a set of motors to be connected.
     * @param motorName Primary Motor (Cart Wheel)
     * @param slaveWheel Secondary Motor (Omni Wheel)
     */
    public FIXIT_Dc_Motor(String motorName, String slaveWheel)
    {
        super(motorName);
        slaveMotor = new FtcDcMotor(slaveWheel);
    }

    /**
     * Setting the power reduction to Cart wheel for both wheels to spin at the same Tangential
     * Velocity.
     * @param power Power Reduction
     */
    @Override
    public void set(double power)
    {
        super.set(power*BACK_WHEEL_POWER_REDUCTION);
        slaveMotor.set(power);
    }

    /**
     * Setting the runMode for the Primary Motor.
     * @param runMode Primary Motor's runMode
     */
    public void setMode(DcMotor.RunMode runMode)
    {
        super.motor.setMode(runMode);
        slaveMotor.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set the motor direction.
     * @param inverted specifies true to invert motor direction, false otherwise
     */
    @Override
    public void setInverted (boolean inverted)
    {
        super.setInverted(inverted);
        slaveMotor.setInverted(inverted);
    }

    /**
     * Set the motor brake mode.
     * @param enabled specifies true to enable brake mode, false otherwise
     */
    @Override
    public void setBrakeModeEnabled (boolean enabled)
    {
        super.setBrakeModeEnabled(enabled);
        slaveMotor.setBrakeModeEnabled(enabled);
    }
}
