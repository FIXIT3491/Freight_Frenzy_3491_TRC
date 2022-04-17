/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.Season_Setup;

import org.opencv.core.Point;

import java.util.ArrayList;

import TrcCommonLib.trclib.TrcPath;
import TrcCommonLib.trclib.TrcSimpleDriveBase;
import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcGyro;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcWaypoint;
import TrcFtcLib.ftclib.FtcBNO055Imu;


/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
@SuppressWarnings("unused")
public class RobotDrive
{
    // Sensors
    public final FtcBNO055Imu imu;
    public final TrcGyro gyro;


    // Drive motors
    public final FIXIT_Dc_Motor leftWheels;
    public final FIXIT_Dc_Motor rightWheels;

    // Drive Base
    public final TrcDriveBase driveBase;
    public final TrcPidController encoderYPidCtrl;
    public final TrcPidController gyroPidCtrl;
    public final TrcPidDrive pidDrive;
    public final TrcPurePursuitDrive purePursuitDrive;


    // Coefficients for PID controllers.
    public final TrcPidController.PidCoefficients yPosPidCoeff;
    public final TrcPidController.PidCoefficients turnPidCoeff;
    public final TrcPidController.PidCoefficients velPidCoeff;


    /**
     * Constructor: Create an instance of the object.
     */
    public RobotDrive(Robot robot)
    {
        //// Robot Drivebase Setup
        // Sensors
        imu = new FtcBNO055Imu(RobotParams.HWNAME_IMU);
        gyro = imu.gyro;

        // Creating Drivebase Motors
        leftWheels = new FIXIT_Dc_Motor(RobotParams.HWNAME_LEFT_BACK_WHEEL,
                RobotParams.HWNAME_LEFT_FRONT_WHEEL);
        rightWheels = new FIXIT_Dc_Motor(RobotParams.HWNAME_RIGHT_BACK_WHEEL,
                RobotParams.HWNAME_RIGHT_FRONT_WHEEL);

        // Setting Motor Mode
        leftWheels.setMode(RobotParams.DRIVE_MOTOR_MODE);
        rightWheels.setMode(RobotParams.DRIVE_MOTOR_MODE);

        // Setting Motor Direction
        leftWheels.setInverted(RobotParams.LEFT_WHEEL_INVERTED);
        rightWheels.setInverted(RobotParams.RIGHT_WHEEL_INVERTED);

        // Setting Motor Brake mode
        leftWheels.setBrakeModeEnabled(RobotParams.DRIVE_WHEEL_BRAKE_MODE);
        rightWheels.setBrakeModeEnabled(RobotParams.DRIVE_WHEEL_BRAKE_MODE);

        // Creating Drivebase
        driveBase = new TrcSimpleDriveBase(leftWheels, rightWheels, gyro);

        // Creating Odometry Scale
        driveBase.setOdometryScales(RobotParams.ENCODER_Y_INCHES_PER_COUNT);


        // Create and initialize PID controllers
        yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ENCODER_Y_KP, RobotParams.ENCODER_Y_KI, RobotParams.ENCODER_Y_KD);
        turnPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.GYRO_KP, RobotParams.GYRO_KI, RobotParams.GYRO_KD);
        velPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ROBOT_VEL_KP, RobotParams.ROBOT_VEL_KI, RobotParams.ROBOT_VEL_KD, RobotParams.ROBOT_VEL_KF);

        encoderYPidCtrl = new TrcPidController(
            "encoderYPidCtrl", yPosPidCoeff, RobotParams.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
        gyroPidCtrl = new TrcPidController(
            "gyroPidCtrl", turnPidCoeff, RobotParams.GYRO_TOLERANCE, driveBase::getHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);


        // FTC robots generally have USB performance issues where the sampling rate of the gyro is not high enough.
        // If the robot turns too fast, PID will cause oscillation. By limiting turn power, the robot turns slower.
        gyroPidCtrl.setOutputLimit(RobotParams.TURN_POWER_LIMIT);


        // Creating PID Drive
        pidDrive = new TrcPidDrive("pidDrive", driveBase, null, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setTurnMode(TrcPidDrive.TurnMode.CURVE);


        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base
        // is keeping track of the absolute target position.
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setStallDetectionEnabled(RobotParams.PID_DRIVEBASE_STALL_ENABLED);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase,
            RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE, RobotParams.PPD_TURN_TOLERANCE,
            null, yPosPidCoeff, turnPidCoeff, velPidCoeff);
        purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setMsgTracer(robot.globalTracer, true, true);
    }   // RobotDrive

    /**
     * This method cancels any PIDDrive operation still in progress.
     */
    public void cancel()
    {
        if (pidDrive.isActive())
        {
            pidDrive.cancel();
        }

        if (purePursuitDrive.isActive())
        {
            purePursuitDrive.cancel();
        }

        driveBase.stop();
    }   // cancel

    /**
     * This method enables/disables robot base odometry.
     *
     * @param enabled specifies true to enable odometry, false to disable.
     */
    public void setOdometryEnabled(boolean enabled)
    {
        driveBase.setOdometryEnabled(enabled);
    }   // setOdometryEnabled

    /**
     * This method creates a TrcPose2D point in the target path for PurePursuitDrive.
     *
     * @param xTargetLocation specifies the target location in field reference frame.
     * @param yTargetLocation specifies the target location in field reference frame.
     * @param heading specifies the robot end heading.
     * @param tileUnit specifies true if location unit is in floor tile unit, false if in inches unit.
     * @return path point to be used in PurePursuitDrive.
     */
    public TrcPose2D pathPoint(double xTargetLocation, double yTargetLocation, double heading, boolean tileUnit)
    {
        double unitScale = tileUnit? RobotParams.FULL_TILE_INCHES: 1.0;

        return new TrcPose2D(xTargetLocation*unitScale, yTargetLocation*unitScale, heading);
    }   // pathPoint

    /**
     * This method creates a TrcPose2D point in the target path for PurePursuitDrive.
     *
     * @param xTargetLocation specifies the target location in field reference frame.
     * @param yTargetLocation specifies the target location in field reference frame.
     * @param heading specifies the robot end heading.
     * @return path point to be used in PurePursuitDrive.
     */
    public TrcPose2D pathPoint(double xTargetLocation, double yTargetLocation, double heading)
    {
        return pathPoint(xTargetLocation, yTargetLocation, heading, true);
    }   // pathPoint

    /**
     * This method creates a TrcPose2D point in the target path for PurePursuitDrive.
     *
     * @param targetLocation specifies the target location in field reference frame.
     * @param heading specifies the robot end heading.
     * @param tileUnit specifies true if location unit is in floor tile unit, false if in inches unit.
     * @return path point to be used in PurePursuitDrive.
     */
    public TrcPose2D pathPoint(Point targetLocation, double heading, boolean tileUnit)
    {
        return pathPoint(targetLocation.x, targetLocation.y, heading, tileUnit);
    }   // pathPoint

    /**
     * This method creates a TrcPose2D point in the target path for PurePursuitDrive.
     *
     * @param targetLocation specifies the target location in field reference frame.
     * @param heading specifies the robot end heading.
     * @return path point to be used in PurePursuitDrive.
     */
    public TrcPose2D pathPoint(Point targetLocation, double heading)
    {
        return pathPoint(targetLocation.x, targetLocation.y, heading, true);
    }   // pathPoint

    /**
     * This method adds a series of intermediate points for a given end point so that the robot will turn only
     * angleInc at each intermediate point.
     *
     * @param points specifies the points list to add intermediate points to.
     * @param endPoint specifies the new segment end point.
     * @param angleInc specifies the angle increment between intermediate points.
     */
    private void addIntermediatePoints(ArrayList<TrcPose2D> points, TrcPose2D endPoint, double angleInc)
    {
        if (points.size() == 0)
        {
            points.add(endPoint);
        }
        else
        {
            TrcPose2D startPoint = points.get(points.size() - 1);
            TrcPose2D relPose = endPoint.relativeTo(startPoint);
            int numPoints = (int) Math.round(Math.abs(relPose.angle/angleInc));
            double xDelta = relPose.x/numPoints;
            double yDelta = relPose.y/numPoints;
            double angleDelta = relPose.angle/numPoints;

            for (int i = 1; i < numPoints; i++)
            {
                points.add(startPoint.addRelativePose(new TrcPose2D(xDelta*i, yDelta*i, angleDelta*i)));
            }

            points.add(endPoint);
        }
    }   // addIntermediatePoints

    /**
     * This method builds a path with the given list of poses. It will also insert intermediate points between poses
     * so that the robot will only turn up to angleInc between intermediate points.
     *
     * @param angleInc specifies the angle increment between intermediate points.
     * @param incrementalPath specifies true if the poses are incremental from the their poses.
     * @param poses specifies an array of poses used to build the path.
     * @return path built.
     */
    public TrcPath buildPath(double angleInc, boolean incrementalPath, TrcPose2D... poses)
    {
        final String funcName = "buildPath";
        ArrayList<TrcPose2D> points = new ArrayList<>();

        addIntermediatePoints(points, driveBase.getFieldPosition(), angleInc);
        for (TrcPose2D pose: poses)
        {
            if (incrementalPath)
            {
                pose = points.get(points.size() - 1).addRelativePose(pose);
            }
            addIntermediatePoints(points, pose, angleInc);
        }

        TrcWaypoint[] waypoints = new TrcWaypoint[points.size()];
        for (int i = 0; i < points.size(); i++)
        {
            waypoints[i] = new TrcWaypoint(points.get(i), null);
        }

        return new TrcPath(waypoints);
    }   // buildPath
}   // class RobotDrive
