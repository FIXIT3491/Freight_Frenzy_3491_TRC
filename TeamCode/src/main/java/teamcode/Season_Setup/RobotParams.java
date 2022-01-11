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

import android.os.Environment;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This class contains robot and subsystem constants and parameters.
 */
public class RobotParams
{
    /**
     * This class contains robot preferences. It controls enabling/disabling of various robot features.
     */
    public static class Preferences
    {
        public static boolean visionOnly = false;
        public static boolean initSubsystems = true;
        public static boolean useBlinkin = true;
        public static boolean useTraceLog = true;
        public static boolean useBatteryMonitor = false;
        public static boolean useLoopPerformanceMonitor = true;
        public static boolean useVelocityControl = false;

        public static boolean competitionMode = false;

        public static boolean useArm = true;
    }   //class Preferences

    public enum DriveMode
    {
        TANK_MODE,
        ARCADE_MODE
    }   //enum DriveMode

    public static final String LOG_PATH_FOLDER                         =
        Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftc3543";
    //
    // Hardware names.
    //
    public static final String HWNAME_IMU                              = "imu";
    public static final String HWNAME_WEBCAM                           = "Webcam 1";
//    static final String HWNAME_BLINKIN                          = "blinkin";
    public static final String HWNAME_LEFT_FRONT_WHEEL                 = "frontL";
    public static final String HWNAME_RIGHT_FRONT_WHEEL                = "frontR";
    public static final String HWNAME_LEFT_BACK_WHEEL                  = "lbWheel";
    public static final String HWNAME_RIGHT_BACK_WHEEL                 = "rbWheel";

    public static final String HWNAME_ARM                              = "armRotator";

    //
    // Field dimensions.
    //
    public static final double FULL_FIELD_INCHES                       = 141.0;
    public static final double HALF_FIELD_INCHES                       = FULL_FIELD_INCHES/2.0;
    public static final double FULL_TILE_INCHES                        = 23.75;
    public static final double HALF_TILE_INCHES                        = FULL_TILE_INCHES/2.0;
    //
    // Robot dimensions.
    //
    public static final double ROBOT_LENGTH                            = 17.0;
    public static final double ROBOT_WIDTH                             = 17.0;
    //
    // Motor Odometries.
    //
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static final double GOBILDA_5203_312_ENCODER_PPR            = ((((1.0 + (46.0/17.0)))*(1.0 + (46.0/11.0)))*28.0);
    public static final double GOBILDA_5203_312_RPM                    = 312.0;
    public static final double GOBILDA_5203_312_MAX_VELOCITY_PPS       =
        GOBILDA_5203_312_ENCODER_PPR*GOBILDA_5203_312_RPM/60.0; // 2795.987 pps

    // DriveBase subsystem.
    public static final DriveMode ROBOT_DRIVE_MODE                     = DriveMode.TANK_MODE;
    public static final DcMotor.RunMode DRIVE_MOTOR_MODE               = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final boolean LEFT_WHEEL_INVERTED                    = true;
    public static final boolean RIGHT_WHEEL_INVERTED                   = false;
    public static final boolean DRIVE_WHEEL_BRAKE_MODE                 = true;
    public static final double TURN_POWER_LIMIT                        = 0.5;
    public static final double SLOW_DRIVE_POWER_SCALE                  = 0.5;

    // Velocity controlled constants.
    public static final double DRIVE_MOTOR_MAX_VELOCITY_PPS            = GOBILDA_5203_312_MAX_VELOCITY_PPS;

    public static final double ENCODER_X_KP                            = 0.095;
    public static final double ENCODER_X_KI                            = 0.0;
    public static final double ENCODER_X_KD                            = 0.001;
    public static final double ENCODER_X_TOLERANCE                     = 1.0;
    public static final double ENCODER_X_INCHES_PER_COUNT              = 0.01924724265461924299065420560748;

    public static final double ENCODER_Y_KP                            = 0.06;
    public static final double ENCODER_Y_KI                            = 0.0;
    public static final double ENCODER_Y_KD                            = 0.002;
    public static final double ENCODER_Y_TOLERANCE                     = 1.0;
    public static final double ENCODER_Y_INCHES_PER_COUNT              = 0.02166184604662450653409090909091;

    public static final double GYRO_KP                                 = 0.025;
    public static final double GYRO_KI                                 = 0.0;
    public static final double GYRO_KD                                 = 0.001;
    public static final double GYRO_TOLERANCE                          = 2.0;

    public static final double PIDDRIVE_STALL_TIMEOUT                  = 0.2;  //in seconds.
    //
    // Pure Pursuit parameters.
    //
    // No-Load max velocity (i.e. theoretical maximum)
    // goBILDA 5203-312 motor, max shaft speed = 312 RPM
    // motor-to-wheel gear ratio = 1:1
    // max wheel speed = pi * wheel diameter * wheel gear ratio * motor RPM / 60.0
    // = 3.1415926535897932384626433832795 * 4 in. * 1.0 * 312.0 / 60.0
    // = 65.345127194667699360022982372214 in./sec.
    public static final double ROBOT_MAX_VELOCITY                      = 25.0;     // measured maximum from drive speed test.
    public static final double ROBOT_MAX_ACCELERATION                  = 3380.0;   // measured maximum from drive speed test.
    public static final double ROBOT_VEL_KP                            = 0.0;
    public static final double ROBOT_VEL_KI                            = 0.0;
    public static final double ROBOT_VEL_KD                            = 0.0;
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    public static final double ROBOT_VEL_KF                            = 1.0 / ROBOT_MAX_VELOCITY;
    public static final double PPD_FOLLOWING_DISTANCE                  = 6.0;
    public static final double PPD_POS_TOLERANCE                       = 2.0;
    public static final double PPD_TURN_TOLERANCE                      = 1.0;
    //
    // Odometry Wheel Deployer subsystem.
    //
    public static final double ODWHEEL_X_INCHES_PER_COUNT              = 7.6150160901199168116026724971383e-4;
    public static final double ODWHEEL_Y_INCHES_PER_COUNT              = 8.3527984931543701389098271890307e-4;

    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X             = 0.0;
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y             = 0.0;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X            = 639;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y            = 0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X          = 0.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y          = 479;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X         = 639;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y         = 479;

    // These should be in real-world robot coordinates. Needs calibration after camera is actually mounted in position.
    // Measurement unit: inches
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_X              = -22.25;
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y              = 60;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X             = 23;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y             =60;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X           = -8.75;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y           = 16;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X          = 7.5;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y          = 16;
    //
    // Vision subsystem.
    //
    public static final String TRACKABLE_IMAGES_FILE                   = "FreightFrenzy";
    public static final double CAMERA_FRONT_OFFSET                     = 7.5;  //Camera offset from front of robot in inches
    public static final double CAMERA_HEIGHT_OFFSET                    = 16.0; //Camera offset from floor in inches
    public static final double CAMERA_LEFT_OFFSET                      = 8.875;//Camera offset from left of robot in inches

    //
    // Arm subsystem.
    //
    public static final double ARM_KP                                  = 0.2;
    public static final double ARM_KI                                  = 0.0;
    public static final double ARM_KD                                  = 0.0;
    public static final double ARM_TOLERANCE                           = 0.5;
    public static final double ARM_ENCODER_PPR                         = GOBILDA_5203_312_ENCODER_PPR;
    // https://www.gobilda.com/super-duty-worm-drive-pan-kit-28-1-ratio/
    public static final double ARM_GEAR_RATIO                          = 28.0;
    public static final double ARM_DEG_PER_COUNT                       = 360.0/(ARM_ENCODER_PPR*ARM_GEAR_RATIO);
    public static final double ARM_OFFSET                              = 33.0;
    public static final double ARM_MIN_POS                             = 33.0;
    public static final double ARM_MAX_POS                             = 140.0;
    public static final double ARM_TRAVEL_POS                          = ARM_MIN_POS+2.0;
    public static final boolean ARM_MOTOR_INVERTED                     = true;
    public static final boolean ARM_HAS_LOWER_LIMIT_SWITCH             = false;
    public static final boolean ARM_LOWER_LIMIT_INVERTED               = false;
    public static final boolean ARM_HAS_UPPER_LIMIT_SWITCH             = false;
    public static final boolean ARM_UPPER_LIMIT_INVERTED               = false;
    public static final double ARM_CAL_POWER                           = 0.0;
    public static final double ARM_STALL_MIN_POWER                     = 0.3;
    public static final double ARM_STALL_TIMEOUT                       = 1.0;
    public static final double ARM_RESET_TIMEOUT                       = 0.5;
    public static final double[] ARM_PRESET_LEVELS                     = new double[] {ARM_MIN_POS, 51.6, 78, 107};
    public static final double ARM_SLOW_POWER_SCALE                    = 0.5;

}   //class RobotInfo
