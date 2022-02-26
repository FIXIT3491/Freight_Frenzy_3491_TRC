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

import TrcCommonLib.trclib.TrcPose2D;


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
        // System Preferences
        public static boolean useVision = true;
        public static boolean visionOnly = false;
        public static boolean useBlinkin = false;
        public static boolean useTraceLog = true;
        public static boolean useBatteryMonitor = false;
        public static boolean useLoopPerformanceMonitor = true;

        // Competition Mode
        public static boolean competitionMode = true;

        // Mechanism Preferences
        public static boolean initSubsystems = true;

        public static boolean useArmSystem = true; // Includes: Collector, Arm Extension, Arm Rotator, Arm Platform Rotator
        public static boolean useDuckSystem = true; // Includes: CarouselSpinner, CarouselSpinnerRotator
        public static boolean useTapeMeasure = false; // Includes: Tape Measure

    }   // class Preferences


    // Enum declaration for different drive styles.
    public enum DriveMode
    {
        TANK_MODE,
        ARCADE_MODE
    }   // enum DriveMode


    public static final String LOG_PATH_FOLDER                         =
        Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftc3491";


    //----------------------------------------------------------------------------------------------
    // Robot Parameters
    //----------------------------------------------------------------------------------------------

    // Drivebase
    public static final String HWNAME_LEFT_FRONT_WHEEL                 = "frontL";
    public static final String HWNAME_RIGHT_FRONT_WHEEL                = "frontR";
    public static final String HWNAME_LEFT_BACK_WHEEL                  = "backL";
    public static final String HWNAME_RIGHT_BACK_WHEEL                 = "backR";

    // Mechanism (Motors and Servos)
    public static final String HWNAME_COLLECTOR                        = "collector";
    public static final String HWNAME_ARM_EXTENDER                     = "armExtender";
    public static final String HWNAME_ARM_ROTATOR                      = "armRotator";
    public static final String HWNAME_ARM_PLATFORM_ROTATOR             = "armPlatformRotator";

    public static final String HWNAME_CAROUSEL_SPINNER                 = "carouselSpinner";
    public static final String HWNAME_CAROUSEL_SPINNER_ROTATOR         = "carouselSpinnerRotator";

    public static final String HWNAME_TAPE_MEASURE                     = "tapeMeasure";

    // Sensors
    public static final String HWNAME_IMU                              = "imu";
    public static final String HWNAME_WEBCAM                           = "Webcam 1";
//    static final String HWNAME_BLINKIN                          = "blinkin";


    // Field dimensions
    static final double FULL_FIELD_INCHES                              = 141.0;
    static final double HALF_FIELD_INCHES                              = FULL_FIELD_INCHES/2.0;
    static final double QUAD_FIELD_INCHES                              = FULL_FIELD_INCHES/4.0;
    static final double FULL_TILE_INCHES                               = 23.75;
    static final double HALF_TILE_INCHES                               = FULL_TILE_INCHES/2.0;


    // Robot dimensions
    public static final double ROBOT_LENGTH                            = 17.0;
    public static final double ROBOT_WIDTH                             = 17.0;

    // Game positions.
    static final double STARTPOS_FROM_FIELDCENTER_Y                    = HALF_FIELD_INCHES - ROBOT_LENGTH/2.0;
    static final double STARTPOS_FROM_FIELDCENTER_X1                   = QUAD_FIELD_INCHES;
    static final double STARTPOS_FROM_FIELDCENTER_X2                   = HALF_TILE_INCHES;

    public static final TrcPose2D STARTPOS_RED_NEAR                           =
            new TrcPose2D(-STARTPOS_FROM_FIELDCENTER_X1, -STARTPOS_FROM_FIELDCENTER_Y, 0.0);
    public static final TrcPose2D STARTPOS_RED_FAR                            =
            new TrcPose2D(STARTPOS_FROM_FIELDCENTER_X2, -STARTPOS_FROM_FIELDCENTER_Y, 0.0);
    public static final TrcPose2D STARTPOS_BLUE_NEAR                          =
            new TrcPose2D(-STARTPOS_FROM_FIELDCENTER_X1, STARTPOS_FROM_FIELDCENTER_Y, 180.0);
    public static final TrcPose2D STARTPOS_BLUE_FAR                           =
            new TrcPose2D(STARTPOS_FROM_FIELDCENTER_X2, STARTPOS_FROM_FIELDCENTER_Y, 180.0);

    // Motor Odometries
    // (Drive Motors) - https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static final double GOBILDA_5203_312_ENCODER_PPR            = 537.689839572;
    public static final double GOBILDA_5203_312_RPM                    = 312.0;
    public static final double GOBILDA_5203_312_MAX_VELOCITY_PPS       =
        GOBILDA_5203_312_ENCODER_PPR*GOBILDA_5203_312_RPM/60.0; // 2795.987 pps

    // (Arm Rotator Motor) - https://www.gobilda.com/5204-series-yellow-jacket-planetary-gear-motor-99-5-1-ratio-80mm-length-8mm-rex-shaft-60-rpm-3-3-5v-encoder/
    public static final double GOBILDA_5204_60_ENCODER_PPR             = 2786.21098687;
    public static final double GOBILDA_5204_60_RPM                     = 60.0;
    public static final double GOBILDA_5204_60_MAX_VELOCITY_PPS        =
            GOBILDA_5204_60_ENCODER_PPR*GOBILDA_5204_60_RPM/60.0;

    // (Core Hex) - https://www.revrobotics.com/rev-41-1300/
    public static final double REV_CORE_HEX_ENCODER_PPR                = 288.0;
    public static final double REV_CORE_HEX_RPM                        = 125.0;
    public static final double REV_CORE_MAX_VELOCITY_PPS               =
            REV_CORE_HEX_ENCODER_PPR*REV_CORE_HEX_RPM/60.0;

    // (UltraPlanetary Motor - with 3:1,4:1,5:1) - https://www.revrobotics.com/rev-41-1600/
    public static final double REV_ULTRAPLANETARY_ENCODER_PPR          = 28.0 *3*4*5;
    public static final double REV_ULTRAPLANETARY_RPM                  = 6000.0/ (3*4*5);
    public static final double REV_ULTRAPLANETARY_MAX_VELOCITY_PPS     =
            REV_ULTRAPLANETARY_ENCODER_PPR*REV_ULTRAPLANETARY_RPM/60.0;


    // DriveBase subsystem
    public static final DriveMode ROBOT_DRIVE_MODE                     = DriveMode.TANK_MODE;
    public static final DcMotor.RunMode DRIVE_MOTOR_MODE               = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final boolean LEFT_WHEEL_INVERTED                    = true;
    public static final boolean RIGHT_WHEEL_INVERTED                   = false;
    public static final boolean DRIVE_WHEEL_BRAKE_MODE                 = true;
    public static final double TURN_POWER_LIMIT                        = 0.5;
    public static final double SLOW_DRIVE_POWER_SCALE                  = 0.5;
    public static final boolean PID_DRIVEBASE_STALL_ENABLED            = true;


    // Velocity controlled constants.
    public static final double DRIVE_MOTOR_MAX_VELOCITY_PPS            = GOBILDA_5203_312_MAX_VELOCITY_PPS;

    public static final double ENCODER_Y_KP                            = 0.05; // Previous val - 0.1
    public static final double ENCODER_Y_KI                            = 0.0;
    public static final double ENCODER_Y_KD                            = 0.00; // Previous val - 0.015
    public static final double ENCODER_Y_TOLERANCE                     = 1.0;
    public static final double ENCODER_Y_INCHES_PER_COUNT              = 0.06135823203645178153789678017701;

    public static final double GYRO_KP                                 = 0.003;
    public static final double GYRO_KI                                 = 0.0;
    public static final double GYRO_KD                                 = 0.000;
    public static final double GYRO_TOLERANCE                          = 2.0;

    public static final double PIDDRIVE_STALL_TIMEOUT                  = 0.2;  // in seconds.


    //// Pure Pursuit parameters
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
    public static final double PPD_FOLLOWING_DISTANCE                  = 12.0; // If robot does not turn properly, change to larger value
    public static final double PPD_POS_TOLERANCE                       = 2.0;
    public static final double PPD_TURN_TOLERANCE                      = 1.0;


//    // Homography
//    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X             = 0.0;
//    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y             = 0.0;
//    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X            = 639;
//    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y            = 0;
//    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X          = 0.0;
//    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y          = 479;
//    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X         = 639;
//    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y         = 479;
//
//    // These should be in real-world robot coordinates. Needs calibration after camera is actually mounted in position.
//    // Measurement unit: inches
//    public static final double HOMOGRAPHY_WORLD_TOPLEFT_X              = -22.25;
//    public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y              = 60;
//    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X             = 23;
//    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y             =60;
//    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X           = -8.75;
//    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y           = 16;
//    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X          = 7.5;
//    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y          = 16;
//
//
//    // Vision subsystem.
//    public static final String TRACKABLE_IMAGES_FILE                   = "FreightFrenzy";
//    public static final double CAMERA_FRONT_OFFSET                     = 7.5;   // Camera offset from front of robot in inches
//    public static final double CAMERA_HEIGHT_OFFSET                    = 16.0;  // Camera offset from floor in inches
//    public static final double CAMERA_LEFT_OFFSET                      = 8.875; // Camera offset from left of robot in inches


    //----------------------------------------------------------------------------------------------
    // Mechanism Parameters/ subsystems
    //----------------------------------------------------------------------------------------------

    // Collector
    public static final double COLLECTOR_PICKUP_POWER                  = 0.0;
    public static final double COLLECTOR_DEPOSIT_POWER                 = 1.0;
    public static final double COLLECTOR_STOP_POWER                    = 0.5;
    public static final double COLLECTOR_PICKUP_TIME                   = 0.0;
    public static final double COLLECTOR_DEPOSITING_TIME               = 2.0;

    // Arm Extender
    public static final double ARM_EXTENDER_KP                         = 0.5;
    public static final double ARM_EXTENDER_KI                         = 0.0;
    public static final double ARM_EXTENDER_KD                         = 0.0;
    public static final double ARM_EXTENDER_TOLERANCE                  = 0.2;
    public static final double ARM_EXTENDER_ENCODER_PPR                = REV_CORE_HEX_ENCODER_PPR;
    // https://www.revrobotics.com/rev-41-1300/
    public static final double ARM_EXTENDER_SPROCKET_DIAMETER          = 1.952853882; // in inches
    public static final double ARM_EXTENDER_INCHES_PER_COUNT           = Math.PI * ARM_EXTENDER_SPROCKET_DIAMETER/ARM_EXTENDER_ENCODER_PPR;
    public static final double ARM_EXTENDER_OFFSET                     = 0.0;
    public static final double ARM_EXTENDER_MIN_POS                    = 0.0; // in inches
    public static final double ARM_EXTENDER_MAX_POS                    = 10; // in inches (true max = 8.5625)
    public static final boolean ARM_EXTENDER_MOTOR_INVERTED            = false;
    public static final boolean ARM_EXTENDER_HAS_LOWER_LIMIT_SWITCH    = false;
    public static final boolean ARM_EXTENDER_LOWER_LIMIT_INVERTED      = false;
    public static final boolean ARM_EXTENDER_HAS_UPPER_LIMIT_SWITCH    = false;
    public static final boolean ARM_EXTENDER_UPPER_LIMIT_INVERTED      = false;
    public static final double ARM_EXTENDER_CAL_POWER                  = 0.5; // Calibration Power
    public static final double ARM_EXTENDER_STALL_MIN_POWER            = 0.45;
    public static final double ARM_EXTENDER_STALL_TIMEOUT              = 0.5;
    public static final double ARM_EXTENDER_RESET_TIMEOUT              = 0.5;
    public static final double[] ARM_EXTENDER_PRESET_LENGTH            = new double[] {ARM_EXTENDER_MIN_POS, 3, 6, 8};
    public static final double ARM_EXTENDER_SLOW_POWER_SCALE           = 0.5;

    // Arm Rotator subsystem
    public static final double ARM_ROTATOR_KP                          = 0.03;
    public static final double ARM_ROTATOR_KI                          = 0.0;
    public static final double ARM_ROTATOR_KD                          = 0.0;
    public static final double ARM_ROTATOR_TOLERANCE                   = 5.0;
    public static final double ARM_ROTATOR_ENCODER_PPR                 = GOBILDA_5204_60_ENCODER_PPR;
    // https://www.gobilda.com/5204-series-yellow-jacket-planetary-gear-motor-99-5-1-ratio-80mm-length-8mm-rex-shaft-60-rpm-3-3-5v-encoder/
    public static final double ARM_ROTATOR_GEAR_RATIO                  = 1.0;
    public static final double ARM_ROTATOR_DEG_PER_COUNT               = 360.0/(ARM_ROTATOR_ENCODER_PPR * ARM_ROTATOR_GEAR_RATIO);
    public static final double ARM_ROTATOR_OFFSET                      = 7.4; // (True value is 27.4) Changed by around 16 degrees after moving up and down for the first time.
    public static final double ARM_ROTATOR_OFFSET_TELEOP               = 35.7; // (True value is 55.7) Changed by around 16 degrees after moving up and down for the first time.
    public static final double ARM_ROTATOR_MIN_POS                     = 54.7;
    public static final double ARM_ROTATOR_MAX_POS                     = 120.0;
    public static final boolean ARM_ROTATOR_MOTOR_INVERTED             = true;
    public static final boolean ARM_ROTATOR_HAS_LOWER_LIMIT_SWITCH     = false;
    public static final boolean ARM_ROTATOR_LOWER_LIMIT_INVERTED       = false;
    public static final boolean ARM_ROTATOR_HAS_UPPER_LIMIT_SWITCH     = false;
    public static final boolean ARM_ROTATOR_UPPER_LIMIT_INVERTED       = false;
    public static final double ARM_ROTATOR_MAX_GRAVITY_COMPENSATION    = 0.1;
    public static final double ARM_ROTATOR_STALL_MIN_POWER             = ARM_ROTATOR_MAX_GRAVITY_COMPENSATION+.3;
    public static final double ARM_ROTATOR_STALL_TIMEOUT               = 0.5;
    public static final double ARM_ROTATOR_RESET_TIMEOUT               = 0.5;
    public static final double ARM_ROTATOR_CAL_POWER                   = ARM_ROTATOR_STALL_MIN_POWER+0.05;
    public static final double[] ARM_ROTATOR_PRESET_LEVELS             = new double[] {ARM_ROTATOR_MIN_POS, 60, 80, 108};
    public static final double ARM_ROTATOR_SLOW_POWER_SCALE            = 0.375;
    public static final double ARM_ROTATOR_LOWERING_ARM_POWER_SCALE    = 4.0;


    // Arm Platform Rotator
    public static final double ARM_PLATFORM_ROTATOR_KP                 = 0.005;
    public static final double ARM_PLATFORM_ROTATOR_KI                 = 0.0;
    public static final double ARM_PLATFORM_ROTATOR_KD                 = 0.0;
    public static final double ARM_PLATFORM_ROTATOR_TOLERANCE          = 2.0;
    public static final double ARM_PLATFORM_ROTATOR_ENCODER_PPR        = REV_ULTRAPLANETARY_ENCODER_PPR;
    // https://www.revrobotics.com/rev-41-1600/
    public static final double ARM_PLATFORM_ROTATOR_GEAR_RATIO         = 1.0;
    public static final double ARM_PLATFORM_ROTATOR_DEG_PER_COUNT      = 360.0/(ARM_PLATFORM_ROTATOR_ENCODER_PPR * ARM_PLATFORM_ROTATOR_GEAR_RATIO);
    public static final double ARM_PLATFORM_ROTATOR_OFFSET             = 66.0;
    public static final double ARM_PLATFORM_ROTATOR_OFFSET_TELEOP      = 10.0;
    public static final double ARM_PLATFORM_ROTATOR_MIN_POS            = 0.0;
    public static final double ARM_PLATFORM_ROTATOR_MAX_POS            = 195.0;
    public static final boolean ARM_PLATFORM_ROTATOR_MOTOR_INVERTED    = false;
    public static final boolean ARM_PLATFORM_ROTATOR_HAS_LOWER_LIMIT_SWITCH  = false;
    public static final boolean ARM_PLATFORM_ROTATOR_LOWER_LIMIT_INVERTED    = false;
    public static final boolean ARM_PLATFORM_ROTATOR_HAS_UPPER_LIMIT_SWITCH  = false;
    public static final boolean ARM_PLATFORM_ROTATOR_UPPER_LIMIT_INVERTED    = false;
    public static final double ARM_PLATFORM_ROTATOR_CAL_POWER          = 0.2;
    public static final double ARM_PLATFORM_ROTATOR_STALL_MIN_POWER    = 0.1;
    public static final double ARM_PLATFORM_ROTATOR_STALL_TIMEOUT      = 1.0;
    public static final double ARM_PLATFORM_ROTATOR_RESET_TIMEOUT      = 0.5;
    public static final double[] ARM_PLATFORM_ROTATOR_PRESET_LEVELS    = new double[] {5.0, 100.0,195.0};
    public static final double ARM_PLATFORM_ROTATOR_SLOW_POWER_SCALE   = 0.25;


    // Carousel Spinner
    public static final double CAROUSEL_SPINNER_BLUE                   = 1.0;
    public static final double CAROUSEL_SPINNER_RED                    = 0.0;
    public static final double CAROUSEL_SPINNER_STOP_POWER             = 0.5;
    public static final double CAROUSEL_SPINNER_SPIN_TIME              = 4.0;

    // Carousel Spinner Rotator
    public static final double CAROUSEL_SPINNER_ROTATOR_KP             = 0.005;
    public static final double CAROUSEL_SPINNER_ROTATOR_KI             = 0.0;
    public static final double CAROUSEL_SPINNER_ROTATOR_KD             = 0.0;
    public static final double CAROUSEL_SPINNER_ROTATOR_TOLERANCE      = 2.0;
    public static final double CAROUSEL_SPINNER_ROTATOR_ENCODER_PPR    = REV_CORE_HEX_ENCODER_PPR;
    // https://www.revrobotics.com/rev-41-1600/
    public static final double CAROUSEL_SPINNER_ROTATOR_GEAR_RATIO     = 1.0;
    public static final double CAROUSEL_SPINNER_ROTATOR_DEG_PER_COUNT  = 360.0/(REV_CORE_HEX_ENCODER_PPR * CAROUSEL_SPINNER_ROTATOR_GEAR_RATIO);
    public static final double CAROUSEL_SPINNER_ROTATOR_OFFSET         = 0.0;
    public static final double CAROUSEL_SPINNER_ROTATOR_MIN_POS        = 0.0;
    public static final double CAROUSEL_SPINNER_ROTATOR_MAX_POS        = 195.0;
    public static final double CAROUSEL_SPINNER_ROTATOR_TRAVEL_POS               = CAROUSEL_SPINNER_ROTATOR_MIN_POS+2.0;
    public static final boolean CAROUSEL_SPINNER_ROTATOR_MOTOR_INVERTED          = true;
    public static final boolean CAROUSEL_SPINNER_ROTATOR_HAS_LOWER_LIMIT_SWITCH  = false;
    public static final boolean CAROUSEL_SPINNER_ROTATOR_LOWER_LIMIT_INVERTED    = false;
    public static final boolean CAROUSEL_SPINNER_ROTATOR_HAS_UPPER_LIMIT_SWITCH  = false;
    public static final boolean CAROUSEL_SPINNER_ROTATOR_UPPER_LIMIT_INVERTED    = false;
    public static final double CAROUSEL_SPINNER_ROTATOR_CAL_POWER          = 0.2;
    public static final double CAROUSEL_SPINNER_ROTATOR_STALL_MIN_POWER    = 0.1;
    public static final double CAROUSEL_SPINNER_ROTATOR_STALL_TIMEOUT      = 1.0;
    public static final double CAROUSEL_SPINNER_ROTATOR_RESET_TIMEOUT      = 0.5;
    public static final double[] CAROUSEL_SPINNER_ROTATOR_PRESET_LEVELS    = new double[] {CAROUSEL_SPINNER_ROTATOR_MIN_POS, CAROUSEL_SPINNER_ROTATOR_MAX_POS};
    public static final double CAROUSEL_SPINNER_ROTATOR_SLOW_POWER_SCALE   = 0.5;
    
    // Tape Measure
    public static final double TAPE_MEASURE_MIN_POS                    = 0.0;
    public static final double TAPE_MEASURE_MAX_POS                    = 1.0;
    public static final double TAPE_MEASURE_HOLD_SPOOL                 = TAPE_MEASURE_MIN_POS;
    public static final double TAPE_MEASURE_SHOOT                      = TAPE_MEASURE_MAX_POS;

}   // class RobotInfo
