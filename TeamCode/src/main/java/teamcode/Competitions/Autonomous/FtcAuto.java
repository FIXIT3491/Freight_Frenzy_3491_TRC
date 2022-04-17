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

package teamcode.Competitions.Autonomous;

import java.util.Locale;

import TrcCommonLib.command.CmdPidDrive;
import TrcCommonLib.trclib.TrcPose2D;
import teamcode.Season_Setup.Robot;
import teamcode.Season_Setup.RobotParams;
import TrcCommonLib.command.CmdTimedDrive;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcChoiceMenu;
import TrcFtcLib.ftclib.FtcMatchInfo;
import TrcFtcLib.ftclib.FtcMenu;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcValueMenu;


/**
 * This class contains the Autonomous Mode program.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Programs", group="Competition")

public class FtcAuto extends FtcOpMode
{
    /**
     * Choices for what Autonomous Strategy will be conducted.
     */
    public enum AutoStrategy
    {
        // Advanced Programs
        AUTO_NEAR_CAROUSEL,             // Barcode, Freight, Carousel, Park
        AUTO_FAR_CAROUSEL,              // Barcode, Freight, Carousel, Park
//        AUTO_NEAR_CAROUSEL_DUCK_DELIVERY_STORAGE_PARKING,
//        AUTO_NEAR_CAROUSEL_DUCK_DELIVERY_WAREHOUSE_PARKING,
//        AUTO_SHUTTLE_BACK_AND_FORTH,

        // Backup Programs
        AUTO_ALLIANCE_HUB_ONLY,         // PID
        AUTO_ALLIANCE_TIMED_HUB_ONLY,   // Time Only

        // Basic Programs
        PID_DRIVE,
        TIMED_DRIVE,
        DO_NOTHING
    }   // enum AutoStrategy

    /**
     * To state which alliance the robot is in.
     */
    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   // enum Alliance

    /**
     * Choices for where to park.
     */
    public enum Parking
    {
        WAREHOUSE_PARKING,
        STORAGE_PARKING,
        NO_PARKING
    }   // enum Parking

    /**
     * Choices for how to get to the Warehouse
     */
    public enum WarehousePath
    {
        AROUND_HUB,
        MIDDLE,
        THROUGH_GAP,
        NOT_APPLICABLE
    }

    /**
     * Choosing where to park in the Warehouse
     */
    public enum WarehouseParking
    {
        PREP_ALLIANCE_HUB,
        PREP_SHARED_HUB,
        PARK
    }

    /**
     * This class stores the autonomous menu choices.
     */
    public static class AutoChoices
    {
        // Main pathing selection
        double startDelay = 0.0;
        Alliance alliance = Alliance.RED_ALLIANCE;
        AutoStrategy strategy = AutoStrategy.AUTO_FAR_CAROUSEL;
        boolean freightDelivery = false;
        boolean doCarousel = false;
        Parking parking = Parking.WAREHOUSE_PARKING;
        WarehousePath warehousePath = WarehousePath.MIDDLE;
        WarehouseParking warehouseParking = WarehouseParking.PARK;

        // Additional choices
        double yTarget = 0.0;
        double turnTarget = 0.0;
        double driveTime = 0.0;
        double drivePower = 0.0;

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "startDelay=%.0f " +
                "alliance=\"%s\" " +
                "strategy=\"%s\" " +
                "yTarget=%.1f " +
                "turnTarget=%.0f " +
                "driveTime=%.0f " +
                "drivePower=%.1f",
                startDelay, alliance, strategy, yTarget, turnTarget, driveTime, drivePower);
        }   // toString

    }   // class AutoChoices

    private static final String moduleName = "FtcAuto";
    private static final boolean logEvents = true;
    private static final boolean debugPid = true;

    private Robot robot;
    private FtcMatchInfo matchInfo;
    private final AutoChoices autoChoices = new AutoChoices();
    private TrcRobot.RobotCommand autoCommand = null;


    // Implements FtcOpMode abstract method

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void initRobot()
    {
        @SuppressWarnings("unused")
        final String funcName = "initRobot";

        // Create and initialize robot object
        robot = new Robot(TrcRobot.getRunMode());

        // Open trace log
        if (RobotParams.Preferences.useTraceLog)
        {
            matchInfo = FtcMatchInfo.getMatchInfo();
            String filePrefix = String.format(Locale.US, "%s%02d", matchInfo.matchType, matchInfo.matchNumber);
            robot.globalTracer.openTraceLog(RobotParams.LOG_PATH_FOLDER, filePrefix);
        }

        // Create and run choice menus
        doAutoChoicesMenus();

        Robot.isRedAlliance = autoChoices.alliance == Alliance.RED_ALLIANCE;

        // Create autonomous command according to chosen strategy
        switch (autoChoices.strategy)
        {
            // Advanced Programs
            case AUTO_NEAR_CAROUSEL:
                if (!RobotParams.Preferences.visionOnly)
                {
                    autoCommand = new PurePursuit_CmdAutoNearCarousel(robot, autoChoices);
                }
                break;

            case AUTO_FAR_CAROUSEL:
                if (!RobotParams.Preferences.visionOnly)
                {
                    autoCommand = new PurePursuit_CmdAutoFarCarousel(robot, autoChoices);
                }
                break;


            // Backup Programs
            case AUTO_ALLIANCE_HUB_ONLY:
                if (!RobotParams.Preferences.visionOnly)
                {
                    autoCommand = new zBackup_CmdAutoAllianceHubOnly_PID(robot, autoChoices);
                }
                break;

            case AUTO_ALLIANCE_TIMED_HUB_ONLY:
                if (!RobotParams.Preferences.visionOnly)
                {
                    autoCommand = new zBackup_CmdAutoAllianceHubOnly_Time(robot, autoChoices);
                }
                break;


            // Basic Programs
            case PID_DRIVE:
                if (!RobotParams.Preferences.visionOnly)
                {
                    autoCommand = new CmdPidDrive(
                            robot.robotDrive.driveBase, robot.robotDrive.pidDrive, autoChoices.startDelay,
                            autoChoices.drivePower, null,
                            new TrcPose2D(0.0, autoChoices.yTarget*12.0, autoChoices.turnTarget));
                }
                break;

            case TIMED_DRIVE:
                if (!RobotParams.Preferences.visionOnly)
                {
                    autoCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, autoChoices.startDelay, autoChoices.driveTime,
                        0.0, autoChoices.drivePower, 0.0);
                }
                break;

            case DO_NOTHING:
            default:
                autoCommand = null;
                break;
        }
    }   // initRobot


    // Overrides TrcRobot.RobotMode methods.

    /**
     * This method is called periodically after initRobot() is called but before competition starts. For this season,
     * we are detecting the duck's barcode position before the match starts.
     */
    @Override
    public void initPeriodic()
    {
    }   // initPeriodic

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.dashboard.clearDisplay();

        if (RobotParams.Preferences.useTraceLog)
        {
            robot.globalTracer.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(moduleName, "***** Starting autonomous *****");
        if (matchInfo != null)
        {
            robot.globalTracer.logInfo(moduleName, "MatchInfo", "%s", matchInfo);
        }
        robot.globalTracer.logInfo(moduleName, "AutoChoices", "%s", autoChoices);

        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode
        robot.startMode(nextMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(true);
        }

        // Move arm up and over from the initialization position, to the front of the robot.
        robot.armRotator.setLevel(0);
        robot.armPlatformRotator.setLevel(0.5,0);

        // Debugging tools
        robot.robotDrive.pidDrive.setMsgTracer(robot.globalTracer, logEvents, debugPid, robot.battery);
        robot.robotDrive.purePursuitDrive.setMsgTracer(robot.globalTracer, logEvents, debugPid, robot.battery);
    }   // startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        // Opmode is about to stop, cancel autonomous command in progress if any.
        if (autoCommand != null)
        {
            autoCommand.cancel();
        }

        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode
        robot.stopMode(prevMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(false);
        }

        printPerformanceMetrics(robot.globalTracer);

        if (robot.globalTracer.tracerLogIsOpened())
        {
            robot.globalTracer.closeTraceLog();
        }
    }   // stopMode

    /**
     * This method is called periodically as fast as the control system allows. Typically, you put code that requires
     * servicing at a higher frequency here. To make the robot as responsive and as accurate as possible especially
     * in autonomous mode, you will typically put that code here.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoCommand != null)
        {
            // Run the autonomous command
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   // runContinuous

    /**
     * This method creates the autonomous menus, displays them and stores the choices.
     */
    private void doAutoChoicesMenus()
    {
        // Construct menus
        FtcValueMenu startDelayMenu = new FtcValueMenu(
                "Start delay time:", null, 0.0, 30.0, 1.0, 0.0, " %.0f sec");
        FtcChoiceMenu<Alliance> allianceMenu                   = new FtcChoiceMenu<>("Alliance:", startDelayMenu);
        FtcChoiceMenu<AutoStrategy> strategyMenu               = new FtcChoiceMenu<>("Auto Strategies:", allianceMenu);
        FtcChoiceMenu<Boolean> freightDeliveryMenu             = new FtcChoiceMenu<>("Freight Delivery:", strategyMenu);
        FtcChoiceMenu<Boolean> carouselMenu                    = new FtcChoiceMenu<>("Carousel:", freightDeliveryMenu);
        FtcChoiceMenu<Parking> parkingMenu                     = new FtcChoiceMenu<>("Parking:", carouselMenu);
        FtcChoiceMenu<WarehousePath> warehousePathingMenu      = new FtcChoiceMenu<>("Warehouse Pathing:", parkingMenu);
        FtcChoiceMenu<WarehouseParking> warehouseParkingMenu   = new FtcChoiceMenu<>("Warehouse Parking", warehousePathingMenu);

        // Menu setup
        FtcValueMenu yTargetMenu    = new FtcValueMenu(
            "yTarget:", strategyMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
        FtcValueMenu driveTimeMenu  = new FtcValueMenu(
            "Drive time:", strategyMenu, 0.0, 30.0, 1.0, 5.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
            "Drive power:", strategyMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");

        // Set menu order
        startDelayMenu.setChildMenu(allianceMenu);
        yTargetMenu.setChildMenu(drivePowerMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);

        // Populate choice menus
        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, strategyMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, strategyMenu);

        //// Choosing Autonomous Strategy
        // Advanced Programs
        strategyMenu.addChoice("Near Carousel Autonomous", AutoStrategy.AUTO_NEAR_CAROUSEL, true, freightDeliveryMenu);
        strategyMenu.addChoice("Far Carousel Autonomous", AutoStrategy.AUTO_FAR_CAROUSEL, false, freightDeliveryMenu);

        // Backup Programs
        strategyMenu.addChoice("Alliance Hub Only", AutoStrategy.AUTO_ALLIANCE_HUB_ONLY, false);
        strategyMenu.addChoice("Alliance Hub Only - Timed", AutoStrategy.AUTO_ALLIANCE_TIMED_HUB_ONLY, false);
        
        // Basic Programs
        strategyMenu.addChoice("PID Drive", AutoStrategy.PID_DRIVE, false, yTargetMenu);
        strategyMenu.addChoice("Timed Drive", AutoStrategy.TIMED_DRIVE, false, driveTimeMenu);
        strategyMenu.addChoice("Do nothing", AutoStrategy.DO_NOTHING, false);

        
        // Freight choices
        freightDeliveryMenu.addChoice("Do Freight Delivery", true, true, carouselMenu);
        freightDeliveryMenu.addChoice("No Freight Delivery", false, false, carouselMenu);

        // Carousel choices
        carouselMenu.addChoice("Do Carousel", true, true, parkingMenu);
        carouselMenu.addChoice("No Carousel", false, false, parkingMenu);

        // Parking choices
        parkingMenu.addChoice("Warehouse Parking", Parking.WAREHOUSE_PARKING, true, warehousePathingMenu);
        parkingMenu.addChoice("Storage Parking", Parking.STORAGE_PARKING, false, warehousePathingMenu);
        parkingMenu.addChoice("No Parking", Parking.NO_PARKING, false, warehousePathingMenu);

        // Warehouse Path choices
        warehousePathingMenu.addChoice("Around Hub", WarehousePath.AROUND_HUB, false, warehouseParkingMenu);
        warehousePathingMenu.addChoice("Middle", WarehousePath.MIDDLE, true, warehouseParkingMenu);
        warehousePathingMenu.addChoice("Through Gap", WarehousePath.THROUGH_GAP, false, warehouseParkingMenu);

        // Warehouse Parking choices
        warehouseParkingMenu.addChoice("Prep for Alliance Hub", WarehouseParking.PREP_ALLIANCE_HUB,false);
        warehouseParkingMenu.addChoice("Prep for Shared Hub", WarehouseParking.PREP_SHARED_HUB,true);
        warehouseParkingMenu.addChoice("Stop", WarehouseParking.PARK,false);


        // Traverse menus
        FtcMenu.walkMenuTree(startDelayMenu);

        // Fetch choices
        autoChoices.startDelay = startDelayMenu.getCurrentValue();
        autoChoices.alliance = allianceMenu.getCurrentChoiceObject();
        autoChoices.strategy = strategyMenu.getCurrentChoiceObject();
        autoChoices.freightDelivery = freightDeliveryMenu.getCurrentChoiceObject();
        autoChoices.doCarousel = carouselMenu.getCurrentChoiceObject();
        autoChoices.parking = parkingMenu.getCurrentChoiceObject();
        autoChoices.warehousePath = warehousePathingMenu.getCurrentChoiceObject();
        autoChoices.warehouseParking = warehouseParkingMenu.getCurrentChoiceObject();
        autoChoices.yTarget = yTargetMenu.getCurrentValue();
        autoChoices.driveTime = driveTimeMenu.getCurrentValue();
        autoChoices.drivePower = drivePowerMenu.getCurrentValue();

        // Show choices
        robot.dashboard.displayPrintf(2, "Auto Choices: %s", autoChoices);
    }   // doAutoChoicesMenus

}   // class FtcAuto
