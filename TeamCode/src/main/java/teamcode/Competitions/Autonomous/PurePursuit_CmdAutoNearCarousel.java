/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import teamcode.Season_Setup.Freight_Frenzy_Pipeline;
import teamcode.Season_Setup.Robot;
import teamcode.Season_Setup.RobotParams;

@SuppressWarnings("ConstantConditions")
class PurePursuit_CmdAutoNearCarousel implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoNearCarousel";
    private static final double PARK_WAREHOUSE_AROUND_HUB_TIME = 8.0;
    private static final double PARK_WAREHOUSE_NORMAL_TIME = 5.0;

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int elementPosition;
    Freight_Frenzy_Pipeline freight_frenzy_pipeline = new Freight_Frenzy_Pipeline();


    private enum State
    {
        START_DELAY,

        DRIVE_TO_ALLIANCE_SHIPPING_HUB,
        DEPOSIT_FREIGHT,

        DRIVE_TO_CAROUSEL,
        GET_TO_CAROUSEL,
        SPIN_CAROUSEL,

//        POSITION_TO_FIND_THE_DUCK,
//        FIND_THE_DUCK,
//        GO_PICKUP_DUCK,
//        DONE_PICKUP_DUCK,

        CHECK_PARKING_CHOICE,
        DRIVE_TO_ALLIANCE_STORAGE_UNIT,
        DRIVE_TO_WAREHOUSE_AROUND_HUB,
        DRIVE_TO_WAREHOUSE_TO_BARRIER,
        DRIVE_TO_WAREHOUSE_THROUGH_GAP,
        GET_INTO_WAREHOUSE,

        DONE
    }   // enum State

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    PurePursuit_CmdAutoNearCarousel(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        robot.robotDrive.purePursuitDrive.setFastModeEnabled(true);
        sm.start(State.START_DELAY);
    }   // CmdAutoNearCarousel

    // Implements the TrcRobot.RobotCommand interface.

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   // isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        robot.robotDrive.cancel();
        sm.stop();
    }   // cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            boolean traceState = true;
            String msg;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state) {
                case START_DELAY:
                    // Set robot starting position in the field.
                    robot.robotDrive.driveBase.setFieldPosition(
                            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                                    RobotParams.STARTPOS_RED_NEAR_CAROUSEL : RobotParams.STARTPOS_BLUE_NEAR_CAROUSEL);

                    // Lift armRotator above ground, and rotate to front of robot.
                    robot.armRotator.setLevel(0);
                    robot.armPlatformRotator.setLevel(0.5,2);

                    // Call vision at the beginning to figure out the position of the duck.
                    if (robot.vision != null)
                    {
                        elementPosition = freight_frenzy_pipeline.getPosition().value;
                    }

                    if (freight_frenzy_pipeline.getPosition().value == 0)
                    {
                        // We still can't see the element, default to level 3.
                        elementPosition = 3;
                        msg = "No element found, default to position " + elementPosition;
                        robot.globalTracer.traceInfo(moduleName, msg);

                        robot.speak(msg);
                    }
                    else
                    {
                        msg = "Element found at position " + elementPosition;
                        robot.globalTracer.traceInfo(moduleName, msg);

                        robot.speak("Element found at position " + elementPosition);
                    }

                    // Do start delay if any.
                    if (autoChoices.startDelay == 0.0)
                    {
                        // Intentionally falling through to the next state.
                        sm.setState(State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                    }
                    else
                    {
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                        break;
                    }

                case DRIVE_TO_ALLIANCE_SHIPPING_HUB:

                    if (!autoChoices.freightDelivery)
                    {
                        // We are not doing freight delivery, go to next state.
                        sm.setState(State.DRIVE_TO_CAROUSEL);
                    }
                    else
                    {
                        // Note: the smaller the number the closer to the hub.
                        double distanceToHub = elementPosition == 3? 0.75: elementPosition == 2? 0.75: 0.75;

                        // Drive to the alliance specific hub from the starting position.
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                    event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(-1.5, -1, 90.0));
                            robot.robotDrive.purePursuitDrive.start(
                                    event, 4.0,robot.robotDrive.driveBase.getFieldPosition(), true,
                                    robot.robotDrive.pathPoint(distanceToHub, distanceToHub, 90.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                    event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(-1.5, 1.5, 135.0));
                            robot.robotDrive.purePursuitDrive.start(
                                    event, 4.0,robot.robotDrive.driveBase.getFieldPosition(), true,
                                    robot.robotDrive.pathPoint(distanceToHub, -distanceToHub, 135.0));
                        }

                        // Raise arm to the detected duck level at the same time.
                        robot.armRotator.setLevel(elementPosition);

                        sm.waitForSingleEvent(event, State.DEPOSIT_FREIGHT);
                    }
                    break;

                case DEPOSIT_FREIGHT:
                    // Dumps the freight, when done, signals event and goes to next state.
                    robot.collector.setPosition(RobotParams.COLLECTOR_DEPOSIT_POWER, RobotParams.COLLECTOR_DEPOSITING_TIME, event);

                    sm.waitForSingleEvent(event, State.DRIVE_TO_CAROUSEL);
                    break;

                case DRIVE_TO_CAROUSEL:
                    // Turn off Collector
                    robot.collector.setPosition(RobotParams.COLLECTOR_STOP_POWER);

                    // Lower armRotator, delayed to avoid hitting the alliance hub if there.
                    robot.armRotator.setLevel(1.0, 1);

                    if (!autoChoices.doCarousel)
                    {
                        // We are not spinning the carousel, skip to next state.
                        sm.setState(State.DRIVE_TO_WAREHOUSE_AROUND_HUB);
                    }
                    else
                    {
                        // Drive to the carousel from the starting position.
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                    event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(-2.5, -2.0, 0.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                    event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(-2.3, 2.0, 135.0));
                        }
                        sm.waitForSingleEvent(event, State.GET_TO_CAROUSEL);
                    }
                    break;
//
//                case GET_TO_CAROUSEL:
//                    // We are a few inches from the carousel, drive slowly towards it to touch it.
//                    robot.robotDrive.driveBase.tankDrive(-0.2, -0.2, false);
//                    timer.set(0.8, event);
//                    sm.waitForSingleEvent(event, State.SPIN_CAROUSEL);
//                    break;
//
//                case SPIN_CAROUSEL:
//                    // We touched the carousel, so stop the drive base.
//                    robot.robotDrive.driveBase.stop();
//
//                    // Spin the carousel.
//                    robot.carouselSpinner.setPosition(
//                            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
//                                    RobotParams.CAROUSEL_SPINNER_RED: RobotParams.CAROUSEL_SPINNER_BLUE,
//                            RobotParams.CAROUSEL_SPINNER_SPIN_TIME, event);
//                    sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_STORAGE_UNIT);
//                    break;
//
//
//                case CHECK_PARKING_CHOICE:
//                // Check if we are parking.
//                    if (autoChoices.parking == FtcAuto.Parking.NO_PARKING)
//                    {
//                        // We are not parking anywhere, just stop and be done.
//                        sm.setState(State.DONE);
//                    }
//                    // If we don't have enough time to go to the warehouse, park at the storage unit instead - Checking time for going around the Hub.
//                    else if (autoChoices.parking == FtcAuto.Parking.WAREHOUSE_PARKING  &&
//                            autoChoices.warehousePath == FtcAuto.WarehousePath.AROUND_HUB &&
//                            30.0 - elapsedTime > PARK_WAREHOUSE_AROUND_HUB_TIME)
//                    {
//                        // We are Driving to the Warehouse by going around the Hub.
//                        sm.setState(State.DRIVE_TO_WAREHOUSE_AROUND_HUB);
//                    }
//                    // If we don't have enough time to go to the warehouse, park at the storage unit instead - Checking time for going through Barrier/ Gap.
//                    else if (autoChoices.parking == FtcAuto.Parking.WAREHOUSE_PARKING  &&
//                            30.0 - elapsedTime > PARK_WAREHOUSE_NORMAL_TIME)
//                    {
//                        // We are Driving to the Warehouse by going through the Middle.
//                        if (autoChoices.warehousePath == FtcAuto.WarehousePath.MIDDLE)
//                        {
//                            sm.setState(State.DRIVE_TO_WAREHOUSE_TO_BARRIER);
//                        }
//                        // We are Driving to the Warehouse by going through the Gap.
//                        else
//                        {
//                            sm.setState(State.DRIVE_TO_WAREHOUSE_THROUGH_GAP);
//                        }
//
//                    }
//                    // We are Driving to the Storage Unit.
//                    else
//                    {
//                        sm.setState(State.DRIVE_TO_ALLIANCE_STORAGE_UNIT);
//                    }
//                    break;
//
//                case DRIVE_TO_ALLIANCE_STORAGE_UNIT:
//                    // Drive to storage unit. We could be coming from starting position, carousel or alliance hub.
//                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
//                    {
//                        // Rotate arm to the side of robot, to possibly fit fully within the storage unit.
//                        robot.armRotator.setLevel(2);
//                        robot.armPlatformRotator.setLevel(0.5,3);
//
//                        robot.robotDrive.purePursuitDrive.start(
//                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                robot.robotDrive.pathPoint(-2.5, -1.5, 90.0)); // Realistic Heading: 0.0
//                    }
//                    else
//                    {
//                        // Rotate arm to the side of robot, to possibly fit fully within the storage unit.
//                        robot.armRotator.setLevel(2);
//                        robot.armPlatformRotator.setLevel(0.5,1);
//
//                        robot.robotDrive.purePursuitDrive.start(
//                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                robot.robotDrive.pathPoint(-2.5, 1.5, 90.0)); // Realistic Heading: 180.0
//                    }
//                    sm.waitForSingleEvent(event, State.DONE);
//
//                    break;
//
//                case DRIVE_TO_WAREHOUSE_AROUND_HUB:
//                    // We are heading to the warehouse but we could be coming from starting position, carousel or
//                    // alliance hub.
//                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
//                    {
//                        robot.robotDrive.purePursuitDrive.start(
//                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                robot.robotDrive.pathPoint(-1.5, -0.5, 45.0),
//                                robot.robotDrive.pathPoint(0.5, -0.5, 180.0),
//                                robot.robotDrive.pathPoint(0.5, -1.5, 90.0));
//                    }
//                    else
//                    {
//                        robot.robotDrive.purePursuitDrive.start(
//                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                robot.robotDrive.pathPoint(-1.5, 0.5, 135.0),
//                                robot.robotDrive.pathPoint(0.5, 0.5, 0.0),
//                                robot.robotDrive.pathPoint(0.5, 1.5, 90.0));
//                    }
//                    // Lift arm to go over barrier
//                    robot.armRotator.setLevel(1);
//
//                    sm.waitForSingleEvent(event, State.GET_INTO_WAREHOUSE);
//                    break;
//
//                case DRIVE_TO_WAREHOUSE_TO_BARRIER:
//                    // We are heading to the warehouse but we could be coming from starting position, carousel or
//                    // alliance hub.
//                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
//                    {
//                        robot.robotDrive.purePursuitDrive.start(
//                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                robot.robotDrive.pathPoint(-1.5, -1.9, 90.0),
//                                robot.robotDrive.pathPoint(0.5, -1.9, 90.0));
//                    }
//                    else
//                    {
//                        robot.robotDrive.purePursuitDrive.start(
//                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                robot.robotDrive.pathPoint(-1.5, 1.9, 90.0),
//                                robot.robotDrive.pathPoint(0.5, 1.9, 90.0));
//                    }
//                    // Lift arm to go over barrier
//                    robot.armRotator.setLevel(1);
//
//                    sm.waitForSingleEvent(event, State.GET_INTO_WAREHOUSE);
//                    break;
//
//                case DRIVE_TO_WAREHOUSE_THROUGH_GAP:
//                    // We are heading to the warehouse but we could be coming from starting position, carousel or
//                    // alliance hub.
//                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
//                    {
//                        robot.robotDrive.purePursuitDrive.start(
//                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                robot.robotDrive.pathPoint(0.5, -2.76, 90.0));
//                    }
//                    else
//                    {
//                        robot.robotDrive.purePursuitDrive.start(
//                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                robot.robotDrive.pathPoint(0.5, 2.76, 90.0));
//
//                    }
//                    // Lift arm to go over barrier
//                    robot.armRotator.setLevel(1);
//
//                    sm.waitForSingleEvent(event, State.GET_INTO_WAREHOUSE);
//                    break;
//
//                case GET_INTO_WAREHOUSE:
//                    //// Check how to get into warehouse.
//                    // Parking in a manner for preparing to do Alliance Hub in TeleOp.
//                    if (autoChoices.warehouseParking == FtcAuto.WarehouseParking.PREP_ALLIANCE_HUB)
//                    {
//                        // Check Previous position of Robot.
//                        if (autoChoices.warehousePath == FtcAuto.WarehousePath.THROUGH_GAP)
//                        {
//                            if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
//                            {
//                                robot.robotDrive.purePursuitDrive.start(
//                                        event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                        robot.robotDrive.pathPoint(1.5, -2.76, 90.0));
//                            }
//                            else
//                            {
//                                robot.robotDrive.purePursuitDrive.start(
//                                        event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                        robot.robotDrive.pathPoint(1.5, 2.76, 90.0));
//                            }
//                        }
//                        // If robot will need to drive over Barrier.
//                        else
//                        {
//                            // Run full speed into the warehouse crossing the barriers.
//                            robot.robotDrive.driveBase.tankDrive(1.0, 1.0, false);
//                            timer.set(1.0, event);
//                        }
//                    }
//
//                    // Parking in a manner for preparing to do Shared Hub in TeleOp.
//                    else if (autoChoices.warehouseParking == FtcAuto.WarehouseParking.PREP_SHARED_HUB)
//                    {
//                        // Check Previous position of Robot.
//                        if (autoChoices.warehousePath == FtcAuto.WarehousePath.THROUGH_GAP)
//                        {
//                            if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
//                            {
//                                robot.robotDrive.purePursuitDrive.start(
//                                        event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                        robot.robotDrive.pathPoint(1.5,-2.76,45.0),
//                                        robot.robotDrive.pathPoint(2.76,-1.5,0.0));
//                            }
//                            else
//                            {
//                                robot.robotDrive.purePursuitDrive.start(
//                                        event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                        robot.robotDrive.pathPoint(1.5,2.76,45.0),
//                                        robot.robotDrive.pathPoint(2.76,1.5,0.0));
//                            }
//                        }
//
//                        // If robot will need to drive over Barrier.
//                        else
//                        {
//                            // Run full speed into the warehouse crossing the barriers.
//                            robot.robotDrive.driveBase.tankDrive(1.0, 1.0, false);
//                            timer.set(1.0, event);
//
//                            if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
//                            {
//                                robot.robotDrive.driveBase.setFieldPosition(new TrcPose2D(0.0, 0.0, 0.0));
//                                robot.robotDrive.purePursuitDrive.start(
//                                        robot.robotDrive.driveBase.getFieldPosition(), false,
//                                        robot.robotDrive.pathPoint(0.0, 1.0, -90.0));
//                            }
//                            else
//                            {
//                                robot.robotDrive.driveBase.setFieldPosition(new TrcPose2D(0.0, 0.0, 0.0));
//                                robot.robotDrive.purePursuitDrive.start(
//                                        robot.robotDrive.driveBase.getFieldPosition(), false,
//                                        robot.robotDrive.pathPoint(0.0, 1.0, 90.0));
//                            }
//                        }
//                    }
//
//                    sm.waitForSingleEvent(event, State.DONE);
//                    break;

                case DONE:
                default:
                    // We are done, zero calibrate the arm will lower it.
                    robot.armRotator.zeroCalibrate();
                    cancel();
                    break;
            }

            if (traceState)
            {
                robot.globalTracer.traceStateInfo(
                        sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive, robot.robotDrive.purePursuitDrive,
                        null);
            }
        }

        return !sm.isEnabled();
    }   // cmdPeriodic

}   // class CmdAutoNearCarousel