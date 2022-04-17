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
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import teamcode.Season_Setup.Freight_Frenzy_Pipeline;
import teamcode.Season_Setup.Robot;
import teamcode.Season_Setup.RobotParams;

class PurePursuit_CmdAutoNearCarousel implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoNearCarousel";
    private static final double PARK_WAREHOUSE_TIME = 8.0;
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

        DRIVE_TO_ALLIANCE_STORAGE_UNIT,
        DRIVE_TO_WAREHOUSE,
        GET_INTO_WAREHOUSE,

        DONE
    }   // enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

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
            switch (state)
            {
//                case START_DELAY:
//                    // Set robot starting position in the field.
//                    robot.robotDrive.driveBase.setFieldPosition(
//                            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
//                                    RobotParams.STARTPOS_RED_NEAR : RobotParams.STARTPOS_BLUE_NEAR);
//
//                    // Lift armRotator above ground, and rotate to front of robot, and lower arm.
//                    robot.armRotator.setLevel(0);
//                    robot.armPlatformRotator.setLevel(0.5,0);
//
//                    // Call vision at the beginning to figure out the position of the duck.
//                    if (robot.vision != null)
//                    {
//                        elementInfo = robot.vision.getElementInfo();
//                    }
//
//                    if (freight_frenzy_pipeline.getPosition().value == 0)
//                    {
//                        // We still can't see the element, default to level 3.
//
//                        elementInfo.elementPosition = 3;
//                        msg = "No element found, default to position " + elementInfo.elementPosition;
//                        robot.globalTracer.traceInfo(moduleName, msg);
//                        robot.speak(msg);
//                    }
//                    else
//                    {
//                        msg = "Element found at position " + elementInfo.elementPosition;
//                        robot.globalTracer.traceInfo(moduleName, msg);
//                        robot.speak("Element found at position " + elementInfo.elementPosition);
//                    }
//
//                    // Do start delay if any.
//                    if (autoChoices.startDelay == 0.0)
//                    {
//                        // Intentionally falling through to the next state.
//                        sm.setState(State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
//                    }
//                    else
//                    {
//                        timer.set(autoChoices.startDelay, event);
//                        sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
//                        break;
//                    }
//
//                case DRIVE_TO_ALLIANCE_SHIPPING_HUB:
//
//                    robot.vision.disableWebcam();
//
//                    if (!autoChoices.freightDelivery)
//                    {
//                        // We are not doing freight delivery, go to next state.
//                        sm.setState(State.DRIVE_TO_CAROUSEL);
//                    }
//                    else
//                    {
//                        // Note: the smaller the number the closer to the hub.
//                        double distanceToHub = elementInfo.elementPosition == 3? 1.1: elementInfo.elementPosition == 2? 1.3: 1.0;
//
//                        // Drive to the alliance specific hub from the starting position.
//                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
//                        {
//                            robot.robotDrive.purePursuitDrive.start(
//                                    event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                    robot.robotDrive.pathPoint(-1.5, -1.5, 45.0));
//                            robot.robotDrive.purePursuitDrive.start(
//                                    event, robot.robotDrive.driveBase.getFieldPosition(), true,
//                                    robot.robotDrive.pathPoint(distanceToHub, distanceToHub, 45.0));
//                        }
//                        else
//                        {
//                            robot.robotDrive.purePursuitDrive.start(
//                                    event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                    robot.robotDrive.pathPoint(-1.5, 1.5, -45.0));
//                            robot.robotDrive.purePursuitDrive.start(
//                                    event, robot.robotDrive.driveBase.getFieldPosition(), true,
//                                    robot.robotDrive.pathPoint(distanceToHub, -distanceToHub, 45.0));
//                        }
//
//                        // Raise arm to the detected duck level at the same time.
//                        robot.armRotator.setLevel(elementInfo.elementPosition);
//
//                        sm.waitForSingleEvent(event, State.DEPOSIT_FREIGHT);
//                    }
//                    break;

                case DEPOSIT_FREIGHT:
                    // Dumps the freight, when done, signals event and goes to next state.
                    robot.collector.setPosition(RobotParams.COLLECTOR_DEPOSIT_POWER, RobotParams.COLLECTOR_DEPOSITING_TIME, event);

                    sm.waitForSingleEvent(event, State.DRIVE_TO_CAROUSEL);
                    break;

                case DRIVE_TO_CAROUSEL:
                    // Lower armRotator, delayed to avoid hitting the carousel
                    robot.armRotator.setLevel(1.0, 1);

                    // Rotate Carousel Spinner Rotator to the appropriate alliance side
//                    robot.carouselSpinnerRotator.setLevel(autoChoices.alliance ==
//                            FtcAuto.Alliance.RED_ALLIANCE?0:1);

                    if (!autoChoices.doCarousel)
                    {
                        // We are not spinning the carousel, skip to next state.
                        sm.setState(State.DRIVE_TO_WAREHOUSE);
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
                                    robot.robotDrive.pathPoint(-2.5, 2.0, 180.0));
                        }
                        sm.waitForSingleEvent(event, State.GET_TO_CAROUSEL);
                    }
                    break;

                case GET_TO_CAROUSEL:
                    // We are a few inches from the carousel, drive slowly towards it to touch it.
                    robot.robotDrive.driveBase.tankDrive(-0.2, -0.2, false);
                    timer.set(0.8, event);
                    sm.waitForSingleEvent(event, State.SPIN_CAROUSEL);
                    break;

                case SPIN_CAROUSEL:
                    // We touched the carousel, so stop the drive base.
                    robot.robotDrive.driveBase.stop();

                    // Spin the carousel.
                    robot.carouselSpinner.setPosition(
                            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                                    RobotParams.CAROUSEL_SPINNER_RED: RobotParams.CAROUSEL_SPINNER_BLUE,
                            RobotParams.CAROUSEL_SPINNER_SPIN_TIME, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_STORAGE_UNIT);
                    break;

                case DRIVE_TO_ALLIANCE_STORAGE_UNIT:
                    // Rotate Carousel Spinner Rotator to Red side
//                    robot.carouselSpinnerRotator.setLevel(0);

                    if (autoChoices.parking == FtcAuto.Parking.NO_PARKING)
                    {
                        // We are not parking anywhere, just stop and be done.
                        sm.setState(State.DONE);
                    }
                    // If we don't have enough time to go to the warehouse, park at the storage unit instead.
                    else if (autoChoices.parking == FtcAuto.Parking.WAREHOUSE_PARKING  &&
                            30.0 - elapsedTime > PARK_WAREHOUSE_TIME)
                    {
                        // We are parking at the warehouse.
                        sm.setState(State.DRIVE_TO_WAREHOUSE);
                    }
                    else
                    {
                        // Drive to storage unit. We could be coming from starting position, carousel or alliance hub.
                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                    event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(-2.5, -1.5, 90.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                    event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                    robot.robotDrive.pathPoint(-2.5, 1.5, 90.0));
                        }
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                case DRIVE_TO_WAREHOUSE:
                    // We are heading to the warehouse but we could be coming from starting position, carousel or
                    // alliance hub.
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.0, 0.0, 90.0),
                                robot.robotDrive.pathPoint(0.5, 0.0, 180.0),
                                robot.robotDrive.pathPoint(0.5, -1.5, 90.0));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(-2.0, 0.0, 90.0),
                                robot.robotDrive.pathPoint(0.5, 0.0, 0.0),
                                robot.robotDrive.pathPoint(0.5, 1.5, 90.0));
                    }
                    // Lift arm to go over barrier
                    robot.armRotator.setLevel(1);

                    sm.waitForSingleEvent(event, State.GET_INTO_WAREHOUSE);
                    break;

                case GET_INTO_WAREHOUSE:
                    // Run full speed into the warehouse crossing the barriers.
                    robot.robotDrive.driveBase.tankDrive(1.0, 1.0, false);
                    timer.set(1.0, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

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