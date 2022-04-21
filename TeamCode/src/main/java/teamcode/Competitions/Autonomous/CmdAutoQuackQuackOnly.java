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
import TrcFtcLib.ftclib.FtcChoiceMenu;
import teamcode.Season_Setup.Robot;
import teamcode.Season_Setup.RobotParams;

@SuppressWarnings("ConstantConditions")
class CmdAutoQuackQuackOnly implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoNearCarousel";

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;


    private enum State
    {
        START_DELAY,
        PREPARE_TO_SPIN_DUCK,
        SPIN_CAROUSEL,
        RETRACT_CAROUSEL,

        DONE
    }   // enum State


    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdAutoQuackQuackOnly(Robot robot, FtcAuto.AutoChoices autoChoices)
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

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state) {
                case START_DELAY:
                    // Reset Position of robot
                    robot.robotDrive.driveBase.setFieldPosition(new TrcPose2D(0.0, 0.0, 0.0));

                    // Do start delay if any.
                    if (autoChoices.startDelay == 0.0) {
                        // Intentionally falling through to the next state.
                        sm.setState(State.PREPARE_TO_SPIN_DUCK);
                    }
                    else
                    {
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.PREPARE_TO_SPIN_DUCK);
                        break;
                    }

                case PREPARE_TO_SPIN_DUCK:

                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.carouselExtenderOne.setPosition(RobotParams.CAROUSEL_EXTENDER_ONE_EXTEND_POWER);
                        robot.carouselExtenderTwo.setPosition(RobotParams.CAROUSEL_EXTENDER_TWO_EXTEND_POWER);
                        timer.set(RobotParams.CAROUSEL_EXTENDER_ONE_EXTENDING_TIME, event);
                    }
                    else
                    {
                        robot.robotDrive.driveBase.setFieldPosition(new TrcPose2D(0.0, 0.0, 0.0));

                        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);

                        robot.robotDrive.purePursuitDrive.start(
                                robot.robotDrive.driveBase.getFieldPosition(), true,
                                new TrcPose2D(0.0,-16.0, 0.0));
                    }

                    sm.waitForSingleEvent(event, State.SPIN_CAROUSEL);

                    break;

                case SPIN_CAROUSEL:
                    // We touched the carousel, so stop the drive base.
                    robot.robotDrive.driveBase.stop();

                    // Spin the carousel.
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.carouselExtenderOne.setPosition(RobotParams.CAROUSEL_EXTENDER_ONE_STOP_POWER);
                        robot.carouselExtenderTwo.setPosition(RobotParams.CAROUSEL_EXTENDER_TWO_STOP_POWER);

                        robot.carouselSpinner.setPosition(RobotParams.CAROUSEL_SPINNER_RED,
                                RobotParams.CAROUSEL_SPINNER_SPIN_TIME, event);
                        sm.waitForSingleEvent(event, State.RETRACT_CAROUSEL);
                    }
                    else
                    {
                        robot.carouselSpinner.setPosition(RobotParams.CAROUSEL_SPINNER_BLUE,
                                RobotParams.CAROUSEL_SPINNER_SPIN_TIME, event);
                        sm.waitForSingleEvent(event, State.DONE);
                    }

                    break;

                case RETRACT_CAROUSEL:
                    robot.robotDrive.pidDrive.setRelativeTarget(
                            0.0,-4.0,0.0, event, 5.0);

                    // TODO: add DONE state for blue alliance
                    // Stops the Collector
                    robot.carouselSpinner.setPosition(RobotParams.CAROUSEL_SPINNER_STOP_POWER);

                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        // Retract Carousel
                        robot.carouselExtenderOne.setPosition(RobotParams.CAROUSEL_EXTENDER_ONE_RETRACT_POWER);
                        robot.carouselExtenderTwo.setPosition(RobotParams.CAROUSEL_EXTENDER_TWO_RETRACT_POWER);
                        timer.set(RobotParams.CAROUSEL_EXTENDER_ONE_EXTENDING_TIME, event);
                    }

                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    // Retract Carousel
                    robot.carouselExtenderOne.setPosition(RobotParams.CAROUSEL_EXTENDER_ONE_STOP_POWER);
                    robot.carouselExtenderTwo.setPosition(RobotParams.CAROUSEL_EXTENDER_TWO_STOP_POWER);


                    // We are done, lower arm.
                    robot.armRotator.setTarget(1);

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

}   // class CmdAutoAllianceHubOnly