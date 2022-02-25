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

import TrcCommonLib.command.CmdPidDrive;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import teamcode.Season_Setup.Freight_Frenzy_Pipeline;
import teamcode.Season_Setup.Robot;
import teamcode.Season_Setup.RobotParams;

class CmdAutoAllianceHubOnly implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoNearCarousel";

    private enum State
    {
        START_DELAY,
        DRIVE_TO_ALLIANCE_SHIPPING_HUB,
        DEPOSIT_FREIGHT,
        MOVE_BACK,

        DONE
    }   // enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private Freight_Frenzy_Pipeline.ElementInfo elementInfo;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdAutoAllianceHubOnly(Robot robot, FtcAuto.AutoChoices autoChoices)
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
                            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ?
                                    RobotParams.STARTPOS_RED_NEAR : RobotParams.STARTPOS_BLUE_NEAR);

                    // Lift armRotator above ground, and rotate to front of robot, and lower arm.
                    robot.armRotator.setLevel(0);
                    robot.armPlatformRotator.setLevel(0.5, 0);

                    // Set to highest Arm level
                    elementInfo = robot.vision.getElementInfo();
                    elementInfo.elementPosition = 3;

                    // Do start delay if any.
                    if (autoChoices.startDelay == 0.0) {
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
                    // Note: the smaller the number the closer to the hub.
                    double distanceToHub = elementInfo.elementPosition == 3 ? 1.1 : elementInfo.elementPosition == 2 ? 1.3 : 1.0;

                    // Drive to the alliance specific hub from the starting position.
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                    robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), true,
                            new TrcPose2D(0.0,26.0+distanceToHub, 0.0));

                    // Raise arm to the detected duck level at the same time.
                    robot.armRotator.setLevel(elementInfo.elementPosition);

                    sm.waitForSingleEvent(event, State.DEPOSIT_FREIGHT);

                    break;

                case DEPOSIT_FREIGHT:
                    // Dumps the freight, when done, signals event and goes to next state.
                    robot.collector.setPosition(RobotParams.COLLECTOR_DEPOSIT_POWER, RobotParams.COLLECTOR_DEPOSITING_TIME, event);

                    sm.waitForSingleEvent(event, State.MOVE_BACK);
                    break;

                case MOVE_BACK:
                    robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), true,
                            new TrcPose2D(0.0,-10.0, 0.0));

                    // Stops the Collector
                    robot.collector.setPosition(RobotParams.COLLECTOR_STOP_POWER);

                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    // We are done, lower arm.
                    robot.armRotator.setTarget(0);

                    cancel();
                    break;
            }

            if (traceState)
            {
                robot.globalTracer.traceStateInfo(
                        state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive, robot.robotDrive.purePursuitDrive,
                        null);
            }
        }

        return !sm.isEnabled();
    }   // cmdPeriodic

}   // class CmdAutoAllianceHubOnly