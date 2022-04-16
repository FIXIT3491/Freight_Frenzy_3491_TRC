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
import teamcode.Season_Setup.Robot;
import teamcode.Season_Setup.RobotParams;

@SuppressWarnings("ConstantConditions")
class CmdAutoAllianceHubOnly_Time implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoNearCarousel_TimeOnly";

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

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdAutoAllianceHubOnly_Time(Robot robot, FtcAuto.AutoChoices autoChoices)
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

                    // Lift armRotator above ground, and rotate to front of robot
                    robot.armRotator.setLevel(0);
                    robot.armPlatformRotator.setLevel(0.5, 0);

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
                    // Drive to the alliance specific hub from the starting position.
                    robot.robotDrive.driveBase.tankDrive("Drive to Hub",0.5,
                            0.5,0.8, event);

                    // Raise arm to the detected duck level at the same time.
                    robot.armRotator.setLevel(3);

                    sm.waitForSingleEvent(event, State.DEPOSIT_FREIGHT);

                    break;

                case DEPOSIT_FREIGHT:
                    // Dumps the freight, when done, signals event and goes to next state.
                    robot.collector.setPosition(RobotParams.COLLECTOR_DEPOSIT_POWER, RobotParams.COLLECTOR_DEPOSITING_TIME, event);

                    sm.waitForSingleEvent(event, State.MOVE_BACK);
                    break;

                case MOVE_BACK:
                    robot.robotDrive.driveBase.tankDrive("Drive away from Hub",0.5,
                            0.5,0.2, event);


                    // Stops the Collector
                    robot.collector.setPosition(RobotParams.COLLECTOR_STOP_POWER);

                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
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