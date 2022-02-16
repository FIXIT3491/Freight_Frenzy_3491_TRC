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

import java.util.Arrays;

import TrcCommonLib.trclib.TrcDashboard;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidController.PidCoefficients;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import teamcode.Season_Setup.Robot;

/**
 * This class implements a generic PID control drive command. It is agnostic to the PID controller sensors.
 * The caller provides the PID drive object that has all PID controllers which means the caller controls
 * what sensors are controlling the X, Y and turn PID controllers. For example, the caller can provide a PID
 * drive object that uses the encoders to control the X and Y PID controllers and a gyro for the turn PID
 * controller. The caller can also use the encoders to control the X and Y PID controllers but a camera to
 * control the turn PID controller.
 */
public class CmdAuto implements TrcRobot.RobotCommand
{
    private enum State
    {
        DO_DELAY,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAuto";

    private final TrcDashboard dashboard = TrcDashboard.getInstance();
    private final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private final Robot robot;
    private final Autonomous.AutoChoices autoChoices;

    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;


    /**
     * Constructor: Create an instance of the object.
     */
    public CmdAuto(
            Robot robot, Autonomous.AutoChoices autoChoices)
    {

        globalTracer.traceInfo(
            moduleName,
            "Robot = %s, Auto Choices = %s",
            robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);



        sm.start(State.DO_DELAY);
    }   //CmdPidDrive

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {

        sm.stop();
    }   //cancel

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
            dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            dashboard.displayPrintf(1, "State: %s", state);

            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (autoChoices.startDelay == 0.0)
                    {
                        sm.setState(State.DONE);
                        //
                        // Intentionally falling through to DO_PID_DRIVE.
                        //
                    }
                    else
                    {
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.DONE);
                        break;
                    }

                case DONE:
                default:
                    //
                    // We are done, restore everything.
                    //
                    cancel();
                    break;
            }

            globalTracer.traceStateInfo(state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdPidDrive
