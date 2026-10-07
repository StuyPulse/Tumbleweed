/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.constants;

import static org.wpilib.units.Units.*;

import org.wpilib.framework.RobotBase;
import org.wpilib.units.measure.*;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/** File containing non-subsystem settings for the robot. */
public interface GlobalSettings {
    Time DT = Milliseconds.of(20);

    /**
     * Uses a {@link LoggedNetworkBoolean} for each subsystem to add subsystem toggling
     * functionality from external dashboards.
     */
    interface EnabledSubsystems {}

    /** What mode the robot is in when running a simulation. */
    RobotMode SIMULATION_TASK = RobotMode.SIM;

    RobotMode CURRENT_MODE = RobotBase.isReal() ? RobotMode.REAL : SIMULATION_TASK;

    enum RobotMode {
        /** Running on a real robot. */
        REAL,

        /** Running in simulation. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
