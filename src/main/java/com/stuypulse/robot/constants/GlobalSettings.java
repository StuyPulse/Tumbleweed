/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.constants;

import static org.wpilib.units.Units.*;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
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

    interface DriveSettings {
        public interface Constraints {
            LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4.3);

            LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(20.0);

            AngularVelocity MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(400.0);

            AngularAcceleration MAX_ANGULAR_ACCEL = DegreesPerSecondPerSecond.of(300.0);
        }

        interface Alignment {
            interface Tolerances {
                Distance X_TOLERANCE = Inches.of(2.0);

                Distance Y_TOLERANCE = Inches.of(2.0);

                Rotation2d THETA_TOLERANCE = Rotation2d.fromDegrees(8);

                Pose2d POSE_TOLERANCE =
                        new Pose2d(X_TOLERANCE.in(Meters), Y_TOLERANCE.in(Meters), THETA_TOLERANCE);

                LinearVelocity MAX_VELOCITY_WHEN_ALIGNED = MetersPerSecond.of(0.15);

                Time ALIGNMENT_DEBOUNCE = Seconds.of(0.15);
            }
        }
    }

    interface DriveGains {
        interface Alignment {
            double akP = 8.8;
            double akI = 0.0;
            double akD = 0.0;
        }
    }

    interface DriverConstants {
        public interface Driving {
            double DEADBAND = 0.05;
            double RC = 0.05;
            double POWER = 2.0;
        }

        public interface Turning {
            double DEADBAND = 0.07;
            double RC = 0.05;
            double POWER = 2.0;
        }

        double ANGLE_KP = 5.0;
        double ANGLE_KI = 0.0;
        double ANGLE_KD = 0.4;
        double ANGLE_MAX_VELOCITY = 8.0;
        double ANGLE_MAX_ACCELERATION = 20.0;
        Time FF_START_DELAY = Seconds.of(2.0); // Secs
        double FF_RAMP_RATE = 0.1; // Volts/Sec
        double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
        double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
    }
}
