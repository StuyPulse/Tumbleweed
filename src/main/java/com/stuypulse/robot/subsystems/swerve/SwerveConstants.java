/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.swerve;

import static org.wpilib.units.Units.*;

import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.*;

public interface SwerveConstants {
    public interface SwerveConstraints {
        LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4.16);
        LinearVelocity MAX_VELOCITY_SOTM = MetersPerSecond.of(1.75);
        LinearVelocity MAX_VELOCITY_FOTM = MetersPerSecond.of(4.16);

        AngularVelocity MAX_ANGULAR_VEL = DegreesPerSecond.of(300.0);
        AngularVelocity MAX_ANGULAR_VEL_SOTM = DegreesPerSecond.of(75.0);
        AngularVelocity MAX_ANGULAR_VEL_FOTM = DegreesPerSecond.of(150.0);

        LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(15.0);
        LinearAcceleration MAX_ACCEL_SOTM = MetersPerSecondPerSecond.of(4.0);
        LinearAcceleration MAX_ACCEL_FOTM = MetersPerSecondPerSecond.of(15.0);
        AngularAcceleration MAX_ANGULAR_ACCEL_RAD_PER_S_SQUARED =
                DegreesPerSecondPerSecond.of(900.0);
    }

    public interface SwerveCharacterization {
        Time FF_START_DELAY = Seconds.of(2.0);
        double FF_RAMP_RATE = 0.1;

        double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
        double WHEEL_RADIUS_RAMP_RATE = 0.05;

        Time SYSID_REST = Seconds.of(1.5);

        public interface DriveSysId {
            Velocity<VoltageUnit> RAMP_RATE = Volts.of(1).per(Second);
            Voltage STEP_VOLTAGE = Volts.of(4);
            Time QUASISTATIC_TIMEOUT = Seconds.of(5);
            Time DYNAMIC_TIMEOUT = Seconds.of(2);
        }

        public interface SteerSysId {
            Velocity<VoltageUnit> RAMP_RATE = Volts.of(1).per(Second);
            Voltage STEP_VOLTAGE = Volts.of(7);
            Time QUASISTATIC_TIMEOUT = Seconds.of(6);
            Time DYNAMIC_TIMEOUT = Seconds.of(2);
        }
    }
}
