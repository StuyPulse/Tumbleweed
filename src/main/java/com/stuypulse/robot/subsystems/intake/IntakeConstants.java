/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.intake;

import static org.wpilib.units.Units.*;

import com.stuypulse.robot.util.talonfx.TalonFXConfig;

import org.wpilib.units.measure.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public interface IntakeConstants {
    interface IntakeSettings {
        double STOP_DUTY_CYCLE = 0.0;
        double INTAKE_DUTY_CYCLE = 1.0;
        double OUTTAKE_DUTY_CYCLE = -1.0;

        Voltage STOP_VOLTAGE = Volts.zero();
        Voltage INTAKE_VOLTAGE = Volts.of(12.0);

        double GEAR_RATIO = 1.0 / 1.0; // placeholder
        Current STATOR_LIMIT = Amps.of(60);
        MomentOfInertia MOI = KilogramSquareMeters.of(0.0);
    }

    interface IntakeGains {}

    interface IntakeConfigs {
        TalonFXConfig ROLLER_MOTOR_CONFIG =
                new TalonFXConfig()
                        .withInvertedValue(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withSensorToMechanismRatio(IntakeSettings.GEAR_RATIO)
                        .withStatorCurrentLimit(IntakeSettings.STATOR_LIMIT);
    }

    interface IntakePorts {
        int ROLLER_MOTOR_PORT = 1;
    }
}
