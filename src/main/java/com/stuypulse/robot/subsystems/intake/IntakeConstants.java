/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.intake;

import static org.wpilib.units.Units.*;

import com.stuypulse.robot.util.talonfx.TalonFXConfig;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.function.IntFunction;

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

    enum IntakeMotorConfiguration {
        NEO_1POINT1(DCMotor::getNEO),
        NEO_VORTEX(DCMotor::getNeoVortex),
        KRAKEN_X60(DCMotor::getKrakenX60),
        KRAKEN_X44(DCMotor::getKrakenX44);

        private final IntFunction<DCMotor> motorGetter;

        private IntakeMotorConfiguration(IntFunction<DCMotor> motorGetter) {
            this.motorGetter = motorGetter;
        }

        public DCMotor getSimMotors() {
            return motorGetter.apply(NUM_MOTORS);
        }

        public static final IntakeMotorConfiguration CURRENT_CONFIGURATION = NEO_VORTEX;
        private static final int NUM_MOTORS = 1;
    }

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
