/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.intake;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface IntakeIO {
    @AutoLog
    class IntakeIORollerInputs {
        Current rollerMotorStatorCurrent = Amps.zero();
        Current rollerMotorSupplyCurrent = Amps.zero();
        AngularVelocity rollerMotorAngularVelocity = RPM.zero();
        Temperature rollerMotorTemperature = Kelvin.zero();
        Voltage rollerMotorAppliedVoltage = Volts.zero();
    }

    enum IntakeIORollerOutputMode {
        STOP,
        VOLTAGE,
        DUTY_CYCLE
    }

    class IntakeIORollerOutputs {
        @AutoLogOutput(key = "Intake/Output Mode")
        IntakeIORollerOutputMode outputMode = IntakeIORollerOutputMode.STOP;

        @AutoLogOutput(key = "Intake/Target Duty Cycle")
        double targetDutyCycle = 0.0;

        @AutoLogOutput(key = "Intake/Target Voltage")
        Voltage targetVoltage = Volts.zero();
    }

    public default void readInputs(IntakeIORollerInputs inputs) {}

    public default void applyOutputs(IntakeIORollerOutputs outputs) {}
}
