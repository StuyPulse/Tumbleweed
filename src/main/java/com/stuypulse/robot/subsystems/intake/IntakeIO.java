/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.intake;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.*;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIORollerInputs {
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

    public class IntakeIORollerOutputs {
        IntakeIORollerOutputMode outputMode = IntakeIORollerOutputMode.STOP;
        double targetDutyCycle = 0.0;
        Voltage targetVoltage = Volts.zero();
    }

    public default void updateInputs(IntakeIORollerInputs inputs) {}

    public default void applyOutputs(IntakeIORollerOutputs outputs) {}
}
