/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.intake;

import static org.wpilib.units.Units.*;

import org.wpilib.hardware.bus.CANPort;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.Signal;

import java.util.function.Supplier;

public class IntakeIOSpark implements IntakeIO {
    protected final SparkMax rollerMotor;

    private final Signal<Double> rollerMotorStatorCurrent;
    private final Supplier<Double> rollerMotorSupplyCurrent;
    private final Signal<Double> rollerMotorAngularVelocity;
    private final Signal<Double> rollerMotorTemperature;
    private final Signal<Double> rollerMotorAppliedVoltage;

    public IntakeIOSpark() {
        this.rollerMotor =
                new SparkMax(
                        CANPort.CAN_S1,
                        IntakeConstants.IntakePorts.ROLLER_MOTOR_PORT,
                        MotorType.kBrushless);

        this.rollerMotorStatorCurrent = rollerMotor.getOutputCurrent();
        this.rollerMotorSupplyCurrent =
                () ->
                        rollerMotorStatorCurrent.get()
                                * Math.abs(rollerMotor.getAppliedOutput().get()); // gg
        this.rollerMotorAngularVelocity = rollerMotor.getAbsoluteEncoder().getVelocity();
        this.rollerMotorTemperature = rollerMotor.getMotorTemperature();
        this.rollerMotorAppliedVoltage = rollerMotor.getBusVoltage();
    }

    @Override
    public void updateInputs(IntakeIORollerInputs inputs) {
        inputs.rollerMotorStatorCurrent = Amps.of(rollerMotorStatorCurrent.get());
        inputs.rollerMotorSupplyCurrent = Amps.of(rollerMotorSupplyCurrent.get());
        inputs.rollerMotorAngularVelocity = RPM.of(rollerMotorAngularVelocity.get());
        inputs.rollerMotorTemperature = Celsius.of(rollerMotorTemperature.get());
        inputs.rollerMotorAppliedVoltage = Volts.of(rollerMotorAppliedVoltage.get());
    }

    @Override
    public void applyOutputs(IntakeIORollerOutputs outputs) {
        switch (outputs.outputMode) {
            case STOP -> rollerMotor.stopMotor();
            case DUTY_CYCLE -> rollerMotor.setThrottle(outputs.targetDutyCycle);
            case VOLTAGE -> rollerMotor.setVoltage(outputs.targetVoltage);
        }
    }
}
