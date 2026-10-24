/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.subsystems.intake.IntakeConstants.IntakeConfigs;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.IntakeSettings;

import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

abstract class IntakeIOTalonFXBase implements IntakeIO {
    private final TalonFX rollerMotor;

    private final DutyCycleOut dutyCycleController;
    private final VoltageOut voltageController;

    private final StatusSignal<Current> rollerMotorStatorCurrent;
    private final StatusSignal<Current> rollerMotorSupplyCurrent;
    private final StatusSignal<AngularVelocity> rollerMotorAngularVelocity;
    private final StatusSignal<Temperature> rollerMotorTemperature;
    private final StatusSignal<Voltage> rollerMotorAppliedVoltage;

    protected IntakeIOTalonFXBase(TalonFX rollerMotor) {
        this.rollerMotor = rollerMotor;

        IntakeConfigs.ROLLER_MOTOR_CONFIG.configure(this.rollerMotor);

        this.dutyCycleController =
                new DutyCycleOut(IntakeSettings.STOP_DUTY_CYCLE).withEnableFOC(true);
        this.voltageController = new VoltageOut(IntakeSettings.STOP_VOLTAGE).withEnableFOC(true);

        this.rollerMotorStatorCurrent = this.rollerMotor.getStatorCurrent();
        this.rollerMotorSupplyCurrent = this.rollerMotor.getSupplyCurrent();
        this.rollerMotorAngularVelocity = this.rollerMotor.getVelocity();
        this.rollerMotorTemperature = this.rollerMotor.getDeviceTemp();
        this.rollerMotorAppliedVoltage = this.rollerMotor.getMotorVoltage();
    }

    @Override
    public void readInputs(IntakeIORollerInputs inputs) {
        BaseStatusSignal.refreshAll(
                rollerMotorStatorCurrent,
                rollerMotorSupplyCurrent,
                rollerMotorAngularVelocity,
                rollerMotorTemperature,
                rollerMotorAppliedVoltage);

        inputs.rollerMotorStatorCurrent = rollerMotorStatorCurrent.getValue();
        inputs.rollerMotorSupplyCurrent = rollerMotorSupplyCurrent.getValue();
        inputs.rollerMotorAngularVelocity = rollerMotorAngularVelocity.getValue();
        inputs.rollerMotorTemperature = rollerMotorTemperature.getValue();
        inputs.rollerMotorAppliedVoltage = rollerMotorAppliedVoltage.getValue();
    }

    @Override
    public void applyOutputs(IntakeIORollerOutputs outputs) {
        switch (outputs.outputMode) {
            case STOP -> rollerMotor.stopMotor();
            case DUTY_CYCLE ->
                    rollerMotor.setControl(dutyCycleController.withOutput(outputs.targetDutyCycle));
            case VOLTAGE ->
                    rollerMotor.setControl(voltageController.withOutput(outputs.targetVoltage));
        }
    }
}
