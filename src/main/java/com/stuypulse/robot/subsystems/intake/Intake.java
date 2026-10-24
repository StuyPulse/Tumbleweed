/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.IntakeSettings;
import com.stuypulse.robot.subsystems.intake.IntakeIO.IntakeIORollerOutputMode;
import com.stuypulse.robot.subsystems.intake.IntakeIO.IntakeIORollerOutputs;
import com.stuypulse.robot.util.PeriodicSubsystem;

import org.wpilib.command3.Command;
import org.wpilib.units.measure.Voltage;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends PeriodicSubsystem {
    private final IntakeIO io;
    private final IntakeIORollerInputsAutoLogged inputs;
    private final IntakeIORollerOutputs outputs;

    @AutoLogOutput(key = "States/Intake")
    private IntakeState state;

    public Intake(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeIORollerInputsAutoLogged();
        this.outputs = new IntakeIORollerOutputs();

        this.state = IntakeState.STOP;
    }

    private enum IntakeState {
        STOP,
        INTAKE,
        OUTTAKE
    }

    private Command commandIntakeState(IntakeState state) {
        return run(coroutine -> this.state = state).named(getName() + "Set" + state.name());
    }

    public Command commandIntakeStopState() {
        return commandIntakeState(IntakeState.STOP);
    }

    public Command commandIntakeIntakeState() {
        return commandIntakeState(IntakeState.INTAKE);
    }

    public Command commandIntakeOuttakeState() {
        return commandIntakeState(IntakeState.OUTTAKE);
    }

    private void runIntakeStop() {
        this.outputs.outputMode = IntakeIORollerOutputMode.STOP;
    }

    private void runIntakeDutyCycle(double targetDutyCycle) {
        this.outputs.outputMode = IntakeIORollerOutputMode.DUTY_CYCLE;
        this.outputs.targetDutyCycle = targetDutyCycle;
    }

    @SuppressWarnings("unused")
    private void runIntakeVoltage(Voltage targetVoltage) {
        this.outputs.outputMode = IntakeIORollerOutputMode.VOLTAGE;
        this.outputs.targetVoltage = targetVoltage;
    }

    @Override
    protected void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);

        if (!GlobalSettings.EnabledSubsystems.INTAKE.get()) {
            this.runIntakeStop();
            return;
        }

        switch (this.state) {
            case STOP -> runIntakeStop();
            case INTAKE -> runIntakeDutyCycle(IntakeSettings.INTAKE_DUTY_CYCLE);
            case OUTTAKE -> runIntakeDutyCycle(IntakeSettings.OUTTAKE_DUTY_CYCLE);
        }
    }

    @Override
    protected void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }
}
