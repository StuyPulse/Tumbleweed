/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.intake;

import static org.wpilib.units.Units.KilogramSquareMeters;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.IntakeMotorConfiguration;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.IntakePorts;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.IntakeSettings;
import com.stuypulse.robot.util.talonfx.sim.SystemSim;
import com.stuypulse.robot.util.talonfx.sim.TalonFXSimulation;

import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;

public class IntakeIOTalonFXSim extends IntakeIOTalonFXBase {
    private final SystemSim<FlywheelSim> rollerSim;
    private final TalonFXSimulation rollerMotor;

    public IntakeIOTalonFXSim() {
        SystemSim<FlywheelSim> rollerSim =
                SystemSim.of(
                        new FlywheelSim(
                                Models.flywheelFromPhysicalConstants(
                                        IntakeMotorConfiguration.CURRENT_CONFIGURATION
                                                .getSimMotors(),
                                        IntakeSettings.MOI.in(KilogramSquareMeters),
                                        IntakeSettings.GEAR_RATIO),
                                IntakeMotorConfiguration.CURRENT_CONFIGURATION.getSimMotors()));
        TalonFXSimulation rollerMotor =
                new TalonFXSimulation(
                        IntakePorts.ROLLER_MOTOR_PORT, IntakeSettings.GEAR_RATIO, rollerSim);
        super(rollerMotor);
        this.rollerSim = rollerSim;
        this.rollerMotor = rollerMotor;
    }

    @Override
    public void readInputs(IntakeIORollerInputs inputs) {
        rollerSim.update(GlobalSettings.DT);

        rollerMotor.refresh();

        super.readInputs(inputs);
    }
}
