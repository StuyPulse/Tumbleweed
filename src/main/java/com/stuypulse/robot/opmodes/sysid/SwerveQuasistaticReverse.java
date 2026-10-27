/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.opmodes.sysid;

import static org.wpilib.units.Units.Volts;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.subsystems.swerve.SwerveConstants.SwerveCharacterization.DriveSysId;
import com.stuypulse.robot.util.SysIdRoutine;
import com.stuypulse.robot.util.SysIdRoutine.Direction;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;

import org.littletonrobotics.junction.Logger;

@Utility(name = "Quasistatic Reverse", group = "SysId")
public class SwerveQuasistaticReverse implements OpMode {
    private final Swerve swerve;
    private final Command routine;

    public SwerveQuasistaticReverse(Robot robot, Command sysIdCommand) {
        swerve = robot.getContainer().getSwerve();

        SysIdRoutine.Config config =
                new SysIdRoutine.Config(
                        DriveSysId.RAMP_RATE,
                        DriveSysId.STEP_VOLTAGE,
                        DriveSysId.QUASISTATIC_TIMEOUT,
                        state -> Logger.recordOutput("SysId/SwerveDrive/State", state.toString()));

        SysIdRoutine.SysIdMechanism mechanism =
                new SysIdRoutine.SysIdMechanism(
                        volts -> swerve.runCharacterization(volts.in(Volts)),
                        null,
                        swerve,
                        "SwerveDrive");

        SysIdRoutine sysId = new SysIdRoutine(config, mechanism);

        routine = sysId.quasistatic(Direction.REVERSE);
    }

    @Override
    public void start() {
        Scheduler.getDefault().schedule(routine);
    }

    @Override
    public void end() {
        Scheduler.getDefault().cancel(routine);
    }
}
