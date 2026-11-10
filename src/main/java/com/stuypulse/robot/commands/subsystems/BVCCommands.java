/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.commands.subsystems;

import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.subsystems.swerve.SwerveConstants.SwerveGains;
import com.stuypulse.robot.util.bvc.ConstraintCalculator;

import org.wpilib.command3.Command;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public interface BVCCommands {
    public static Command PIDToPose(Swerve swerve) {
        return PIDToPose(swerve, "Pose");
    }

    /**
     * uses PID to drive towards the target provided by BVC
     *
     * @param swerve Swerve subsystem instance
     * @param targetName Name of the target you want to approach
     * @return repeating command that drives towards the target
     */
    public static Command PIDToPose(Swerve swerve, String targetName) {
        final PIDController xController =
                new PIDController(
                        SwerveGains.Translation.kP,
                        SwerveGains.Translation.kI,
                        SwerveGains.Translation.kD);
        final PIDController yController =
                new PIDController(
                        SwerveGains.Translation.kP,
                        SwerveGains.Translation.kI,
                        SwerveGains.Translation.kD);
        return swerve.runRepeatedly(
                        () -> {
                            Pose2d self = swerve.getPose();
                            Pose2d target = ConstraintCalculator.getTarget();
                            ChassisVelocities speeds =
                                    new ChassisVelocities(
                                            xController.calculate(self.getX(), target.getX()),
                                            yController.calculate(self.getY(), target.getY()),
                                            0);
                            swerve.runVelocity(speeds);
                        })
                .whenExited(
                        () -> {
                            xController.reset();
                            yController.reset();
                        })
                .named("PIDTo" + targetName);
    }

    public static Command PIDToRyan(Swerve swerve, String targetName) {
        final PIDController xController =
                new PIDController(
                        SwerveGains.Translation.kP,
                        SwerveGains.Translation.kI,
                        SwerveGains.Translation.kD);
        final PIDController yController =
                new PIDController(
                        SwerveGains.Translation.kP,
                        SwerveGains.Translation.kI,
                        SwerveGains.Translation.kD);
        return swerve.runRepeatedly(
                        () -> {
                            Pose2d self = swerve.getPose();
                            Pose2d target = Pose2d.ZERO;
                            ChassisVelocities speeds =
                                    new ChassisVelocities(
                                            xController.calculate(self.getX(), target.getX()),
                                            yController.calculate(self.getY(), target.getY()),
                                            0);
                            swerve.runVelocity(speeds);
                        })
                .whenExited(
                        () -> {
                            xController.reset();
                            yController.reset();
                        })
                .named("PIDTo" + targetName);
    }
}
