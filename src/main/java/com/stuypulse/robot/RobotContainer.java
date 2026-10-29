/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.AutonomousRoutines;
import com.stuypulse.robot.commands.subsystems.DriveCommands;
import com.stuypulse.robot.constants.GlobalPorts;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.generated.TunerConstants;
import com.stuypulse.robot.subsystems.swerve.GyroIO;
import com.stuypulse.robot.subsystems.swerve.GyroIOOnboard;
import com.stuypulse.robot.subsystems.swerve.GyroIOPigeon2;
import com.stuypulse.robot.subsystems.swerve.ModuleIO;
import com.stuypulse.robot.subsystems.swerve.ModuleIOSim;
import com.stuypulse.robot.subsystems.swerve.ModuleIOTalonFX;
import com.stuypulse.robot.subsystems.swerve.Swerve;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Selectable;

public class RobotContainer {

    // Gamepads
    public final CommandGamepad driver =
            new CommandGamepad(GlobalPorts.Gamepad.DRIVER); // model-agnostic controller
    // public final CommandGamepad operator = new
    // CommandGamepad(GlobalPorts.Gamepad.OPERATOR);

    // Subsystems

    private final Swerve swerve;
    // Autons
    private static Selectable<Command> autonChooser = new Selectable<>();

    // Robot container

    public RobotContainer() {
        switch (GlobalSettings.CURRENT_MODE) {
            case REAL -> {
                swerve =
                        new Swerve(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight),
                                new GyroIOOnboard());
            }
            case SIM -> {
                swerve =
                        new Swerve(
                                new GyroIO() {},
                                new ModuleIOSim(TunerConstants.FrontLeft),
                                new ModuleIOSim(TunerConstants.FrontRight),
                                new ModuleIOSim(TunerConstants.BackLeft),
                                new ModuleIOSim(TunerConstants.BackRight),
                                new GyroIO() {});
            }
            default -> {
                swerve =
                        new Swerve(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new GyroIO() {});
            }
        }
        configureLogging();
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /***************/
    /*** LOGGING ***/
    /***************/

    private void configureLogging() {}

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                DriveCommands.joystickDrive(
                        swerve, driver::getLeftY, driver::getLeftX, () -> -driver.getRightX()));
    }

    /***************/

    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        driver.dpadLeft()
                .onTrue(
                        Command.noRequirements(coroutine -> swerve.resetHeading(Rotation2d.ZERO))
                                .named("Swerve Reset Heading"));

        driver.faceLeft().onTrue(DriveCommands.xMode(swerve));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.addDefault("Do Nothing", AutonomousRoutines.doNothingAuton());

        // // Set up SysId routines
        // autonChooser.add(
        //         "Drive Wheel Radius Characterization",
        // DriveCommands.wheelRadiusCharacterization(drive));
        // autonChooser.add(
        //         "Drive Simple FF Characterization",
        // DriveCommands.feedforwardCharacterization(drive));
        // autonChooser.add(
        //         "Drive SysId (Quasistatic Forward)",
        //         drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autonChooser.add(
        //         "Drive SysId (Quasistatic Reverse)",
        //         drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autonChooser.add(
        //         "Drive SysId (Dynamic Forward)",
        // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autonChooser.add(
        //         "Drive SysId (Dynamic Reverse)",
        // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        Telemetry.log("Autonomous", autonChooser);
    }

    public Swerve getSwerve() {
        return swerve;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return The command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
