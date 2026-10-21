/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.commands.subsystems;

import static org.wpilib.units.Units.Seconds;

import com.stuypulse.robot.constants.GlobalSettings.DriverConstants;
import com.stuypulse.robot.subsystems.swerve.Swerve;

import org.wpilib.command3.*;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public interface DriveCommands {
    public static Command resetPose(final Swerve drive, final Pose2d pose) {
        return drive.run(
                        coroutine -> {
                            drive.resetPose(pose);
                        })
                .named(drive.getName() + "ResetPose");
    }

    public static Command resetHeading(final Swerve drive, final Rotation2d heading) {
        return drive.run(
                        coroutine -> {
                            drive.resetHeading(heading);
                        })
                .named(drive.getName() + "ResetHeading");
    }

    public static Command xMode(final Swerve drive) {
        return drive.run(coroutine -> drive.stopWithX()).named(drive.getName() + "XMode");
    }

    // private static Command alignToTarget(final Target target, final Swerve drive) {
    //     final Pose2d targetPose = target.getPose();
    //     final Debouncer alignmentDebouncer = new Debouncer(
    //             DriveSettings.Alignment.Tolerances.ALIGNMENT_DEBOUNCE.in(Seconds),
    // DebounceType.kBoth);
    //     final PIDController headingController = new PIDController(DriveGains.Alignment.akP,
    // DriveGains.Alignment.akI,
    //             DriveGains.Alignment.akD);
    //     headingController.enableContinuousInput(-Math.PI, Math.PI);
    //
    // headingController.setTolerance(DriveSettings.Alignment.Tolerances.THETA_TOLERANCE.getRadians());
    //     final BooleanSupplier isAligned = headingController::atSetpoint;

    //     return drive.run(coroutine -> {
    //         headingController.reset();

    //         while (!alignmentDebouncer.calculate(isAligned.getAsBoolean())) {
    //             double omega =
    // headingController.calculate(drive.getPose().getRotation().getRadians(),
    //                     getTargetAlignmentAngle(drive.getPose(), targetPose).getRadians());
    //             ChassisVelocities speeds = new ChassisVelocities(
    //                     0,
    //                     0,
    //                     omega * drive.getMaxAngularSpeedRadPerSec());
    //             drive.runVelocity(speeds.toRobotRelative(drive.getRotation()));
    //             coroutine.yield();
    //         }
    //     })
    //             .whenCanceled(headingController::close)
    //             .named(drive.getName() + "AlignTo" + target.name());
    // }

    // public static Command alignToScale(final Swerve drive) {
    //     return alignToTarget(Field.getScaleTarget(), drive);
    // }

    // public static Command alignToSwitch(final Swerve drive) {
    //     return alignToTarget(Field.getSwitchTarget(), drive);
    // }

    // private static Rotation2d getTargetAlignmentAngle(final Pose2d currentPose, final Pose2d
    // targetPose) {
    //     double atan = Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() -
    // currentPose.getX());
    //     return new Rotation2d((atan));
    // }

    // public static Translation2d getLinearVelocityFromJoysticks(double x, double
    // y) {
    // // Apply deadband
    // double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y),
    // DriverConstants.Driving.DEADBAND);
    // Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // // Square magnitude for more precise control
    // linearMagnitude = linearMagnitude * linearMagnitude;

    // // Return new linear velocity
    // return new Pose2d(new Translation2d(), linearDirection)
    // .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
    // .getTranslation();
    // }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    // public static Command joystickDrive(
    // Swerve drive,
    // DoubleSupplier xSupplier,
    // DoubleSupplier ySupplier,
    // DoubleSupplier omegaSupplier) {
    // return drive
    // .run(
    // coroutine -> {
    // // Get linear velocity
    // Translation2d linearVelocity =
    // getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
    // ySupplier.getAsDouble());

    // // Apply rotation deadband
    // double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(),
    // DriverConstants.Driving.DEADBAND);

    // // Square rotation value for more precise control
    // omega = Math.copySign(omega * omega, omega);

    // // Convert to field relative speeds & send command
    // ChassisVelocities speeds = new ChassisVelocities(
    // linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
    // linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
    // omega * drive.getMaxAngularSpeedRadPerSec());
    // boolean isFlipped = MatchState.getAlliance().isPresent()
    // && MatchState.getAlliance().get() == Alliance.RED;
    // drive.runVelocity(
    // speeds.toFieldRelative(
    // isFlipped
    // ? drive.getRotation().plus(new Rotation2d(Math.PI))
    // : drive.getRotation()));
    // })
    // .named("Joystick Swerve");
    // }

    // public static Command drive(Swerve drive, CommandGamepad joystick) {
    // final DriveInputProcessor speed = new DriveInputProcessor(
    // joystick,
    // DriverConstants.Driving.DEADBAND,
    // DriverConstants.Driving.POWER,
    // GlobalSettings.DriveSettings.Constraints.MAX_VELOCITY,
    // GlobalSettings.DriveSettings.Constraints.MAX_ACCEL,
    // Driving.RC);
    // final DriveTurnInputProcessor turn = new DriveTurnInputProcessor(
    // joystick,
    // Turning.DEADBAND,
    // Turning.POWER,
    // DriveSettings.Constraints.MAX_ANGULAR_VELOCITY, Turning.RC);
    // return drive.run(coroutine -> {
    // while (true) {
    // speed.update();
    // turn.update();

    // ChassisVelocities fieldRelative = new ChassisVelocities(
    // speed.get().getX(),
    // speed.get().getY(),
    // -turn.get());

    // drive.runVelocity(fieldRelative.toRobotRelative(drive.getRotation()));

    // coroutine.yield();
    // }
    // }).named("FixedDriveCommand");
    // }
    public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude =
                MathUtil.applyDeadband(Math.hypot(x, y), DriverConstants.Driving.DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            Swerve drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return drive.run(
                        coroutine -> {
                            while (true) {
                                // Get linear velocity
                                Translation2d linearVelocity =
                                        getLinearVelocityFromJoysticks(
                                                xSupplier.getAsDouble(), ySupplier.getAsDouble());

                                // Apply rotation deadband
                                double omega =
                                        MathUtil.applyDeadband(
                                                omegaSupplier.getAsDouble(),
                                                DriverConstants.Driving.DEADBAND);

                                // Square rotation value for more precise control
                                omega = Math.copySign(omega * omega, omega);

                                // Convert to field relative speeds & send command
                                ChassisVelocities speeds =
                                        new ChassisVelocities(
                                                linearVelocity.getX()
                                                        * drive.getMaxLinearSpeedMetersPerSec(),
                                                linearVelocity.getY()
                                                        * drive.getMaxLinearSpeedMetersPerSec(),
                                                omega * drive.getMaxAngularSpeedRadPerSec());
                                boolean isFlipped =
                                        MatchState.getAlliance().isPresent()
                                                && MatchState.getAlliance().get() == Alliance.RED;
                                drive.runVelocity(
                                        speeds.toRobotRelative(
                                                isFlipped
                                                        ? drive.getRotation()
                                                                .plus(new Rotation2d(Math.PI))
                                                        : drive.getRotation()));

                                coroutine.yield();
                            }
                        })
                .named("Joystick Swerve");
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Swerve drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier) {
        // Create PID controller
        ProfiledPIDController angleController =
                new ProfiledPIDController(
                        DriverConstants.ANGLE_KP,
                        DriverConstants.ANGLE_KI,
                        DriverConstants.ANGLE_KD,
                        new TrapezoidProfile.Constraints(
                                DriverConstants.ANGLE_MAX_VELOCITY,
                                DriverConstants.ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        Command joystickDrive =
                drive.run(
                                coroutine -> {
                                    // Get linear velocity
                                    Translation2d linearVelocity =
                                            getLinearVelocityFromJoysticks(
                                                    xSupplier.getAsDouble(),
                                                    ySupplier.getAsDouble());

                                    // Calculate angular speed
                                    double omega =
                                            angleController.calculate(
                                                    drive.getRotation().getRadians(),
                                                    rotationSupplier.get().getRadians());

                                    // Convert to field relative speeds & send command
                                    ChassisVelocities speeds =
                                            new ChassisVelocities(
                                                    linearVelocity.getX()
                                                            * drive.getMaxLinearSpeedMetersPerSec(),
                                                    linearVelocity.getY()
                                                            * drive.getMaxLinearSpeedMetersPerSec(),
                                                    omega);
                                    boolean isFlipped =
                                            MatchState.getAlliance().isPresent()
                                                    && MatchState.getAlliance().get()
                                                            == Alliance.RED;
                                    drive.runVelocity(
                                            speeds.toFieldRelative(
                                                    isFlipped
                                                            ? drive.getRotation()
                                                                    .plus(new Rotation2d(Math.PI))
                                                            : drive.getRotation()));
                                })
                        .named("Joystick Swerve at Angle");

        // Reset PID controller when command starts
        Command resetAngleController =
                drive.run(coroutine -> angleController.reset(drive.getRotation().getRadians()))
                        .named("Reset angle controller");

        return drive.run(coroutine -> resetAngleController.andThen(joystickDrive))
                .named("Joystick Swerve at Angle");
    }

    /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Swerve drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Command.sequence(
        // Reset data
        Command.noRequirements(
            coroutine -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }).named("Swerve Characterization Clear Samples"),

        // Allow modules to orient
        drive.runRepeatedly(
                () -> {
                  drive.runCharacterization(0.0);
                }).named("Drive Start Delay")
            .withTimeout(Seconds.of(1)),

        // Start timer
        Command.noRequirements(coroutine -> timer.restart()).named("Restart Timer"),

        // Accelerate and gather data
        drive.runRepeatedly(
                () -> {
                  double voltage = timer.get() * 0.1;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                })

            // When cancelled, calculate and print results
            .whenCanceled(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }).named("Swerve Run Characterization")).named("Swerve FF Characterization Routine");
  }


    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Swerve drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(DriverConstants.WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        Command resetLimiter = drive.run(coroutine -> limiter.reset(0.0)).named("Reset limiter");

        Command accelerate =
                drive.run(
                                coroutine -> {
                                    double speed =
                                            limiter.calculate(
                                                    DriverConstants.WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisVelocities(0.0, 0.0, speed));
                                })
                        .named("Accelerate");

        Command recordStartingMeasurement =
                drive.run(
                                coroutine -> {
                                    state.positions =
                                            drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRotation();
                                    state.gyroDelta = 0.0;
                                })
                        .named("Record starting measurement");

        Command updateGyroDelta =
                drive.run(
                                coroutine -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta +=
                                            Math.abs(rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })
                        .named("Update gyro delta");

        Command calculate =
                drive.run(
                                coroutine -> {
                                    double[] positions =
                                            drive.getWheelRadiusCharacterizationPositions();
                                    double wheelDelta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheelDelta +=
                                                Math.abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheelRadius =
                                            (state.gyroDelta * Swerve.DRIVE_BASE_RADIUS)
                                                    / wheelDelta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println(
                                            "********** Wheel Radius Characterization Results **********");
                                    System.out.println(
                                            "\tWheel Delta: "
                                                    + formatter.format(wheelDelta)
                                                    + " radians");
                                    System.out.println(
                                            "\tGyro Delta: "
                                                    + formatter.format(state.gyroDelta)
                                                    + " radians");
                                    System.out.println(
                                            "\tWheel Radius: "
                                                    + formatter.format(wheelRadius)
                                                    + " meters, "
                                                    + formatter.format(
                                                            Units.metersToInches(wheelRadius))
                                                    + " inches");
                                })
                        .named("Calculate results");

        Command driveControl = resetLimiter.andThen(accelerate).named("Swerve control");
        Command measurement =
                Command.waitFor(Seconds.of(1.0))
                        .named("Wait 1 sec")
                        .andThen(recordStartingMeasurement)
                        .andThen(updateGyroDelta)
                        .named("Measurement");

        Command wheel = driveControl.alongWith(measurement).named("Control and measure");

        return wheel.andThen(calculate).named("Wheel radius characterization");
    }

    class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
