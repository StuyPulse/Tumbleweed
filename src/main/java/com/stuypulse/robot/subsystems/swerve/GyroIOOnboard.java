/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.swerve;

import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.geometry.Rotation2d;

import java.util.Queue;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class GyroIOOnboard implements GyroIO {
    private final OnboardIMU imu;
    private final Supplier<Rotation2d> yawSignal;
    private final DoubleSupplier yawVelocitySignal;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOOnboard() {
        imu = new OnboardIMU(MountOrientation.FLAT);

        imu.resetYaw();

        yawSignal = imu::getRotation2d;
        yawVelocitySignal = imu::getGyroRateZ;
        yawPositionQueue =
                PhoenixOdometryThread.getInstance()
                        .registerSignal(() -> yawSignal.get().getDegrees());
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawPosition = yawSignal.get();
        inputs.yawVelocityRadPerSec = yawVelocitySignal.getAsDouble();

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
                yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
