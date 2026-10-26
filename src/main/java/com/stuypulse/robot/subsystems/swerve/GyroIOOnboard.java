/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.swerve;

import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Rotation2d;

import java.util.Queue;

public class GyroIOOnboard implements GyroIO {
    private final OnboardIMU imu;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOOnboard() {
        imu = new OnboardIMU(SwerveConstants.Gyro.SYSCORE_MOUNT_ORIENTATION);

        imu.resetYaw();

        yawPositionQueue =
                PhoenixOdometryThread.getInstance()
                        .registerSignal(() -> imu.getRotation2d().getDegrees());
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // The imu is directly on the systemcore, so if the imu fails then the whole systemcore is
        // likely failing
        inputs.connected = true;
        inputs.yawPosition = imu.getRotation2d();
        inputs.yawVelocityRadPerSec = imu.getGyroRateZ();

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
                yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
