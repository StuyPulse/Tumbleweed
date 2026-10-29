/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.opmodes.sysid;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.util.SysIdRoutine;
import com.stuypulse.robot.util.SysIdRoutine.Direction;

import org.wpilib.opmode.Utility;

@Utility(name = "Quasistatic Forward", group = "SysId")
public class SwerveQuasistaticForward extends SwerveDriveSysId {
    public SwerveQuasistaticForward(Robot robot) {
        super(robot.getContainer().getSwerve(), Direction.FORWARD, SysIdRoutine::quasistatic);
    }
}
