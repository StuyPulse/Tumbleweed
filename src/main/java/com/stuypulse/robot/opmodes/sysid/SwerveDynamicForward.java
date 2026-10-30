/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.opmodes.sysid;

import com.stuypulse.robot.Robot;

import org.wpilib.command3.sysid.SysIdRoutine;
import org.wpilib.command3.sysid.SysIdRoutine.Direction;
import org.wpilib.opmode.Utility;

@Utility(name = "Dynamic Forward", group = "SysId")
public class SwerveDynamicForward extends SwerveDriveSysId {
    public SwerveDynamicForward(Robot robot) {
        super(robot.getContainer().getSwerve(), Direction.FORWARD, SysIdRoutine::dynamic);
    }
}
