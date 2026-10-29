/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.opmodes.sysid;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.util.SysIdRoutine;
import com.stuypulse.robot.util.SysIdRoutine.Direction;

import org.wpilib.opmode.Utility;

@Utility(name = "Dynamic Reverse", group = "SysId")
public class SwerveDynamicReverse extends SwerveDriveSysId {
    public SwerveDynamicReverse(Robot robot) {
        super(robot.getContainer().getSwerve(), Direction.REVERSE, SysIdRoutine::dynamic);
    }
}
