/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.opmodes;

import com.stuypulse.robot.Robot;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

@Autonomous(name = "Auton (Chooser)")
public class ChooserAuton implements OpMode {
    private final Robot robot;
    private Command auto;

    public ChooserAuton(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        auto = robot.getContainer().getAutonomousCommand();
        if (auto != null) {
            Scheduler.getDefault().schedule(auto);
        }
    }

    @Override
    public void end() {
        if (auto != null) {
            Scheduler.getDefault().cancel(auto);
        }
    }
}
