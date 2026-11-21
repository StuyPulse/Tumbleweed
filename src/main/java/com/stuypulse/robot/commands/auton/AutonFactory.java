/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.subsystems.vision.Vision;

/** Class that stores all auton commands */
public class AutonFactory {
    /** Add robot subsystems into this constructor for easy access when making autons */
    public AutonFactory(Swerve swerve, Vision vision) {}

    public Auton doNothingAuton() {
        return new Auton() {};
    }
}
