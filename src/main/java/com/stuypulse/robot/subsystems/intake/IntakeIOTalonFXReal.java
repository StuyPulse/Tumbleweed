/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.GlobalPorts;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.IntakePorts;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFXReal extends IntakeIOTalonFXBase {
    public IntakeIOTalonFXReal() {
        super(new TalonFX(IntakePorts.ROLLER_MOTOR_PORT, GlobalPorts.CAN_S1));
    }
}
