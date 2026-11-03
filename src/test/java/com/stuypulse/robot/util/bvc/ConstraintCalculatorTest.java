/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.util.bvc;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.wpilib.math.geometry.Translation2d;

import org.junit.jupiter.api.Test;

public class ConstraintCalculatorTest {
    private static final Translation2d current = new Translation2d(1.0, 1.0);
    private static final Translation2d neighbor = new Translation2d(2.0, 2.0);
    private static final Translation2d testPoint = new Translation2d(1.5, 1.5);

    @Test
    public void constraintTest() {
        assertDoesNotThrow(
                () -> {
                    var constraintRegular = ConstraintCalculator.getConstraint(current, neighbor);
                    double resultRegular = constraintRegular.applyAsDouble(testPoint.toVector());
                    var constraintSIMD = ConstraintCalculator.getConstraintSIMD(current, neighbor);
                    double resultSIMD = constraintSIMD.applyAsDouble(testPoint.toVector());
                    assertEquals(resultRegular, resultSIMD);
                });
    }
}
