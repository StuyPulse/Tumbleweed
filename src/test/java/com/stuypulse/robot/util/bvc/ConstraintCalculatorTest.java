/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.util.bvc;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.optimization.InequalityConstraints;
import org.wpilib.math.optimization.Problem;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

@ExtendWith(TimingExtension.class)
public class ConstraintCalculatorTest {
    private static final Translation2d current = new Translation2d(1.0, 1.0);
    private static final Translation2d neighbor = new Translation2d(2.0, 2.0);
    private static final Translation2d testPoint = new Translation2d(1.5, 1.5);

    @Test
    public void constraintEqualityTest() {
        assertDoesNotThrow(
                () -> {
                    var constraintRegular = ConstraintCalculator.getConstraint(current, neighbor);
                    double resultRegular = constraintRegular.applyAsDouble(testPoint.toVector());
                    var constraintSIMD = ConstraintCalculator.getConstraintSIMD(current, neighbor);
                    double resultSIMD = constraintSIMD.applyAsDouble(testPoint.toVector());
                    assertEquals(resultRegular, resultSIMD);
                });
    }

    @Test
    public void constraintRegularTimingTest() {
        for (int i = 0; i < 100; i++) {
            var constraint = ConstraintCalculator.getConstraint(current, neighbor);
            constraint.applyAsDouble(testPoint.toVector());
        }
    }

    @Test
    public void constraintSIMDTimingTest() {
        for (int i = 0; i < 100; i++) {
            var constraint = ConstraintCalculator.getConstraintSIMD(current, neighbor);
            constraint.applyAsDouble(testPoint.toVector());
        }
    }

    @Test
    public void constraintSleipnirConvergeTest() {
        double targetX = 3.0;
        double targetY = 2.0;

        double lastX = current.getX();
        double lastY = current.getY();

        for (int i = 0; i < 10; i++) {
            Problem problem = new Problem();
            VariableMatrix targetVariable = problem.decisionVariable(2, 1);

            InequalityConstraints constraintSleipnir =
                    ConstraintCalculator.getConstraintSleipnir(
                            targetVariable, new Translation2d(lastX, lastY), neighbor);
            problem.subjectTo(constraintSleipnir);

            Variable dx = targetVariable.get(0, 0).minus(targetX);
            Variable dy = targetVariable.get(1, 0).minus(targetY);
            Variable cost = dx.times(dx).plus(dy.times(dy));

            problem.minimize(cost);
            problem.solve();

            lastX = targetVariable.get(0, 0).value();
            lastY = targetVariable.get(1, 0).value();
            IO.println("X: " + lastX + ", Y: " + lastY);

            problem.close();
        }
    }

    @Test
    public void constraintSleipnirTest() {
        Problem problem = new Problem();
        VariableMatrix targetVariable = problem.decisionVariable(2, 1);

        InequalityConstraints constraintSleipnir =
                ConstraintCalculator.getConstraintSleipnir(targetVariable, current, neighbor);
        problem.subjectTo(constraintSleipnir);

        double targetX = 3.0;
        double targetY = 3.0;
        Variable dx = targetVariable.get(0, 0).minus(targetX);
        Variable dy = targetVariable.get(1, 0).minus(targetY);
        Variable cost = dx.times(dx).plus(dy.times(dy));

        problem.minimize(cost);
        problem.solve();

        IO.println(
                "X: "
                        + targetVariable.get(0, 0).value()
                        + ", Y: "
                        + targetVariable.get(1, 0).value());
        problem.close();
    }
}
