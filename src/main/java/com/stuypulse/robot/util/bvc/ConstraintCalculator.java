/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.util.bvc;

import org.wpilib.annotation.NoDiscard;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;

import java.util.function.ToDoubleFunction;
import jdk.incubator.vector.DoubleVector;
import jdk.incubator.vector.VectorOperators;

public final class ConstraintCalculator {

    private ConstraintCalculator() {}

    private static final double SAFETY_RADIUS = 1.0; // idk what unit

    @NoDiscard
    public static DoubleVector vectorToVector(Vector<N2> vector) {
        return DoubleVector.fromArray(DoubleVector.SPECIES_128, vector.getData(), 0);
    }

    /**
     * polyline function for quadratic optimization
     *
     * @param currentPose
     * @param neighborPose
     * @return scalar supplier that is ≤ 0 when input is fine
     */
    public static ToDoubleFunction<Vector<N2>> getConstraintSIMD(
            Translation2d currentPose, Translation2d neighborPose) {
        DoubleVector self = vectorToVector(currentPose.toVector());
        DoubleVector neighbor = vectorToVector(neighborPose.toVector());
        DoubleVector displacement = neighbor.sub(self);
        DoubleVector midpoint = self.add(neighbor).div(2);
        double buffer =
                SAFETY_RADIUS
                        * Math.sqrt(
                                displacement.mul(displacement).reduceLanes(VectorOperators.ADD));
        return p ->
                vectorToVector(p).sub(midpoint).mul(displacement).reduceLanes(VectorOperators.ADD)
                        + buffer;
    }

    /**
     * gets a constraint for an invalid case of the bvc (vision integration coming soon)
     *
     * @param swerve swerve subsystem
     */
    public static ToDoubleFunction<Vector<N2>> getConstraint(
            Translation2d currentPose, Translation2d neighborPose) { // TODO: vector api
        Vector<N2> self = currentPose.toVector(); // vector is just a 2x1 matrix (rows x columns)
        Vector<N2> neighbor =
                neighborPose
                        .toVector(); // placeholder, should be sourced from vision in the future,
        // maybe pose supplier?
        Vector<N2> displacement = neighbor.minus(self);
        Matrix<N1, N2> midpoint = self.transpose().plus(neighbor.transpose()).div(2);
        double buffer = SAFETY_RADIUS * displacement.norm();
        return p ->
                p.transpose().minus(midpoint).times(displacement).get(0, 0)
                        + buffer; // worst math oat
    }
}
