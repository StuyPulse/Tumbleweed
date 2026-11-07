/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.util.bvc;

import org.wpilib.annotation.NoDiscard;
import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.optimization.Constraints;
import org.wpilib.math.optimization.InequalityConstraints;

import java.util.function.ToDoubleFunction;
import jdk.incubator.vector.DoubleVector;
import jdk.incubator.vector.VectorOperators;

public final class ConstraintCalculator {
    private ConstraintCalculator() {}

    private static final double SAFETY_RADIUS_METRES =
            1.0; // could use units system from wpilib but no conversions are needed anywhere

    /**
     * Converts a WPILib vector to a JEP 529 vector
     *
     * @param vector Two row WPILib vector
     * @return DoubleVector with enough space for two doubles (128 bits)
     */
    @NoDiscard
    private static DoubleVector toDoubleVector(Vector<N2> vector) {
        return DoubleVector.fromArray(DoubleVector.SPECIES_128, vector.getData(), 0);
    }

    /**
     * Gets a constraint for a given neighbor
     *
     * <p>Backed by SIMD, but it is suboptimal due to its large overhead for the small amount of
     * math performed on the vector registers
     *
     * @param currentPose
     * @param neighborPose
     * @return scalar supplier that is ≤ 0 when input is fine
     * @apiNote this method is meant to implement B-UAVC, which would justify the overhead
     */
    @Deprecated(forRemoval = false)
    public static ToDoubleFunction<Vector<N2>> getConstraintSIMD(
            Translation2d currentPose, Translation2d neighborPose) {
        DoubleVector self = toDoubleVector(currentPose.toVector());
        DoubleVector neighbor = toDoubleVector(neighborPose.toVector());
        DoubleVector displacement = neighbor.sub(self);
        DoubleVector midpoint = self.add(neighbor).div(2);
        double buffer =
                SAFETY_RADIUS_METRES
                        * Math.sqrt(
                                displacement.mul(displacement).reduceLanes(VectorOperators.ADD));
        return p ->
                toDoubleVector(p).sub(midpoint).mul(displacement).reduceLanes(VectorOperators.ADD)
                        + buffer;
    }

    /**
     * gets a constraint for a given neighbor
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
        double buffer = SAFETY_RADIUS_METRES * displacement.norm();
        return p ->
                p.transpose().minus(midpoint).times(displacement).get(0, 0)
                        + buffer; // worst math oat
    }

    /**
     * Gets a constraint for a given neighbor
     *
     * <p>backed by sleipnir for solving purposes
     *
     * @param target pose of the target location
     * @param currentPose pose of the ego-robot (self)
     * @param neighborPose pose of the neighboring robot
     * @return solveable sleipnir constraint
     */
    public static InequalityConstraints getConstraintSleipnir(
            VariableMatrix target, Translation2d currentPose, Translation2d neighborPose) {
        double selfX = currentPose.getX();
        double selfY = currentPose.getY();
        double neighborX = neighborPose.getX();
        double neighborY = neighborPose.getY();

        double dispX = neighborX - selfX;
        double dispY = neighborY - selfY;

        double midX = (selfX + neighborX) / 2.0;
        double midY = (selfY + neighborY) / 2.0;

        double buffer = SAFETY_RADIUS_METRES * Math.sqrt((dispX * dispX) + (dispY * dispY));
        Variable dotX = target.get(0, 0).minus(midX).times(dispX); // dot product
        Variable dotY = target.get(1, 0).minus(midY).times(dispY);
        Variable lhs = dotX.plus(dotY).plus(buffer);
        return Constraints.le(lhs, 0);
    }
}
