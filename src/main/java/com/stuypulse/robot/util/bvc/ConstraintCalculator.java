package com.stuypulse.robot.util.bvc;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;

import com.stuypulse.robot.subsystems.swerve.Swerve;

public final class ConstraintCalculator {
    
    private ConstraintCalculator() {}

    public static double norm(double x, double y) {
        return 0.0;
    }


    private static final double SAFETY_RADIUS = 1.0; // idk what unit
    /**
     * gets a constraint for an invalid case of the bvc (vision integration coming soon)
     * @param swerve swerve subsystem
     */
    public static Function<Vector<N2>, Double> getConstraint(Swerve swerve, Pose2d neighborPose) { // TODO: vector api
        Vector<N2> self = swerve.getPose().getTranslation().toVector(); // vector is just a 2x1 matrix (rows x columns)
        Vector<N2> neighbor = neighborPose.getTranslation().toVector(); // placeholder, should be sourced from vision in the future, maybe pose supplier?
        Vector<N2> displacement = neighbor.minus(self);
        Matrix<N1, N2> midpoint = self.transpose().plus(neighbor.transpose()).div(2);
        double buffer = SAFETY_RADIUS * displacement.norm();
        return p -> p.transpose().minus(midpoint).times(displacement).get(0,0) + buffer; // worst math oat
    }
}
