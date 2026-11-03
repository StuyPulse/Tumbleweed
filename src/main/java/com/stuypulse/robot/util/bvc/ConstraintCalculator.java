package com.stuypulse.robot.util.bvc;

import java.util.function.Consumer;

import org.wpilib.math.linalg.Matrix;
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
    private static void getBadConstraint(Swerve swerve) { // TODO: vector api
        Vector<N2> self = swerve.getPose().getTranslation().toVector(); // vector is just a 2x1 matrix (rows x columns)
        Vector<N2> neighbor = swerve.getPose().getTranslation().toVector(); // placeholder, should be sourced from vision in the future, maybe pose supplier?
        Consumer<Vector<N2>> constraint = p -> {
            Matrix<N1, N2> midpoint = self.transpose().plus(neighbor.transpose()).div(2);
            double unbuffered = p.transpose().minus(p.transpose()).times(neighbor.minus(self)).get(0,0); // extract the scalar from 1x1 matrix
            double buffer = SAFETY_RADIUS * neighbor.minus(self).norm();
            // <= 0???? what!!??!!
        };
    }
}
