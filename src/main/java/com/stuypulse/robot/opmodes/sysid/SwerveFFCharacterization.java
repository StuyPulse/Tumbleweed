package com.stuypulse.robot.opmodes.sysid;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.swerve.Swerve;

import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Utility;

@Utility(
   name = "Swerve FF Characterization",
   group = "SysId",
   description = "Swerve FeedForward characterization routine made by 6328"
)
public class SwerveFFCharacterization implements OpMode {
   private final Swerve swerve;
   
   public SwerveFFCharacterization(Robot robot) {
        swerve = robot.getContainer().getSwerve();
   }
}
