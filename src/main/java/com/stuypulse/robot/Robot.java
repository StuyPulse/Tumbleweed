/********************* PROJECT TUMBLEWEED *********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.stuypulse.robot.constants.GlobalSettings;

import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.Filesystem;
import org.wpilib.util.Alert;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Robot Class
 *
 * <p>
 * This is the main class for robot code, instantiated in
 * {@link com.stuypulse.robot.Main} It
 * extends TimedRobot, meaning that the methods in this class are called
 * automatically during
 * specific states of the robot.
 */
public class Robot extends OpModeRobot {
    private static final Alert allianceDisconnectedAlert = new Alert(
            "AllianceDisconnected",
            "alliance cannot be determined by match state, check FMS",
            Alert.Level.MEDIUM);

    /**
     * Checks the alliance the robot is on
     *
     * @return true if the robot is on the blue alliance, false if the robot is on
     *         the red alliance,
     *         and false if alliance cannot be determined.
     */
    public static boolean isBlue() {
        final Optional<Alliance> alliance = MatchState.getAlliance();
        if (alliance.isPresent()) {
            allianceDisconnectedAlert.set(false);
            return alliance.get() == Alliance.BLUE;
        } else {
            allianceDisconnectedAlert.set(true);
            return false;
        }
    }

    private final RobotContainer robot;
    private final Scheduler defaultScheduler;

    public Robot() {
        robot = new RobotContainer();
        defaultScheduler = Scheduler.getDefault();

        Logger.AdvancedHooks.disableRobotBaseCheck();

        switch (GlobalSettings.CURRENT_MODE) {
            case REAL -> {
                Logger.addDataReceiver(new WPILOGWriter("/home/systemcore/logs"));
                Logger.addDataReceiver(new NT4Publisher());
            }

            case SIM -> {
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }

            case REPLAY -> {
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
            }
        }

        logCommitHash();
        Logger.start();

        AutoLogOutputManager.addObject(this);
    }

    private void logCommitHash() {
        Path commitFile = Filesystem.getDeployDirectory().toPath().resolve("commit.json");

        try {
            if (Files.exists(commitFile)) {
                ObjectMapper json = new ObjectMapper();
                JsonNode object = json.readTree(commitFile.toFile());

                String hash = object.path("hash").asText("unknown");
                String message = object.path("message").asText("unknown");
                String description = object.path("description").asText("");

                Logger.recordMetadata("Commit Hash", hash);
                Logger.recordMetadata("Commit Message", message);
                if (!description.isBlank()) {
                    Logger.recordMetadata("Commit Description", description);
                }
            } else {
                Logger.recordMetadata("Commit", "commit.json file missing");
            }
        } catch (Exception e) {
            Logger.recordMetadata("Commit", "JSON parse failed: " + e);
        }
    }

    public RobotContainer getContainer() {
        return robot;
    }

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotPeriodic() {
        Logger.AdvancedHooks.invokePeriodicBeforeUser();
        defaultScheduler.run();
        Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
    }
}
