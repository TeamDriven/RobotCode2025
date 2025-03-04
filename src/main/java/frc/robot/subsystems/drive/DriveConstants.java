// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.util.swerve.ModuleLimits;

public class DriveConstants {
    public static final boolean shouldPrintZeros = false;

    // Odometry Constants
    public static final double odometryFrequency = switch (Constants.getRobot()) {
        case SIMBOT -> 50.0;
        case DEVBOT -> 100.0;
        case COMPBOT -> 150.0;
    };
    public static final double trackWidthX = Constants.getRobot() == RobotType.DEVBOT
            ? Units.inchesToMeters(22.75)
            : Units.inchesToMeters(22.75);
    public static final double trackWidthY = Constants.getRobot() == RobotType.DEVBOT
            ? Units.inchesToMeters(22.75)
            : Units.inchesToMeters(22.75);
    public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
    public static final double maxLinearSpeed = 4.766;
    public static final double maxAngularSpeed = maxLinearSpeed / driveBaseRadius;

    /** Includes bumpers! */
    public static final double robotWidth = Units.inchesToMeters(28.0) + 2 * Units.inchesToMeters(3.0);

    public static final Translation2d[] moduleTranslations = {
            new Translation2d(trackWidthX / 2, trackWidthY / 2),
            new Translation2d(trackWidthX / 2, -trackWidthY / 2),
            new Translation2d(-trackWidthX / 2, trackWidthY / 2),
            new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
    };

    public static final double wheelRadius = Units.inchesToMeters(1.924437419735719);

    public static final ModuleLimits moduleLimitsFree = new ModuleLimits(maxLinearSpeed, maxAngularSpeed,
            Units.degreesToRadians(1080.0));

    public static final double driveGearReduction = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public static final double turnGearReduction = 150.0 / 7.0;

    public static final ModuleConfig[] moduleConfigs = switch (Constants.getRobot()) {
        case COMPBOT -> new ModuleConfig[] {
                new ModuleConfig(1, 2, 3, new Rotation2d(0.015340), true, false),
                new ModuleConfig(4, 5, 6, new Rotation2d(1.544719), true, false),
                new ModuleConfig(7, 8, 9, new Rotation2d(1.049243), true, false),
                new ModuleConfig(10, 11, 12, new Rotation2d(-1.589204), true, false)
        };
        case DEVBOT -> new ModuleConfig[] {
                new ModuleConfig(1, 2, 3, new Rotation2d(0.038350), true, false),
                new ModuleConfig(4, 5, 6, new Rotation2d(1.575398), true, false),
                new ModuleConfig(7, 8, 9, new Rotation2d(0.013806), true, false),
                new ModuleConfig(10, 11, 12, new Rotation2d(-1.547787), true, false)
        };
        case SIMBOT -> {
            ModuleConfig[] configs = new ModuleConfig[4];
            for (int i = 0; i < configs.length; i++)
                configs[i] = new ModuleConfig(0, 0, 0, new Rotation2d(0), false, false);
            yield configs;
        }
    };

    public record ModuleConfig(
            int driveMotorId,
            int turnMotorId,
            int encoderChannel,
            Rotation2d encoderOffset,
            boolean turnInverted,
            boolean encoderInverted) {
    }
}