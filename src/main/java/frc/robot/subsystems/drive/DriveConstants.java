// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.swerve.ModuleLimits;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
  public static final boolean shouldPrintZeros = false;

  public static final DriveConfig driveConfig = switch (Constants.getRobot()) {
    case SIMBOT, COMPBOT -> new DriveConfig(
        Units.inchesToMeters(1.9579309048831377), // Get from Wheel Radius Characterization
        Units.inchesToMeters(22.75), // Track width X
        Units.inchesToMeters(22.75), // Track width Y
        Units.inchesToMeters(28 + (3.5*2)), // Bumper width X
        Units.inchesToMeters(28 + (3.5*2)), // Bumper width Y
        4.804, // Max Linear Velocity
        Units.feetToMeters(75.0), // Max Linear Acceleration
        12.0, // Max Angular Velocity
        6.0); //6 // Max Angular Acceleration
    case DEVBOT -> new DriveConfig(
        Units.inchesToMeters(1.924437419735719), // Get from Wheel Radius Characterization
        Units.inchesToMeters(22.75), // Track width X
        Units.inchesToMeters(22.75), // Track width Y
        Units.inchesToMeters(34), // Bumper width X
        Units.inchesToMeters(34), // Bumper width Y
        5.641, // Max Linear Velocity
        Units.feetToMeters(75.0), // Max Linear Acceleration
        12.0, // Max Angular Velocity
        6.0); // Max Angular Acceleration
  };
  public static final Translation2d[] moduleTranslations = new Translation2d[] {
      new Translation2d(driveConfig.trackWidthX() / 2.0, driveConfig.trackWidthY() / 2.0),
      new Translation2d(driveConfig.trackWidthX() / 2.0, -driveConfig.trackWidthY() / 2.0),
      new Translation2d(-driveConfig.trackWidthX() / 2.0, driveConfig.trackWidthY() / 2.0),
      new Translation2d(-driveConfig.trackWidthX() / 2.0, -driveConfig.trackWidthY() / 2.0)
  };
  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

  // Odometry Constants
  public static final double odometryFrequency = switch (Constants.getRobot()) {
    case SIMBOT -> 50.0;
    case DEVBOT -> 100.0;
    case COMPBOT -> 250.0;
  };

  public static final Matrix<N3, N1> odometryStateStdDevs = switch (Constants.getRobot()) {
    default -> new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
  };

  // Module Constants
  public static final ModuleConfig[] moduleConfigs = switch (Constants.getRobot()) {
    case COMPBOT -> new ModuleConfig[] {
        new ModuleConfig(1, 2, 3, new Rotation2d(-1.205709), true),
        new ModuleConfig(4, 5, 6, new Rotation2d(-1.940486), true),
        new ModuleConfig(7, 8, 9, new Rotation2d(-0.119651), true),
        new ModuleConfig(10, 11, 12, new Rotation2d(-1.552389), true)
    };
    case DEVBOT -> new ModuleConfig[] {
        new ModuleConfig(1, 2, 3, new Rotation2d(0.038350), true),
        new ModuleConfig(4, 5, 6, new Rotation2d(1.575398), true),
        new ModuleConfig(7, 8, 9, new Rotation2d(0.013806), true),
        new ModuleConfig(10, 11, 12, new Rotation2d(-1.547787), true)
    };
    case SIMBOT -> {
      ModuleConfig[] configs = new ModuleConfig[4];
      for (int i = 0; i < configs.length; i++)
        configs[i] = new ModuleConfig(0, 0, 0, new Rotation2d(0), false);
      yield configs;
    }
  };

  public static final ModuleConstants moduleConstants = switch (Constants.getRobot()) {
    case COMPBOT -> new ModuleConstants(
        5.86908, // Get these two from FeedForwardCharacterization
        0.09108,
        1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp, // A/(N*m)
        35.0,
        0.0,
        4000.0,
        50.0,
        Mk4iReductions.L3_16T.reduction, // L3 16 tooth
        Mk4iReductions.TURN.reduction);
    case DEVBOT -> new ModuleConstants(
        5.07453,
        0.05741,
        1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp,
        35.0,
        0.0,
        3000.0,
        50.0,
        Mk4iReductions.L3_16T.reduction,
        Mk4iReductions.TURN.reduction);
    case SIMBOT -> new ModuleConstants(
        0.014,
        0.134,
        0.0,
        0.1,
        0.0,
        10.0,
        0.0,
        Mk4iReductions.L3_16T.reduction,
        Mk4iReductions.TURN.reduction);
  };

  public static final ModuleLimits moduleLimitsFree = new ModuleLimits(
      driveConfig.maxLinearVelocity(),
      driveConfig.maxLinearAcceleration(),
      Units.degreesToRadians(1080.0));

  public static final ModuleLimits moduleLimitsFlywheelSpinup = new ModuleLimits(
      driveConfig.maxLinearVelocity(),
      driveConfig.maxLinearAcceleration() / 2.0,
      Units.degreesToRadians(1080.0));

  // Swerve Heading Control
  public static final HeadingControllerConstants headingControllerConstants = switch (Constants.getRobot()) {
    default -> new HeadingControllerConstants(4.1, 0.1, 8.0, 20.0);
  };

  public record DriveConfig(
      double wheelRadius,
      double trackWidthX,
      double trackWidthY,
      double bumperWidthX,
      double bumperWidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    }
  }

  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {
  }

  public record ModuleConstants(
      double ffkS,
      double ffkV,
      double ffkT,
      double drivekP,
      double drivekD,
      double turnkP,
      double turnkD,
      double driveReduction,
      double turnReduction) {
  }

  public record AutoAlignConstants(
      double linearkP,
      double linearkD,
      double thetakP,
      double thetakD,
      double linearTolerance,
      double thetaTolerance,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
  }

  public record HeadingControllerConstants(
      double kP, double kD, double maxVelocity, double maxAcceleration) {
  }

  private enum Mk4iReductions {
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0)),
    L3_16T(5.357142857142857);

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
