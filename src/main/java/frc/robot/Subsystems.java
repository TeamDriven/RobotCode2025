// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.coralActuation.CoralActuation;
import frc.robot.subsystems.coralActuation.CoralActuationIO;
import frc.robot.subsystems.coralActuation.CoralActuationIOKraken;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeIO;
import frc.robot.subsystems.coralIntake.CoralIntakeIOKraken;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOKrakenFOC;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

/**
 * The Subsystems class represents the collection of subsystems used in the robot. It provides
 * static references to various subsystem objects that are used in the robot.
 */
public final class Subsystems {
  public static final Drive drive;
  public static final Vision visionOne;

  public static final CoralIntake coralIntake;
  public static final CoralActuation coralActuation;

  static {
    // Create subsystems
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(true),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[0]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[1]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[2]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[3]));
          coralIntake = new CoralIntake(new CoralIntakeIOKraken(13));
          coralActuation = new CoralActuation(new CoralActuationIOKraken(14));
          visionOne = new Vision("VisionOne", new VisionIOLimelight("limelight"), drive::getSpeeds);
        }
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(false),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[0]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[1]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[2]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[3]));
          coralIntake = new CoralIntake(new CoralIntakeIOKraken(13));
          coralActuation = new CoralActuation(new CoralActuationIOKraken(14));
          visionOne = new Vision("VisionOne", new VisionIOLimelight("limelight"), drive::getSpeeds);
        }
        case SIMBOT -> {
          throw new IllegalStateException("SIMBOT is not currently implemented on this robot");
        }
        default -> {
          throw new IllegalStateException("Robot type not selected");
        }
      }
    } else {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
      coralIntake = new CoralIntake(new CoralIntakeIO() {});
      coralActuation = new CoralActuation(new CoralActuationIO() {});
      visionOne = new Vision("VisionOne", new VisionIO() {}, drive::getSpeeds);
    }
  }
}
