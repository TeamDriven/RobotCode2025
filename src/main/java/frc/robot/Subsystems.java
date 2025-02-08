// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.algaeActuation.AlgaeActuation;
import frc.robot.subsystems.algaeActuation.AlgaeActuationIO;
import frc.robot.subsystems.algaeActuation.AlgaeActuationIOKraken;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIO;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIOKraken;
import frc.robot.subsystems.coralActuation.CoralActuation;
import frc.robot.subsystems.coralActuation.CoralActuationIO;
import frc.robot.subsystems.coralActuation.CoralActuationIOKraken;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeIO;
import frc.robot.subsystems.coralIntake.CoralIntakeIOKraken;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOKrakenFOC;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDIOCANdle;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

/**
 * The Subsystems class represents the collection of subsystems used in the robot. It provides
 * static references to various subsystem objects that are used in the robot.
 */
public final class Subsystems {
  public static final Drive drive;

  public static final Vision leftVision;
  public static final Vision rightVision;

  // public static final CoralIntake coralIntake;
  // public static final CoralActuation coralActuation;
  // public static final AlgaeActuation algaeActuation;
  // public static final AlgaeIntake algaeIntake;
  // public static final Elevator elevator;
  // public static final Climber climber;

  // public static final LED leds;

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

          leftVision = new Vision("Left Vision", new VisionIOLimelight("limelight-left"), drive::getSpeeds);
          rightVision = new Vision("Right Vision", new VisionIOLimelight("limelight-right"), drive::getSpeeds);

          // coralIntake = new CoralIntake(new CoralIntakeIOKraken(13));
          // coralActuation = new CoralActuation(new CoralActuationIOKraken(14));
          // algaeActuation = new AlgaeActuation(new AlgaeActuationIOKraken(17));
          // algaeIntake = new AlgaeIntake(new AlgaeIntakeIOKraken(18));
          // elevator = new Elevator(new ElevatorIOKraken(15, 16));
          // climber = new Climber(new ClimberIOKraken(19, 20));

          // leds = new LED(new LEDIOCANdle(60));
        }
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(false),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[0]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[1]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[2]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[3]));

          leftVision = new Vision("Left Vision", new VisionIOLimelight("limelight-left"), drive::getSpeeds);
          rightVision = new Vision("Right Vision", new VisionIOLimelight("limelight-right"), drive::getSpeeds);

          // coralIntake = new CoralIntake(new CoralIntakeIOKraken(13));
          // coralActuation = new CoralActuation(new CoralActuationIOKraken(14));
          // algaeActuation = new AlgaeActuation(new AlgaeActuationIOKraken(17));
          // algaeIntake = new AlgaeIntake(new AlgaeIntakeIOKraken(18));
          // elevator = new Elevator(new ElevatorIOKraken(15, 16));
          // climber = new Climber(new ClimberIOKraken(19, 20));

          // leds = new LED(new LEDIOCANdle(60));
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

      leftVision = new Vision("Left Vision", new VisionIO() {}, drive::getSpeeds);
      rightVision = new Vision("Right Vision", new VisionIO() {}, drive::getSpeeds);

      // coralIntake = new CoralIntake(new CoralIntakeIO() {});
      // coralActuation = new CoralActuation(new CoralActuationIO() {});
      // algaeActuation = new AlgaeActuation(new AlgaeActuationIO() {});
      // algaeIntake = new AlgaeIntake(new AlgaeIntakeIO() {});
      // elevator = new Elevator(new ElevatorIO() {});
      // climber = new Climber(new ClimberIO() {});

      // leds = new LED(new LEDIO() {});
    }
  }
}
