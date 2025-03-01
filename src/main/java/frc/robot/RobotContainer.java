// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.*;
import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.*;
import static frc.robot.subsystems.coralActuation.CoralActuationConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.*;
import static frc.robot.subsystems.algaeActuation.AlgaeActuationConstants.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.subsystems.led.LEDConstants.*;
import static frc.robot.commands.automation.PlaceCoral.*;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.*;
import static frc.robot.Controls.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.Reef;
import frc.robot.RobotState.actions;
import frc.robot.commands.automation.PlaceCoral;
import frc.robot.commands.automation.PrepareForPlaceCoral;
import frc.robot.commands.autos.Place4LeftSide;
import frc.robot.commands.autos.Place4RightSide;
import frc.robot.commands.drivetrain.AutoMoveToNearestPOI;
import frc.robot.subsystems.drive.controllers.AutoAlignController.allignmentMode;
import frc.robot.util.*;
import frc.robot.util.Alert.AlertType;

public class RobotContainer {
  private final RobotState robotState = RobotState.getInstance();
  private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);

  private static SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static boolean shouldUseZones = true;

  public double time = Double.NaN;

  public LoggedTunableNumber elevatorPos = new LoggedTunableNumber("Debug/elevatorPos", 10);
  public LoggedTunableNumber actuationPos = new LoggedTunableNumber("Debug/actuationPos", 10);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    FieldConstants.logFieldConstants();

    // Configure autos and buttons
    setupAutos();
    configureButtonBindings(false);
    // testLEDControls();

    // Alerts for constants
    if (Constants.tuningMode) {
      new Alert("Tuning mode enabled", AlertType.INFO).set(true);
    }
  }

  public void testLEDControls() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // driver.a().onTrue(leds.setAnimation(RAINBOW_ANIMATION));
    // driver.b().onTrue(leds.setAnimation(FIRE_ANIMATION));
    // driver.x().onTrue(leds.setAnimation(TWINKLE_ANIMATION));
    // driver.y().onTrue(leds.setAnimation(LARSON_ANIMATION));

    // driver.pov(0).onTrue(leds.setAnimation(RGB_FADE_ANIMATION));
    // driver.pov(90).onTrue(leds.setColor(255, 0, 0));
    // driver.pov(180).onTrue(leds.setColor(0, 255, 0));
    // driver.pov(270).onTrue(leds.setColor(0, 0, 255));

    // driver.rightBumper().onTrue(leds.setColor(teamDrivenYellow[0],
    // teamDrivenYellow[1], teamDrivenYellow[2], 10, 5));
    // driver.leftBumper().onTrue(leds.setColor(0, 0, 0));
  }

  private void setupAutos() {
    autoChooser.setDefaultOption("place 4 right side", new Place4RightSide().getAuto().cmd());
    autoChooser.addOption("place 4 left side", new Place4LeftSide().getAuto().cmd());

    SmartDashboard.putData(autoChooser);
  }

  private Command driveCommand = drive.run(
      () -> drive.acceptTeleopInput(
          driveX.getAsDouble(), driveY.getAsDouble(), driveOmega.getAsDouble(), false))
      .withName("Drive Teleop Input");

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick}
   * or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings(boolean demo) {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Drivetrain
    drive.setDefaultCommand(driveCommand);

    resetPose.onTrue(
        Commands.runOnce(
            () -> robotState.resetPose(
                new Pose2d(
                    robotState.getEstimatedPose().getTranslation(), AllianceFlipUtil.apply(new Rotation2d()))))
            .ignoringDisable(true));

    resetElevatorPosition.onTrue(elevator.resetPosition());

    // Placing
    placeL4.onTrue(Commands.runOnce(() -> RobotState.getInstance().desiredAction = actions.L4));
    placeL3.onTrue(Commands.runOnce(() -> RobotState.getInstance().desiredAction = actions.L3));
    placeL2.onTrue(Commands.runOnce(() -> RobotState.getInstance().desiredAction = actions.L2));
    placeL1.onTrue(Commands.runOnce(() -> RobotState.getInstance().desiredAction = actions.L1));

    // Auto align if trying to place L2+
    new Trigger(() -> RobotState.getInstance().desiredAction == actions.L4)
        .or(() -> RobotState.getInstance().desiredAction == actions.L3)
        .or(() -> RobotState.getInstance().desiredAction == actions.L2)
        .whileTrue(new AutoMoveToNearestPOI(false, allignmentMode.TWO_STAGE,
            Reef.placePoses));

    new Trigger(() -> RobotState.getInstance().desiredAction == actions.L4)
        .and(RobotState.getInstance()::isInReefZone)
        .onTrue(new PrepareForPlaceCoral(Constants.l4));

    new Trigger(() -> RobotState.getInstance().desiredAction == actions.L4)
        .and(() -> !driveCommand.isScheduled())
        .and(drive::isAutoAlignGoalCompleted)
        .onTrue(
            Commands.sequence(
                new PlaceCoral(Constants.l4),
                Commands.runOnce(AutoMoveToNearestPOI::stop),
                Commands.runOnce(() -> RobotState.getInstance().desiredAction = actions.NONE)));

    new Trigger(() -> RobotState.getInstance().desiredAction == actions.L3)
        .and(RobotState.getInstance()::isInReefZone)
        .onTrue(new PrepareForPlaceCoral(Constants.l3));

    new Trigger(() -> RobotState.getInstance().desiredAction == actions.L3)
        .and(() -> !driveCommand.isScheduled())
        .and(drive::isAutoAlignGoalCompleted)
        .onTrue(
            Commands.sequence(
                new PlaceCoral(Constants.l3),
                Commands.runOnce(AutoMoveToNearestPOI::stop),
                Commands.runOnce(() -> RobotState.getInstance().desiredAction = actions.NONE)));

    new Trigger(() -> RobotState.getInstance().desiredAction == actions.L2)
        .and(RobotState.getInstance()::isInReefZone)
        .onTrue(new PrepareForPlaceCoral(Constants.l2));

    new Trigger(() -> RobotState.getInstance().desiredAction == actions.L2)
        .and(() -> !driveCommand.isScheduled())
        .and(drive::isAutoAlignGoalCompleted)
        .onTrue(
            Commands.sequence(
                new PlaceCoral(Constants.l2),
                Commands.runOnce(AutoMoveToNearestPOI::stop),
                Commands.runOnce(() -> RobotState.getInstance().desiredAction = actions.NONE)));

    // intake
    // coralIntake.setDefaultCommand(coralIntake.run(() ->
    // coralIntake.runVelocity(-10)));
    coralIntakeIn.whileTrue(coralIntake.runVelocityCommand(intakeVelocity::get));
    coralOuttakeOut.whileTrue(coralIntake.runVelocityCommand(outtakeVelocity::get));
    driver.rightTrigger(0.1).whileTrue(Commands.startEnd(() -> coralIntake.runVoltage(tuningVoltage.get()),
        () -> coralIntake.runVoltage(0), coralIntake));

    // coralActuationUp.whileTrue(coralActuation.runVoltageCommand(() ->
    // coralActuationTuningVoltage.get()));
    // coralActuationDown.whileTrue(coralActuation.runVoltageCommand(() ->
    // -coralActuationTuningVoltage.get()));

    driver.x().onTrue(coralActuation.run(() -> coralActuation.setPos(actuationPos.get())));

    // coralActuationDown.onTrue(coralActuation.run(() ->
    // coralActuation.setPos(actuationPos.get())));

    // driver.x().onTrue(coralActuation.run(() -> coralActuation.setPos(-25)));
    // driver.b().onTrue(coralActuation.run(() -> coralActuation.setPos(25)));

    // Elevator
    elevatorUp.whileTrue(elevator.runVoltageCommand(() -> elevatorTuningVoltage.get()));
    elevatorDown.whileTrue(elevator.runVoltageCommand(() -> -elevatorTuningVoltage.get()));

    // elevatorUp.onTrue(elevator.run(() -> elevator.setPos(elevatorPos.get())));

    // elevatorDown.onTrue(elevator.run(() ->
    // elevator.setPos(this.elevatorPos.get())));

    // driver.a().onTrue(elevator.run(() -> elevator.setPos(20)));
    // driver.y().onTrue(elevator.run(() -> elevator.setPos(60)));

    // driver.a().whileTrue(
    //     new AutoMoveToNearestPOI(allignmentMode.TWO_STAGE, Reef.placePoses));

    // driver.b().whileTrue(
    // new AutoMoveToNearestPOI(allignmentMode.TWO_STAGE,
    // CoralStations.pickupLocations));

    // driver.b().onTrue(new PlaceCoral(Constants.l4));

    // climberUp.onTrue(climber.run(() -> climber.runVoltage(12))).onFalse(climber.run(() -> climber.runVoltage(0)));
    // climberDown.onTrue(climber.run(() -> climber.runVoltage(-12))).onFalse(climber.run(() -> climber.runVoltage(0)));
  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
  }

  /** Updates dashboard data. */
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Drive Static
    // Characterization
    // return new StaticCharacterization(
    // drive, drive::runCharacterization, drive::getCharacterizationVelocity)
    // .finallyDo(drive::endCharacterization);

    // Drive FF Characterization
    // return new FeedForwardCharacterization(
    // drive, drive::runCharacterization, drive::getCharacterizationVelocity)
    // .finallyDo(drive::endCharacterization);

    // Drive Wheel Radius Characterization
    // return drive
    // .orientModules(Drive.getCircleOrientations())
    // .andThen(
    // new WheelRadiusCharacterization(
    // drive, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE))
    // .withName("Drive Wheel Radius Characterization");

    // Slippage Calculator
    // return Commands.runOnce(
    // () ->
    // robotState.resetPose(
    // new Pose2d(
    // // robotState.getEstimatedPose().getTranslation(),
    // new Translation2d(), AllianceFlipUtil.apply(new Rotation2d()))))
    // .andThen(new SlippageCalculator(drive))
    // .withName("Slippage Calculator");

    return autoChooser.getSelected();
  }
}
