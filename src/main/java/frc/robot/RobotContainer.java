// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.*;
import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.*;
import static frc.robot.Controls.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.CoralStations;
import frc.robot.FieldConstants.Reef;
import frc.robot.RobotState.actions;
import frc.robot.commands.automation.PlaceCoral;
import frc.robot.commands.automation.SetPosition;
import frc.robot.commands.automation.TuckCommand;
import frc.robot.commands.autos.Place4LeftSide;
import frc.robot.commands.autos.Place4RightSide;
import frc.robot.commands.drivetrain.AutoMoveToNearestPOI;
import frc.robot.subsystems.coralActuation.CoralActuationConstants;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.drive.controllers.AutoAlignController.allignmentMode;
import frc.robot.subsystems.elevator.ElevatorConstants;
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

  private Command setDesiredAction(actions desiredAction) {
    return Commands.runOnce(() -> RobotState.getInstance().desiredAction = desiredAction);
  }

  private BooleanSupplier isDesiredAction(actions desiredAction) {
    return () -> RobotState.getInstance().desiredAction == desiredAction;
  }

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

    new Trigger(isDesiredAction(actions.NONE))
        .onTrue(Commands.parallel(
          new TuckCommand(),
          coralIntake.runVelocityCommand(0),
          Commands.runOnce(() -> drive.clearAutoAlignGoal()),
          Commands.runOnce(() -> drive.clearHeadingGoal())
        ));

    cancelAction.onTrue(setDesiredAction(actions.NONE));

    // Placing
    placeL4.onTrue(setDesiredAction(actions.L4));
    placeL3.onTrue(setDesiredAction(actions.L3));
    placeL2.onTrue(setDesiredAction(actions.L2));
    placeL1.onTrue(setDesiredAction(actions.L1));

    inttake.and(RobotState.getInstance()::isInReefZone).onTrue(setDesiredAction(actions.DEALGIFY));
    inttake.and(() -> !RobotState.getInstance().isInReefZone()).onTrue(setDesiredAction(actions.PICKUP_CORAL));

    outtake.whileTrue(coralIntake.runVelocityCommand(outtakeVelocity));

    processor.onTrue(setDesiredAction(actions.PROCESSOR));

    // Auto align if trying to place L2+
    new Trigger(isDesiredAction(actions.L4))
        .or(isDesiredAction(actions.L3))
        .or(isDesiredAction(actions.L2))
        .whileTrue(new AutoMoveToNearestPOI(false, allignmentMode.TWO_STAGE,
            Reef.placePoses));

    new Trigger(isDesiredAction(actions.L4))
        .and(RobotState.getInstance()::isInReefZone)
        .onTrue(new SetPosition(Constants.l4));

    new Trigger(isDesiredAction(actions.L4))
        .and(() -> !driveCommand.isScheduled())
        .and(drive::isAutoAlignGoalCompleted)
        .onTrue(
            Commands.sequence(
                new PlaceCoral(Constants.l4),
                Commands.runOnce(AutoMoveToNearestPOI::stop),
                Commands.runOnce(() -> RobotState.getInstance().desiredAction = actions.NONE)));

    new Trigger(isDesiredAction(actions.L3))
        .and(RobotState.getInstance()::isInReefZone)
        .onTrue(new SetPosition(Constants.l3));

    new Trigger(isDesiredAction(actions.L3))
        .and(() -> !driveCommand.isScheduled())
        .and(drive::isAutoAlignGoalCompleted)
        .onTrue(
            Commands.sequence(
                new PlaceCoral(Constants.l3),
                Commands.runOnce(AutoMoveToNearestPOI::stop),
                Commands.runOnce(() -> RobotState.getInstance().desiredAction = actions.NONE)));

    new Trigger(isDesiredAction(actions.L2))
        .and(RobotState.getInstance()::isInReefZone)
        .onTrue(new SetPosition(Constants.l2));

    new Trigger(isDesiredAction(actions.L2))
        .and(() -> !driveCommand.isScheduled())
        .and(drive::isAutoAlignGoalCompleted)
        .onTrue(
            Commands.sequence(
                new PlaceCoral(Constants.l2),
                Commands.runOnce(AutoMoveToNearestPOI::stop),
                Commands.runOnce(() -> RobotState.getInstance().desiredAction = actions.NONE)));

    new Trigger(isDesiredAction(actions.L1)).onTrue(new SetPosition(Constants.l1));

    // Pickup Coral
    new Trigger(isDesiredAction(actions.PICKUP_CORAL))
        .onTrue(new AutoMoveToNearestPOI(allignmentMode.TWO_STAGE, CoralStations.pickupLocations));

    new Trigger(isDesiredAction(actions.PICKUP_CORAL))
        .and(RobotState.getInstance()::isInPickupZone)
        .onTrue(Commands.parallel(
            new SetPosition(ElevatorConstants.pickUpPos, CoralActuationConstants.pickUpPos),
            coralIntake.runVelocityCommand(intakeVelocity)));

    new Trigger(isDesiredAction(actions.PICKUP_CORAL)).and(RobotState.getInstance()::hasCoral)
        .onTrue(coralIntake.runOnce(() -> coralIntake.runVelocity(0)));

    new Trigger(isDesiredAction(actions.PICKUP_CORAL))
        .and(RobotState.getInstance()::hasCoral)
        .and(() -> !RobotState.getInstance().isInPickupZone())
        .onTrue(setDesiredAction(actions.NONE));
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
