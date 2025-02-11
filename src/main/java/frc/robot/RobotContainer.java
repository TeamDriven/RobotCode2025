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

import static frc.robot.Constants.*;
import static frc.robot.Controls.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autos.TestAuto;
import frc.robot.commands.autos.TestPath;
import frc.robot.commands.drivetrain.AutoMoveToNearestPOI;
import frc.robot.FieldConstants.Reef;
import frc.robot.commands.autos.Place2RightSide;
import frc.robot.commands.autos.Place4LeftSide;
import frc.robot.commands.autos.Place4RightSide;
import frc.robot.util.*;
import frc.robot.util.Alert.AlertType;

public class RobotContainer {
  private final RobotState robotState = RobotState.getInstance();
  private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);

  private static SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static boolean shouldUseZones = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // FieldConstants.logFieldConstants();

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
    autoChooser.setDefaultOption("Test Auto", new TestAuto().getAuto().cmd());
    autoChooser.addOption("Place 2 right side", new Place2RightSide().getAuto().cmd());
    autoChooser.addOption("TestPath", new TestPath().getAuto().cmd());
    autoChooser.addOption("place 4 right side", new Place4RightSide().getAuto().cmd());
    autoChooser.addOption("place 4 left side", new Place4LeftSide().getAuto().cmd());

    SmartDashboard.putData(autoChooser);
  }

  private Command driveCommand() {
    return drive
        .run(
            () -> drive.acceptTeleopInput(
                driveX.getAsDouble(), driveY.getAsDouble(), driveOmega.getAsDouble(), false))
        .withName("Drive Teleop Input");
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
    drive.setDefaultCommand(driveCommand());

    resetPose.onTrue(
        Commands.runOnce(
            () -> robotState.resetPose(
                new Pose2d(
                    robotState.getEstimatedPose().getTranslation(), AllianceFlipUtil.apply(new Rotation2d()))))
            .ignoringDisable(true));

    // Algae
    // algaeActuationUp.onTrue(algaeActuation.runVoltageCommand(algaeActuationVoltage.get()));
    // algaeActuationDown.onTrue(algaeActuation.runVoltageCommand(-algaeActuationVoltage.get()));

    // algaeIntakeIn.onTrue(algaeIntake.runVelocityCommand(algaeIntakeTuningVelocity.get()));
    // algaeIntakeOut.onTrue(algaeIntake.runVelocityCommand(-algaeIntakeTuningVelocity.get()));

    // intake
    // coralIntakeIn.whileTrue(coralIntake.runVelocityCommand(intakeVelocity.get()));
    // coralOuttakeOut.whileTrue(coralIntake.runVelocityCommand(outtakeVelocity.get()));

    // coralActuationUp.whileTrue(coralActuation.runVoltageCommand(coralActuationTuningVoltage.get()));
    // coralActuationDown.whileTrue(coralActuation.runVoltageCommand(-coralActuationTuningVoltage.get()));

    // Elevator
    // elevatorUp.onTrue(elevator.runVoltageCommand(elevatorTuningVoltage.get()));
    // elevatorDown.onTrue(elevator.runVoltageCommand(-elevatorTuningVoltage.get()));

    // Climber
    // climberUp.onTrue(climber.runVoltageCommand(climberTuningVoltage.get()));
    // climberDown.onTrue(climber.runVoltageCommand(-climberTuningVoltage.get()));

    // Zoning laws
    // new Trigger(RobotState.getInstance()::isInClimbZone)
    //     .whileTrue(
    //         new RepeatCommand(new InstantCommand(() -> System.out.println("Climb: " + Timer.getFPGATimestamp()))));

    // new Trigger(RobotState.getInstance()::isInReefZone)
    //     .whileTrue(
    //         new RepeatCommand(new InstantCommand(() -> System.out.println("Reef: " + Timer.getFPGATimestamp()))));

    // new Trigger(RobotState.getInstance()::isInLeftPickupZone)
    //     .whileTrue(
    //         new RepeatCommand(new InstantCommand(() -> System.out.println("Left Pickup: " + Timer.getFPGATimestamp()))));

    // new Trigger(RobotState.getInstance()::isInRightPickupZone)
    //     .whileTrue(
    //         new RepeatCommand(new InstantCommand(() -> System.out.println("Right Pickup: " + Timer.getFPGATimestamp()))));

    driver.a().whileTrue(
        new AutoMoveToNearestPOI(false, Reef.placePoses).andThen(new AutoMoveToNearestPOI(true, Reef.placePoses)));
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
