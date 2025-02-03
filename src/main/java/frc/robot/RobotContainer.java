// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.*;
import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.intakeVelocity;
import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.outtakeVelocity;
import static frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.inSpeed;
import static frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.outSpeed;

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
import frc.robot.commands.autos.Place2RightSide;
import frc.robot.util.*;
import frc.robot.util.Alert.AlertType;

public class RobotContainer {
  private final RobotState robotState = RobotState.getInstance();
  private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);

  private static SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static boolean shouldUseZones = true;

  private static final LoggedTunableNumber elevatorVel = new LoggedTunableNumber("Elevator/Velocity", 6);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // FieldConstants.logFieldConstants();

    // Configure autos and buttons
    setupAutos();
    configureButtonBindings(false);

    // Alerts for constants
    if (Constants.tuningMode) {
      new Alert("Tuning mode enabled", AlertType.INFO).set(true);
    }
  }

  private void setupAutos() {
    autoChooser.setDefaultOption("Test Auto", new TestAuto().getAuto().cmd());
    autoChooser.addOption("Place 2 right side", new Place2RightSide().getAuto().cmd());

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
    algaeActuationDown.onTrue(new InstantCommand(() -> algaeActuation.runVoltage(2)))
        .onFalse(new InstantCommand(() -> algaeActuation.stop()));
    algaeActuationUp.onTrue(new InstantCommand(() -> algaeActuation.runVoltage(-2)))
        .onFalse(new InstantCommand(() -> algaeActuation.stop()));

    algaeIntakeIn.onTrue(new InstantCommand(() -> algaeIntake.runVelocity(inSpeed)))
        .onFalse(new InstantCommand(() -> algaeIntake.runVelocity(0)));
    algaeIntakeOut.onTrue(new InstantCommand(() -> algaeIntake.runVelocity(outSpeed)))
        .onFalse(new InstantCommand(() -> algaeIntake.runVelocity(0)));

    // intake
    coralIntakeIn.whileTrue(
        new InstantCommand(() -> coralIntake.setMotorVelocity(intakeVelocity)))
        .onFalse(new InstantCommand(() -> coralIntake.setMotorVelocity(0)));

    coralOuttakeOut.whileTrue(
        new InstantCommand(() -> coralIntake.setMotorVelocity(outtakeVelocity)))
        .onFalse(new InstantCommand(() -> coralIntake.setMotorVelocity(0)));

    coralActuationUp.whileTrue(new InstantCommand(() -> coralActuation.runVoltage(4)))
        .whileFalse(new InstantCommand(() -> coralActuation.stop()));
    coralActuationDown.whileTrue(new InstantCommand(() -> coralActuation.runVoltage(-4)))
        .whileFalse(new InstantCommand(() -> coralActuation.stop()));

    // Elevator
    elevatorUp.onTrue(new InstantCommand(() -> elevator.runVoltage(elevatorVel.get())))
        .onFalse(new InstantCommand(() -> elevator.stop()));
    elevatorDown.onTrue(new InstantCommand(() -> elevator.runVoltage(-elevatorVel.get())))
        .onFalse(new InstantCommand(() -> elevator.stop()));
    
    // Climber
    climberUp.onTrue(new InstantCommand(() -> climber.runVoltage(12), climber)).onFalse(Commands.runOnce(() -> climber.runVoltage(0), climber));
    climberDown.onTrue(new InstantCommand(() -> climber.runVoltage(-12), climber)).onFalse(Commands.runOnce(() -> climber.runVoltage(0), climber));
    

    // Zoning laws
    new Trigger(RobotState.getInstance()::isInClimbZone)
        .whileTrue(new RepeatCommand(new InstantCommand(() -> System.out.println("Climb: " + Timer.getFPGATimestamp()))));

    new Trigger(RobotState.getInstance()::isInReefZone)
        .whileTrue(new RepeatCommand(new InstantCommand(() -> System.out.println("Reef: " + Timer.getFPGATimestamp()))));
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
