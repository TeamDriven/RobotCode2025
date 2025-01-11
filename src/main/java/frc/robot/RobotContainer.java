// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.*;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.*;
import frc.robot.util.Alert.AlertType;

public class RobotContainer {
  private final RobotState robotState = RobotState.getInstance();
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);

  private static SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Dashboard inputs
  // private final AutoSelector autoSelector = new AutoSelector("Auto");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure autos and buttons
    configureButtonBindings(false);

    // Alerts for constants
    if (Constants.tuningMode) {
      new Alert("Tuning mode enabled", AlertType.INFO).set(true);
    }
  }

  private Command driveCommand() {
    return drive
        .run(
            () ->
                drive.acceptTeleopInput(
                    driveX.getAsDouble(), driveY.getAsDouble(), driveOmega.getAsDouble(), false))
        .withName("Drive Teleop Input");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings(boolean demo) {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Drivetrain
    drive.setDefaultCommand(driveCommand());

    resetPose.onTrue(
        Commands.runOnce(
                () ->
                    robotState.resetPose(
                        new Pose2d(
                            robotState.getEstimatedPose().getTranslation(), AllianceFlipUtil.apply(new Rotation2d()))))
            .ignoringDisable(true));
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
    //         drive, drive::runCharacterization, drive::getCharacterizationVelocity)
    //     .finallyDo(drive::endCharacterization);

    // Drive FF Characterization
    // return new FeedForwardCharacterization(
    //         drive, drive::runCharacterization, drive::getCharacterizationVelocity)
    //     .finallyDo(drive::endCharacterization);

    // Drive Wheel Radius Characterization
    // return drive
    //     .orientModules(Drive.getCircleOrientations())
    //     .andThen(
    //         new WheelRadiusCharacterization(
    //             drive, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE))
    //     .withName("Drive Wheel Radius Characterization");

    // Slippage Calculator
    // return Commands.runOnce(
    //         () ->
    //             robotState.resetPose(
    //                 new Pose2d(
    //                     // robotState.getEstimatedPose().getTranslation(),
    //                     new Translation2d(), AllianceFlipUtil.apply(new Rotation2d()))))
    //     .andThen(new SlippageCalculator(drive))
    //     .withName("Slippage Calculator");

    return null;
    // return new RepeatCommand(drive.getAutoPath("TestAuto"));
    // return autoChooser.getSelected();
  }
}
