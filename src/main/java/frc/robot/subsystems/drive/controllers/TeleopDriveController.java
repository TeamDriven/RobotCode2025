// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.controllers;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

/** Drive controller for outputting {@link ChassisSpeeds} from driver joysticks. */
public class TeleopDriveController {
  private static final LoggedTunableNumber controllerDeadband =
      new LoggedTunableNumber("TeleopDrive/Deadband", 0.1);
  private static final LoggedTunableNumber maxAngularVelocityScalar =
      new LoggedTunableNumber("TeleopDrive/MaxAngularVelocityScalar", 1.0);

  private static double velocityScalar = 1.0;

  private double controllerX = 0;
  private double controllerY = 0;
  private double controllerOmega = 0;
  private boolean robotRelative = false;

  /**
   * Accepts new drive input from joysticks.
   *
   * @param x Desired x velocity scalar, -1..1
   * @param y Desired y velocity scalar, -1..1
   * @param omega Desired omega velocity scalar, -1..1
   * @param robotRelative Robot relative drive
   */
  public void acceptDriveInput(double x, double y, double omega, boolean robotRelative) {
    controllerX = x;
    controllerY = y;
    controllerOmega = omega;
    this.robotRelative = robotRelative;
  }

  /**
   * Updates the controller with the currently stored state.
   *
   * @return {@link ChassisSpeeds} with driver's requested speeds.
   */
  public ChassisSpeeds update() {
    Translation2d linearVelocity = calcLinearVelocity(controllerX, controllerY);
    double omega = MathUtil.applyDeadband(controllerOmega, controllerDeadband.get());
    omega = Math.copySign(omega * omega, omega);

    final double maxLinearVelocity = driveConfig.maxLinearVelocity() * velocityScalar;
    final double maxAngularVelocity =
        driveConfig.maxAngularVelocity() * (maxAngularVelocityScalar.get() * velocityScalar);
    if (robotRelative) {
      return new ChassisSpeeds(
          linearVelocity.getX() * maxLinearVelocity,
          linearVelocity.getY() * maxLinearVelocity,
          omega * maxAngularVelocity);
    } else {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
      }
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          linearVelocity.getX() * maxLinearVelocity,
          linearVelocity.getY() * maxLinearVelocity,
          omega * maxAngularVelocity,
          RobotState.getInstance().getEstimatedPose().getRotation());
    }
  }

  public static Translation2d calcLinearVelocity(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), controllerDeadband.get());
    Rotation2d linearDirection = new Rotation2d();
    // Have to do this in order to silence an error for some reason
    if (x != 0 || y != 0) {
      linearDirection = new Rotation2d(x, y);
    }
    // Square magnitude
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    return linearVelocity;
  }
}
