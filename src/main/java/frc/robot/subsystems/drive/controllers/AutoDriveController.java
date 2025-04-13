// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.controllers;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;

/** Drive controller for outputting {@link ChassisSpeeds} from driver joysticks. */
public class AutoDriveController {
  private final PIDController xController = new PIDController(1.5, 0.0, 0);
  private final PIDController yController = new PIDController(1.5, 0.0, 0);
  private final PIDController headingController = new PIDController(1.0, 0.0, 0);

  private SwerveSample sample;

  public AutoDriveController() {
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void acceptDriveInput(SwerveSample swerveSample) {
    sample = swerveSample;
  }

  private void logSample() {
    Logger.recordOutput("AutoDrive/pose", new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading)));
    Logger.recordOutput("AutoDrive/vel", new ChassisSpeeds(sample.vx, sample.vy, sample.omega));
    Logger.recordOutput("AutoDrive/accel", new ChassisSpeeds(sample.ax, sample.ay, sample.alpha));
    Logger.recordOutput("AutoDrive/timestamp", sample.t);
  }

  /**
   * Updates the controller with the currently stored state.
   *
   * @return {@link ChassisSpeeds} with driver's requested speeds.
   */
  public ChassisSpeeds update() {
    Pose2d curPose = RobotState.getInstance().getEstimatedPose();
    Rotation2d curRot = RobotState.getInstance().getOdometryPose().getRotation();

    logSample();
    
    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
      sample.vx + xController.calculate(curPose.getX(), sample.x),
      sample.vy + yController.calculate(curPose.getY(), sample.y),
      sample.omega + headingController.calculate(curRot.getRadians(), sample.heading)
    );

    Logger.recordOutput("AutoDrive/FieldRelativeSpeeds", fieldRelativeSpeeds);

    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, curRot);
  }
}
