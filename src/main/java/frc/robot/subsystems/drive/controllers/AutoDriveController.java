// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.controllers;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;

/** Drive controller for outputting {@link ChassisSpeeds} from driver joysticks. */
public class AutoDriveController {
  private final PIDController xController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(2.75, 0.0, 0.0);

  private SwerveSample sample;

  public AutoDriveController() {
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void acceptDriveInput(SwerveSample swerveSample) {
    sample = swerveSample;
  }

  /**
   * Updates the controller with the currently stored state.
   *
   * @return {@link ChassisSpeeds} with driver's requested speeds.
   */
  public ChassisSpeeds update() {
    Pose2d curPose = RobotState.getInstance().getEstimatedPose();
    Rotation2d curRot = RobotState.getInstance().getOdometryPose().getRotation();
    
    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
      sample.vx + xController.calculate(curPose.getX(), sample.x),
      sample.vy + yController.calculate(curPose.getY(), sample.y),
      sample.omega + headingController.calculate(curRot.getRadians(), sample.heading)
    );

    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, curRot);
  }
}
