// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.SwerveDriveWheelPositions;
import frc.robot.util.swerve.ModuleLimits;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  public record OdometryObservation(
    SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {}

  public record VisionObservation(Pose2d visionPose, double timestamp, Vector<N3> stdDevs) {}

  private static final double poseBufferSizeSeconds = 2.0;

  private static final Lock odometryLock = new ReentrantLock();

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose Estimation Members
  @AutoLogOutput private Pose2d odometryPose = new Pose2d();
  @AutoLogOutput private Pose2d estimatedPose = new Pose2d();
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  // Odometry
  private final SwerveDriveKinematics kinematics;
  private SwerveDriveWheelPositions lastWheelPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });
  private Rotation2d lastGyroAngle = new Rotation2d();
  private Twist2d robotVelocity = new Twist2d();
  private Twist2d trajectoryVelocity = new Twist2d();
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(DriveConstants.odometryStateStdDevs.get(i, 0), 2));
    }
    kinematics = DriveConstants.kinematics;
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(kinematics, odometryPose.getRotation(), lastWheelPositions.positions, estimatedPose);
  }

  /** Add odometry observation */
  public void addOdometryObservation(OdometryObservation observation) {
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions.positions, observation.wheelPositions().positions);
    lastWheelPositions = observation.wheelPositions();
    // Check gyro connected
    if (observation.gyroAngle != null) {
      // Update dtheta for twist if gyro connected
      twist =
          new Twist2d(
              twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
      lastGyroAngle = observation.gyroAngle();
    }

    odometryLock.lock();

    // Add twist to odometry pose
    odometryPose = odometryPose.exp(twist);
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    // // Calculate diff from last odometry pose and add onto pose estimate
    // estimatedPose = estimatedPose.exp(twist);
    if (observation.gyroAngle != null) {
      swerveDrivePoseEstimator.updateWithTime(observation.timestamp(), observation.gyroAngle(), observation.wheelPositions().positions);
    }
    estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();

    odometryLock.unlock();
  }

  // The chance of this fully working is low
  public void addVisionObservation(VisionObservation observation) {
    odometryLock.lock();

    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    Logger.recordOutput("Limelight/SentPose", observation.visionPose);

    swerveDrivePoseEstimator.addVisionMeasurement(observation.visionPose(), observation.timestamp(), observation.stdDevs());
    estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();

    odometryLock.unlock();
  }

  public void addVelocityData(Twist2d robotVelocity) {
    this.robotVelocity = robotVelocity;
  }

  public void addTrajectoryVelocityData(Twist2d robotVelocity) {
    trajectoryVelocity = robotVelocity;
  }

  // when adding other systems, you can make the limits change when things happen
  // E.X. when a flywheel is accelerating so you don't brown
  public ModuleLimits getModuleLimits() {
    return DriveConstants.moduleLimitsFree;
  }

  /**
   * Reset estimated pose and odometry pose to pose <br>
   * Clear pose buffer
   */
  public void resetPose(Pose2d initialPose) {
    estimatedPose = initialPose;
    odometryPose = initialPose;
    swerveDrivePoseEstimator.resetPose(initialPose);
    poseBuffer.clear();
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public Twist2d fieldVelocity() {
    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(estimatedPose.getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  /**
   * Predicts what our pose will be in the future. Allows separate translation and rotation
   * lookaheads to account for varying latencies in the different measurements.
   *
   * @param translationLookaheadS The lookahead time for the translation of the robot
   * @param rotationLookaheadS The lookahead time for the rotation of the robot
   * @return The predicted pose.
   */
  public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
    Twist2d velocity = DriverStation.isAutonomousEnabled() ? trajectoryVelocity : robotVelocity;
    return getEstimatedPose()
        .transformBy(
            new Transform2d(
                velocity.dx * translationLookaheadS,
                velocity.dy * translationLookaheadS,
                Rotation2d.fromRadians(velocity.dtheta * rotationLookaheadS)));
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }
}
