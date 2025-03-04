// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.DriveConstants;

import java.util.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  // Must be less than 2.0
  // private static final LoggedTunableNumber txTyObservationStaleSecs =
  // new LoggedTunableNumber("RobotState/TxTyObservationStaleSeconds", 0.5);
  // private static final LoggedTunableNumber minDistanceTagPoseBlend =
  // new LoggedTunableNumber("RobotState/MinDistanceTagPoseBlend",
  // Units.inchesToMeters(24.0));
  // private static final LoggedTunableNumber maxDistanceTagPoseBlend =
  // new LoggedTunableNumber("RobotState/MaxDistanceTagPoseBlend",
  // Units.inchesToMeters(36.0));

  private static final double poseBufferSizeSec = 2.0;
  private static final Lock odometryLock = new ReentrantLock();

  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));
  // private static final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null)
      instance = new RobotState();
    return instance;
  }

  // Pose Estimation Members
  private Pose2d odometryPose = new Pose2d();

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  private Pose2d estimatedPose = new Pose2d();

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  // Odometry
  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };
  // Assume gyro starts at zero
  private Rotation2d gyroOffset = new Rotation2d();

  // private final Map<Integer, TxTyPoseRecord> txTyPoses = new HashMap<>();
  // private Set<AlgaePoseRecord> algaePoses = new HashSet<>();

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  @AutoLogOutput(key = "RobotState/RobotVelocity")
  public ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
  }

  private OptionalDouble distanceToBranch = OptionalDouble.empty();

  public OptionalDouble getDistanceToBranch() {
    return distanceToBranch;
  }

  public void setDistanceToBranch(OptionalDouble distance) {
    distanceToBranch = distance;
  }

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }
    kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(kinematics, odometryPose.getRotation(), lastWheelPositions,
        estimatedPose);
  }

  public void resetPose(Pose2d pose) {
    // Gyro offset is the rotation that maps the old gyro rotation (estimated -
    // offset) to the new
    // frame of rotation
    gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
    estimatedPose = pose;
    odometryPose = pose;
    poseBuffer.clear();
  }

  public void addOdometryObservation(OdometryObservation observation) {
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();
    Pose2d lastOdometryPose = odometryPose;
    odometryPose = odometryPose.exp(twist);
    // Use gyro if connected
    if (observation.gyroAngle != null) {
      Rotation2d angle = observation.gyroAngle.plus(gyroOffset);
      odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
    }
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);

    odometryLock.lock();

    // Add twist to odometry pose
    odometryPose = odometryPose.exp(twist);
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    // // Calculate diff from last odometry pose and add onto pose estimate
    // estimatedPose = estimatedPose.exp(twist);
    if (observation.gyroAngle != null) {
      swerveDrivePoseEstimator.updateWithTime(observation.timestamp(), observation.gyroAngle,
          observation.wheelPositions());
      estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();
    }

    odometryLock.unlock();
  }

  public void addVisionObservation(VisionObservation observation) {
    odometryLock.lock();

    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    Logger.recordOutput("Limelight/SentPose", observation.visionPose);

    swerveDrivePoseEstimator.addVisionMeasurement(observation.visionPose(), observation.timestamp(),
        observation.stdDevs());
    estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();

    odometryLock.unlock();
  }

  // public void addTxTyObservation(TxTyObservation observation) {
  // // Skip if current data for tag is newer
  // if (txTyPoses.containsKey(observation.tagId())
  // && txTyPoses.get(observation.tagId()).timestamp() >= observation.timestamp())
  // {
  // return;
  // }

  // // Get rotation at timestamp
  // var sample = poseBuffer.getSample(observation.timestamp());
  // if (sample.isEmpty()) {
  // // exit if not there
  // return;
  // }
  // Rotation2d robotRotation =
  // estimatedPose.transformBy(new Transform2d(odometryPose,
  // sample.get())).getRotation();

  // // Average tx's and ty's
  // double tx = 0.0;
  // double ty = 0.0;
  // for (int i = 0; i < 4; i++) {
  // tx += observation.tx()[i];
  // ty += observation.ty()[i];
  // }
  // tx /= 4.0;
  // ty /= 4.0;

  // Pose3d cameraPose =
  // VisionConstants.cameras[observation.camera()].pose().get();

  // // Use 3D distance and tag angles to find robot pose
  // Translation2d camToTagTranslation =
  // new Pose3d(Translation3d.kZero, new Rotation3d(0, ty, -tx))
  // .transformBy(
  // new Transform3d(new Translation3d(observation.distance(), 0, 0),
  // Rotation3d.kZero))
  // .getTranslation()
  // .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
  // .toTranslation2d();
  // Rotation2d camToTagRotation =
  // robotRotation.plus(
  // cameraPose.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));
  // var tagPose2d = tagPoses2d.get(observation.tagId());
  // if (tagPose2d == null) return;
  // Translation2d fieldToCameraTranslation =
  // new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
  // .transformBy(GeomUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
  // .getTranslation();
  // Pose2d robotPose =
  // new Pose2d(
  // fieldToCameraTranslation,
  // robotRotation.plus(cameraPose.toPose2d().getRotation()))
  // .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));
  // // Use gyro angle at time for robot rotation
  // robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

  // // Add transform to current odometry based pose for latency correction
  // txTyPoses.put(
  // observation.tagId(),
  // new TxTyPoseRecord(robotPose, camToTagTranslation.getNorm(),
  // observation.timestamp()));
  // }

  public void addDriveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  /** Get 2d pose estimate of robot if not stale. */
  // public Optional<Pose2d> getTxTyPose(int tagId) {
  // if (!txTyPoses.containsKey(tagId)) {
  // return Optional.empty();
  // }
  // var data = txTyPoses.get(tagId);
  // // Check if stale
  // if (Timer.getTimestamp() - data.timestamp() >=
  // txTyObservationStaleSecs.get()) {
  // return Optional.empty();
  // }
  // // Get odometry based pose at timestamp
  // var sample = poseBuffer.getSample(data.timestamp());
  // // Latency compensate
  // return sample.map(pose2d -> data.pose().plus(new Transform2d(pose2d,
  // odometryPose)));
  // }

  /**
   * Get estimated pose using txty data given tagId on reef and aligned pose on
   * reef. Used for algae
   * intaking and coral scoring.
   */
  // public Pose2d getReefPose(int face, Pose2d finalPose) {
  // final boolean isRed = AllianceFlipUtil.shouldFlip();
  // var tagPose =
  // getTxTyPose(
  // switch (face) {
  // case 1 -> isRed ? 6 : 19;
  // case 2 -> isRed ? 11 : 20;
  // case 3 -> isRed ? 10 : 21;
  // case 4 -> isRed ? 9 : 22;
  // case 5 -> isRed ? 8 : 17;
  // // 0
  // default -> isRed ? 7 : 18;
  // });
  // // Use estimated pose if tag pose is not present
  // if (tagPose.isEmpty()) return RobotState.getInstance().getEstimatedPose();
  // // Use distance from estimated pose to final pose to get t value
  // final double t =
  // MathUtil.clamp(
  // (getEstimatedPose().getTranslation().getDistance(finalPose.getTranslation())
  // - minDistanceTagPoseBlend.get())
  // / (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
  // 0.0,
  // 1.0);
  // return getEstimatedPose().interpolate(tagPose.get(), 1.0 - t);
  // }

  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, double timestamp) {
  }

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
  }

  public boolean hasCoral = true;

  public static enum actions {
    L4,
    L3,
    L2,
    L1,
    PICKUP_CORAL,
    PICKUP_ALGAE,
    DEALGIFY,
    CLIMB,
    NONE
  }

  public actions desiredAction = actions.NONE;
}