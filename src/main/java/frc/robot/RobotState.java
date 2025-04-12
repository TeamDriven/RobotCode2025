// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants.Zones;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.SwerveDriveWheelPositions;
import frc.robot.util.swerve.ModuleLimits;

import java.util.NoSuchElementException;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
    public record OdometryObservation(
            SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {
    }

    public record VisionObservation(Pose2d visionPose, double timestamp, Vector<N3> stdDevs) {
    }

    private static final double poseBufferSizeSeconds = 2.0;

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    // Pose Estimation Members
    @AutoLogOutput
    private Pose2d odometryPose = new Pose2d();
    @AutoLogOutput
    private Pose2d estimatedPose = new Pose2d();
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer
            .createBuffer(poseBufferSizeSeconds);
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
    // Odometry
    private final SwerveDriveKinematics kinematics;
    private SwerveDriveWheelPositions lastWheelPositions = new SwerveDriveWheelPositions(
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            });
    private Rotation2d lastGyroAngle = new Rotation2d();
    private Twist2d robotVelocity = new Twist2d();
    private Twist2d trajectoryVelocity = new Twist2d();

    private RobotState() {
        for (int i = 0; i < 3; ++i) {
            qStdDevs.set(i, 0, Math.pow(DriveConstants.odometryStateStdDevs.get(i, 0), 2));
        }
        kinematics = DriveConstants.kinematics;
    }

    /** Add odometry observation */
    public void addOdometryObservation(OdometryObservation observation) {
        Twist2d twist = kinematics.toTwist2d(lastWheelPositions.positions, observation.wheelPositions().positions);
        lastWheelPositions = observation.wheelPositions();
        // Check gyro connected
        if (observation.gyroAngle != null) {
            // Update dtheta for twist if gyro connected
            Logger.recordOutput("RobotState/hasGyro", true);
            twist = new Twist2d(
                    twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
            lastGyroAngle = observation.gyroAngle();
        } else {
            Logger.recordOutput("RobotState/hasGyro", false);
        }

        // Add twist to odometry pose
        odometryPose = odometryPose.exp(twist);
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPose);
        // // Calculate diff from last odometry pose and add onto pose estimate
        estimatedPose = estimatedPose.exp(twist);
    }

    // The chance of this fully working is low
    public void addVisionObservation(VisionObservation observation) {
        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds > observation.timestamp()) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }
        // Get odometry based pose at timestamp
        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) {
            // exit if not there
            return;
        }

        // sample --> odometryPose transform and backwards of that
        var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        // get old estimate by applying odometryToSample Transform
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Calculate 3 x 3 vision matrix
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = observation.stdDevs().get(i) * observation.stdDevs().get(i);
        }
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }
        // difference between estimate and vision pose
        Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
        // scale transform by visionK
        var kTimesTransform = visionK.times(
                VecBuilder.fill(
                        transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        Transform2d scaledTransform = new Transform2d(
                kTimesTransform.get(0, 0),
                kTimesTransform.get(1, 0),
                Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

        // Recalculate current estimate by applying scaled transform to old estimate
        // then replaying odometry data
        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
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
        poseBuffer.clear();
    }

    @AutoLogOutput(key = "RobotState/FieldVelocity")
    public Twist2d fieldVelocity() {
        Translation2d linearFieldVelocity = new Translation2d(robotVelocity.dx, robotVelocity.dy)
                .rotateBy(estimatedPose.getRotation());
        return new Twist2d(
                linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
    }

    @AutoLogOutput(key = "RobotState/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    /**
     * Predicts what our pose will be in the future. Allows separate translation and
     * rotation
     * lookaheads to account for varying latencies in the different measurements.
     *
     * @param translationLookaheadS The lookahead time for the translation of the
     *                              robot
     * @param rotationLookaheadS    The lookahead time for the rotation of the robot
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

    @AutoLogOutput(key = "RobotState/OurReefZone")
    public boolean isInOurReefZone() {
        return Zones.OurReefZone.isRobotInZone(estimatedPose);
    }
    
    @AutoLogOutput(key = "RobotState/TheirReefZone")
    public boolean isInTheirReefZone() {
        return Zones.theirReefZone.isRobotInZone(estimatedPose);
    }

    @AutoLogOutput(key = "RobotState/InAReefZone")
    public boolean isInAReefZone() {
        return isInTheirReefZone() || isInOurReefZone();
    }

    @AutoLogOutput(key = "RobotState/LeftPickupZone")
    public boolean isInLeftPickupZone() {
        return Zones.leftPickupZone.isRobotInZone(estimatedPose);
    }

    @AutoLogOutput(key = "RobotState/RightPickupZone")
    public boolean isInRightPickupZone() {
        return Zones.rightPickupZone.isRobotInZone(estimatedPose);
    }

    public boolean isInPickupZone() {
        return isInLeftPickupZone() || isInRightPickupZone();
    }

    @AutoLogOutput(key = "RobotState/ClimbZone")
    public boolean isInClimbZone() {
        return Zones.climbZone.isRobotInZone(estimatedPose);
    }

    private boolean hasAlgae = false;

    @AutoLogOutput(key = "RobotState/hasAlgae")
    public boolean hasAlgae() {
        return hasAlgae;
    }

    public void setGamePiece(boolean sensorTripped) {
        hasAlgae = sensorTripped;
    }

    public static enum actions {
        AUTO,
        // L4,
        // L3,
        // L2,
        // L1,
        // PICKUP_CORAL,
        PLACE_ALGAE,
        FLOOR_PICKUP,
        DEALGIFY_HIGH,
        DEALGIFY_LOW,
        PROCESSOR,
        CLIMB,
        TURTLE,
        NONE
    }

    private actions desiredAction = actions.NONE;

    @AutoLogOutput(key = "RobotState/desiredAction")
    public actions getDesiredAction() {
        return desiredAction;
    }

    public void setDesiredAction(actions action) {
        desiredAction = action;
    }

    private static enum controlMode {
        STANDARD,
        NO_LIMELIGHT,
        MANUAL
    }

    private static controlMode currentMode = controlMode.STANDARD;

    public void setManualMode() {
        if (isManualMode()) {
            currentMode = controlMode.STANDARD;
        } else {
            currentMode = controlMode.MANUAL;
        }
    }

    public boolean isStandardMode() {
        return currentMode == controlMode.STANDARD;
    }

    public boolean isNoLimelightMode() {
        return currentMode == controlMode.NO_LIMELIGHT;
    }

    public boolean isManualMode() {
        return currentMode == controlMode.MANUAL;
    }
}
