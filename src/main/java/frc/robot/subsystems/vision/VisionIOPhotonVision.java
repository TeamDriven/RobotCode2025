package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionIOPhotonVision implements VisionIO {
    PhotonCamera camera;
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0));
    private final PhotonPoseEstimator photonEstimator;

    public VisionIOPhotonVision(String name) {
        camera = new PhotonCamera(name);
        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        var result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();

        double totalArea = 0;
        double totalDist = 0;
        for (PhotonTrackedTarget target : targets) {
            totalArea += target.getArea();
            totalDist += target.bestCameraToTarget.getTranslation().getNorm();
        }

        
        inputs.avgTagArea = totalArea / targets.size();
        inputs.avgTagDist = totalDist / targets.size();
        inputs.latency = 0;
        inputs.pose = estimatedPose2dAverage();
        inputs.tagCount = targets.size();
        inputs.timestampSeconds = result.getTimestampSeconds();
    }

    @Override
    public void setPipeline(int mode) {
        camera.setPipelineIndex(mode);
    }

    private Pose2d estimatedPose2dAverage() {
        double size = camera.getAllUnreadResults().size();
        double sumX = 0;
        double sumY = 0;
        double sumRadians = 0;
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        // If a valid pose estimate is available
        if (visionEst.isPresent()) {
            for (var change : camera.getAllUnreadResults()) {
                visionEst = photonEstimator.update(change);
                EstimatedRobotPose est = visionEst.get();
                // Get the estimated Pose3d of the robot
                Pose3d estimatedPose3d = est.estimatedPose; 
                Pose2d estimatedPose2d = estimatedPose3d.toPose2d();
                sumX += estimatedPose2d.getX();
                sumY += estimatedPose2d.getY();
                sumRadians += estimatedPose2d.getRotation().getRadians();
            }   
        }
        return new Pose2d(new Translation2d(sumX/size,sumY/size), new Rotation2d(sumRadians/size));
    }
}
