package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.LimelightHelpers;

public class VisionIOPhotonVision implements VisionIO {
    PhotonCamera camera;
    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    public VisionIOPhotonVision(String name) {
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        var result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();

        if (result == null) {
            inputs.avgTagArea = 0;
            inputs.avgTagDist = 0;
            inputs.latency = 0;
            inputs.pose = new Pose2d();
            inputs.tagCount = 0;
            inputs.timestampSeconds = 0;
            return;
        }

        double totalArea = 0;
        double totalDist = 0;
        for (PhotonTrackedTarget target : targets) {
            totalArea += target.getArea(); 
            totalDist += target.bestCameraToTarget.getTranslation().getNorm();
        }

        inputs.avgTagArea = totalArea / targets.size();
        inputs.avgTagDist = totalDist / targets.size();
        inputs.latency = 0;
        inputs.pose = estimate.pose;
        inputs.tagCount = targets.size();
        inputs.timestampSeconds = result.getTimestampSeconds();
    }

    @Override
    public void setRobotOrientation(double yaw) {
        LimelightHelpers.SetRobotOrientation(LIMEIGHT_NAME, yaw, 0, 0, 0, 0, 0);
    }

    @Override
    public void setPipeline(int mode) {
        LimelightHelpers.setPipelineIndex(LIMEIGHT_NAME, mode);
    }

    @Override
    public void setLights(int mode) {
        switch (mode) {
            case 0:
                LimelightHelpers.setLEDMode_PipelineControl(LIMEIGHT_NAME);
                break;
            case 1:
                LimelightHelpers.setLEDMode_ForceOff(LIMEIGHT_NAME);
                break;
            case 2:
                LimelightHelpers.setLEDMode_ForceBlink(LIMEIGHT_NAME);
                break;
            case 3:
                LimelightHelpers.setLEDMode_ForceOn(LIMEIGHT_NAME);
                break;
        }
    }
}
