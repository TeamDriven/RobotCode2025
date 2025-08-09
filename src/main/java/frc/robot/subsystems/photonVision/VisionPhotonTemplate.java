package frc.robot.subsystems.photonVision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionPhotonTemplate extends SubsystemBase {
    PhotonCamera camera;
    Transform3d cameraPos;

    public VisionPhotonTemplate(String name, Transform3d cameraPos) {
        this.camera = new PhotonCamera(name);
        this.cameraPos = cameraPos;
    }

    public Pose2d getGamePiecePose(PhotonTrackedTarget target, Pose2d robotPose) {
        Pose3d robotPose3d = new Pose3d(robotPose);
        robotPose3d.plus(target.getBestCameraToTarget());
        return robotPose3d.toPose2d();
    }

    @Override
    public void periodic() {
        // Query the latest result from PhotonVision.
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        // Does it have targets.
        boolean hasTargets = !results.isEmpty();

        // Get a list of currently tracked targets.
        if (hasTargets) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();
                double targetX = target.;
                double targetY = target.getCenterY();
            }
        }
    }
}
