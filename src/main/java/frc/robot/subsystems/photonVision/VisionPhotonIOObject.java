package frc.robot.subsystems.photonVision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionPhotonIOObject implements VisionPhotonIO {
    public final PhotonCamera camera;
    
    public VisionPhotonIOObject(String name) {
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(VisionPhotonIOInputs inputs) {
        inputs.hasTargets = !camera.getAllUnreadResults().isEmpty();
    }

    @Override
    public Transform3d GamePieceToCamera() {
        return camera.getLatestResult().getBestTarget().getBestCameraToTarget();
    }

}
