package frc.robot.subsystems.photonVision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class PhotonIOCamera implements PhotonIO {
    public final PhotonCamera camera;
    public final double cameraHeight;
    public boolean targetVisable;
    public double targetYaw;
    public double targetRange;

    public PhotonIOCamera(String name, double cameraHeight) {
        this.camera = new PhotonCamera(name);
        this.cameraHeight = cameraHeight;
        this.targetVisable = false;
        this.targetYaw = 0.0;
        this.targetRange = 0.0;
    }

    @Override
    public void updateInputs(PhotonIOInputs inputs) {
        inputs.hasTargets = targetVisable;
        inputs.targetDistance = getDistance();
        inputs.targetYaw = getYaw();
    }

    public PhotonPipelineResult getResult() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            return result;
        } else {
            return null;
        }
    }

    @Override
    public double getDistance() {
        var result = getResult();
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                        0.5, // Measured with a tape measure, or in CAD.
                        0.2032, // From game manual height of gamepeice center.
                        Units.degreesToRadians(0), // Measured with a protractor, or in CAD.
                        Units.degreesToRadians(target.getPitch()));
            }
        } else {
            targetRange = -9999;
        }
        return targetRange;
    }

    @Override
    public double getYaw() {
        var result = getResult();
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                targetYaw = target.getYaw();
            }
        } else {
            targetYaw = -9999;
        }

        return targetYaw;
    }

    @Override
    public boolean targetVisable() {
        var result = getResult();
        if (result.hasTargets()) {
            return true;
        } else {
            return false;
        }
    }

}
