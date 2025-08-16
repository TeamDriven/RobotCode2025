package frc.robot.subsystems.photonVision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;

public class PhotonIOCamera implements PhotonIO {
    public final PhotonCamera camera;
    public final double cameraHeight;
    public boolean targetVisible;
    public double targetYaw;
    public double targetRange;

    public PhotonIOCamera(String name, double cameraHeight) {
        this.camera = new PhotonCamera(name);
        this.cameraHeight = cameraHeight;
        this.targetVisible = false;
        this.targetYaw = 0.0;
        this.targetRange = 0.0;
    }

    @Override
    public void updateInputs(PhotonIOInputs inputs) {
        targetInfo target = getTargetInfo();
        inputs.hasTargets = targetVisible;
        inputs.targetDistance = target.targetDistance;
        inputs.targetYaw = target.targetYaw;
    }

    @Override
    public targetInfo getTargetInfo() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                        targetYaw = target.getYaw();
                        targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        0.5, // Measured with a tape measure, or in CAD.
                                        0.2032, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));

                        targetVisible = true;
                        return new targetInfo(targetVisible, targetYaw, targetRange);
                }
            }
        }
        return new targetInfo(false, -9999, -9999);
    }

}
