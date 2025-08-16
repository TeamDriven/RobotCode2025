package frc.robot.subsystems.photonVision;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface PhotonIO {
    @AutoLog
    class PhotonIOInputs {
        public boolean hasTargets = false;
        public double targetYaw = -9999;
        public double targetDistance = -9999;
        // public List<PhotonPipelineResult> targets = null;
    }

    default void updateInputs(PhotonIOInputs inputs) {}

    default targetInfo getTargetInfo() {return new targetInfo(false, 0, 0);}
}
