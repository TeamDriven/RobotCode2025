package frc.robot.subsystems.photonVision;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionPhotonIO {
    @AutoLog
    class VisionPhotonIOInputs {
        public boolean hasTargets = false;
        // public List<PhotonPipelineResult> targets = null;
    }

    default void updateInputs(VisionPhotonIOInputs inputs) {}

    default Transform3d GamePieceToCamera() {return new Transform3d();}
}
