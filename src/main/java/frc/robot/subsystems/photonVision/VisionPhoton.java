package frc.robot.subsystems.photonVision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionPhoton extends SubsystemBase {
    private VisionPhotonIO photonVisionIO;
    private VisionPhotonIOInputsAutoLogged inputs = new VisionPhotonIOInputsAutoLogged();
    // private visionPhotonIOInputsAutoLogged inputs = new VisionPhotonIO
    // be negative
    public final Transform3d cameraPos;

    public VisionPhoton(VisionPhotonIO photonVisionIO, Transform3d cameraPosition) {
        this.photonVisionIO = photonVisionIO;
        cameraPos = cameraPosition;
    }

    @Override
    public void periodic() {
        photonVisionIO.updateInputs(inputs);
        Logger.processInputs("photonVision", inputs);
    }

    public Pose2d getGamePiecePose(Pose2d robotPose) {
        Pose3d robotPose3d = new Pose3d(robotPose);
        robotPose3d.plus(photonVisionIO.GamePieceToCamera());
        robotPose3d.plus(cameraPos);
        return robotPose3d.toPose2d();
    }
}
