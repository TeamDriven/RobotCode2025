package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;

public class Vision extends SubsystemBase {
    private final String visionName;

    private final VisionIO visionIO;
    private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    
    public Vision(String visionName, VisionIO visionIO, Supplier<ChassisSpeeds> chassisSpeedsSuppier) {
        this.visionName = visionName;
        this.visionIO = visionIO;
        this.chassisSpeedsSupplier = chassisSpeedsSuppier;
    }

    @Override
    public void periodic() {
        final double yaw = RobotState.getInstance().getOdometryPose().getRotation().getDegrees();
        visionIO.setRobotOrientation(yaw);
        
        visionIO.updateInputs(visionInputs);
        Logger.processInputs(visionName, visionInputs);


        final ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();
        if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 9.42478 || (Math.abs(chassisSpeeds.vxMetersPerSecond) > 2.0
                || Math.abs(chassisSpeeds.vyMetersPerSecond) > 2.0)) {
            return;
        }

        if (visionInputs.tagCount == 0) {
            return;
        }

        RobotState.getInstance().addVisionObservation(new VisionObservation(visionInputs.pose, visionInputs.timestampSeconds, VecBuilder.fill(
                        Math.pow(0.8, visionInputs.tagCount) * (visionInputs.avgTagDist) * 2,
                        Math.pow(0.8, visionInputs.tagCount) * (visionInputs.avgTagDist) * 2,
                        9999999)));

    }

    public void setPipeline(int mode) {
        visionIO.setPipeline(mode);
    }

    /**
     * @param mode 0 for active, 1 for inactive
     */
    public void setCameraMode(int mode) {
        visionIO.setCameraMode(mode);
    }

    /**
     * @param mode 0 for default, 1 for off, 2 for blink, 3 for on
     */
    public void setLights(int mode) {
        visionIO.setLights(mode);
    }

    public double getAvgDist() {
        return visionInputs.avgTagDist;
    }

    public double getTagCount() {
        return visionInputs.tagCount;
    }
}
