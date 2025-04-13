package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.subsystems.drive.Drive;

public class Vision extends SubsystemBase {
    private final String visionName;
    private final int visionNum;

    private final Timer enabledTimer = new Timer();

    private final VisionIO visionIO;
    private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    public Vision(String visionName, int visionNum, VisionIO visionIO, Supplier<ChassisSpeeds> chassisSpeedsSuppier) {
        this.visionName = visionName;
        this.visionNum = visionNum;
        this.visionIO = visionIO;
        this.chassisSpeedsSupplier = chassisSpeedsSuppier;

        enabledTimer.start();
    }

    @Override
    public void periodic() {
        final double yaw = RobotState.getInstance().getOdometryPose().getRotation().getDegrees();
        visionIO.setRobotOrientation(yaw);

        // if (DriverStation.getAlliance().isPresent()
        //         && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        if (RobotState.getInstance().getEstimatedPose().getX() > (FieldConstants.fieldLength / 2)) {
            visionIO.setPipeline(1);
        } else {
            visionIO.setPipeline(0);
        }

        visionIO.updateInputs(visionInputs);
        Logger.processInputs(visionName, visionInputs);

        if (DriverStation.isDisabled()) {
            enabledTimer.reset();
        };

        if (!enabledTimer.hasElapsed(0.5)) return;

        final ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();
        if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 9.42478 || (Math.abs(chassisSpeeds.vxMetersPerSecond) > 2.0
                || Math.abs(chassisSpeeds.vyMetersPerSecond) > 2.0)) {
            return;
        }

        if (visionInputs.tagCount == 0)
            return;

        double xyStdDev =
            xyStdDevCoefficient
                * Math.pow(visionInputs.avgTagDist, 2.0)
                / visionInputs.tagCount
                * cameraStdDevFactors[visionNum];

        RobotState.getInstance()
                .addVisionObservation(new VisionObservation(visionInputs.pose, visionInputs.timestampSeconds,
                        VecBuilder.fill(
                                xyStdDev,
                                xyStdDev,
                                Double.POSITIVE_INFINITY)));

    }

    public void setPipeline(int mode) {
        visionIO.setPipeline(mode);
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
