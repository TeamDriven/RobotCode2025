package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
    public final String LIMEIGHT_NAME;

    public VisionIOLimelight(String name) {
        LIMEIGHT_NAME = name;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMEIGHT_NAME);

        if (estimate == null){
            inputs.avgTagArea = 0;
            inputs.avgTagDist = 0;
            inputs.latency = 0;
            inputs.pose = new Pose2d();
            inputs.tagCount = 0;
            inputs.timestampSeconds = 0;
            return;
        }
        
        inputs.avgTagArea = estimate.avgTagArea;
        inputs.avgTagDist = estimate.avgTagDist;
        inputs.latency = estimate.latency;
        inputs.pose = estimate.pose;
        inputs.tagCount = estimate.tagCount;
        inputs.timestampSeconds = estimate.timestampSeconds;
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
