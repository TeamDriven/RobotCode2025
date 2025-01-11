package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {

    @AutoLog
    class VisionIOInputs {
        public Pose2d pose = null;
        public double timestampSeconds = 0;
        public double latency = 0;
        public double tagCount = 0;
        public double avgTagDist = 0;
        public double avgTagArea = 0;
    }

    default void updateInputs(VisionIOInputs inputs) {}

    /**
     * Tells the limelight where you are for setting up MegaTag2
     * @param yaw The current yaw of your robot
     */
    default void setRobotOrientation(double yaw) {}

    default void setPipeline(int mode) {}

    /**
     * @param mode 0 for active, 1 for inactive
     */
    default void setCameraMode(int mode) {}

    /**
     * @param mode 0 for default, 1 for off, 2 for blink, 3 for on
     */
    default void setLights(int mode) {}
}