package frc.robot.subsystems.photonVision;

public class targetInfo {
    public boolean targetVisable;
    public double targetYaw;
    public double targetDistance;
    public targetInfo(boolean targetVisable, double targetYaw, double targetDistance) {
        this.targetVisable = targetVisable;
        this.targetYaw = targetYaw;
        this.targetDistance = targetDistance;
    }
}
