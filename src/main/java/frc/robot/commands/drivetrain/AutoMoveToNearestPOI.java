package frc.robot.commands.drivetrain;

import static frc.robot.Subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

public class AutoMoveToNearestPOI extends Command {
    private final Pose2d[] poses;
    private final boolean slowMode;

    private Pose2d selectedPose;

    public AutoMoveToNearestPOI(boolean slowMode, Pose2d... poses) {
        this.poses = poses;
        this.slowMode = slowMode;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Select a pose here so it won't be overridden if you accidentally move closer to another POI on the way
        Pose2d selectedPose = null;
        double minDist = Double.POSITIVE_INFINITY;

        Pose2d curPose = RobotState.getInstance().getEstimatedPose();

        for (Pose2d pose : poses) {
            double dist = curPose.getTranslation().getDistance(pose.getTranslation());
            if (dist < minDist) {
                minDist = dist;
                selectedPose = pose;
            }
        }

        this.selectedPose = selectedPose;
        // Currently feedforward is empty, you can use this to add driver corrections see
        // https://github.com/Mechanical-Advantage/RobotCode2024/blob/fee73633da2a04a496cb7cf8bb0f6d78a67ae8d1/src/main/java/org/littletonrobotics/frc2024/RobotContainer.java#L748
        drive.setAutoAlignGoal(() -> this.selectedPose, () -> new Translation2d(), slowMode);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean isInterrupted) {
        drive.clearAutoAlignGoal();
    }

    @Override
    public boolean isFinished() {
        return drive.isAutoAlignGoalCompleted();
    }
    
}
