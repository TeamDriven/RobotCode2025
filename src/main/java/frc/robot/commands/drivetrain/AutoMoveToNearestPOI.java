package frc.robot.commands.drivetrain;

import static frc.robot.Subsystems.drive;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.controllers.AutoAlignController.allignmentMode;
import frc.robot.util.AllianceFlipUtil;

public class AutoMoveToNearestPOI extends Command {
    private final Pose2d[] poses;
    private List<Pose2d> actualPoses;
    private final allignmentMode mode;
    private final boolean autoFinish;

    private static boolean alignFinished = false;

    private Pose2d selectedPose;

    public AutoMoveToNearestPOI(boolean autoFinish, allignmentMode mode, Pose2d... poses) {
        this.poses = poses;
        this.mode = mode;
        this.autoFinish = autoFinish;
        addRequirements(drive);
    }

    public AutoMoveToNearestPOI(allignmentMode mode, Pose2d... poses) {
        this(true, mode, poses);
    }

    private void findNearestTarget() {
        Pose2d curPose = RobotState.getInstance().getEstimatedPose();

        this.selectedPose = curPose.nearest(actualPoses);
    }

    @Override
    public void initialize() {
        alignFinished = false;

        actualPoses = Arrays.stream(poses).map(AllianceFlipUtil::apply).toList();
        findNearestTarget();
        drive.setAutoAlignGoal(() -> this.selectedPose, () -> new Translation2d(), mode);
    }

    @Override
    public void execute() {
        Pose2d prevSelectedPose = this.selectedPose;
        findNearestTarget();
        if (prevSelectedPose != this.selectedPose) {
            drive.setAutoAlignGoal(() -> this.selectedPose, () -> new Translation2d(), mode);
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        drive.clearAutoAlignGoal();
    }

    @Override
    public boolean isFinished() {
        if (autoFinish) return drive.isAutoAlignGoalCompleted();
        return alignFinished;
    }
    
    public static void stop() {
        alignFinished = true;
    }
}
