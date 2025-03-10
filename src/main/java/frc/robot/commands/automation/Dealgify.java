package frc.robot.commands.automation;

import static frc.robot.Subsystems.actuation;
import static frc.robot.Subsystems.drive;
import static frc.robot.Subsystems.elevator;
import static frc.robot.Subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.Reef.ReefFace;
import frc.robot.subsystems.actuation.ActuationConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.IntakeConstants;

public class Dealgify extends Command {

    private ReefFace nearestFace;
    
    public Dealgify() {
        addRequirements(intake, elevator, actuation);
    }

    @Override
    public void initialize() {
        Pose2d curPose = RobotState.getInstance().getEstimatedPose();
        nearestFace = Reef.findNearestReefFace(curPose);

        intake.runVelocity(IntakeConstants.intakeVelocity.get());
        
        drive.setHeadingGoal(() -> nearestFace.facePos().getRotation().rotateBy(new Rotation2d(Math.PI)));

        actuation.setPos(ActuationConstants.dealgifyPos);

        elevator.setPos(nearestFace.isAlgaeHigh() ? ElevatorConstants.highDealgifyPos : ElevatorConstants.lowDealgifyPos);
    }

    @Override
    public void execute() {
        Pose2d curPose = RobotState.getInstance().getEstimatedPose();
        ReefFace face = Reef.findNearestReefFace(curPose);

        if (face.equals(nearestFace)) return;

        nearestFace = face;

        drive.setHeadingGoal(() -> nearestFace.facePos().getRotation().rotateBy(new Rotation2d(Math.PI)));

        actuation.setPos(ActuationConstants.dealgifyPos);

        elevator.setPos(nearestFace.isAlgaeHigh() ? ElevatorConstants.highDealgifyPos : ElevatorConstants.lowDealgifyPos);
    }

    @Override
    public void end(boolean isInterrupted) {
        drive.clearHeadingGoal();
        intake.runVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
