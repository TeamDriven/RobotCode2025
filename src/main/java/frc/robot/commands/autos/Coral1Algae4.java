package frc.robot.commands.autos;

import static frc.robot.Constants.barge;
import static frc.robot.Constants.l4;
import static frc.robot.subsystems.intake.IntakeConstants.intakeVelocity;
import static frc.robot.subsystems.intake.IntakeConstants.outtakeVelocity;
import static frc.robot.Subsystems.actuation;
import static frc.robot.Subsystems.drive;
import static frc.robot.Subsystems.elevator;
import static frc.robot.Subsystems.intake;
import static frc.robot.subsystems.actuation.ActuationConstants.L4Pos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants.CoralStations;
import frc.robot.FieldConstants.Reef;
import frc.robot.commands.automation.SetPosition;
import frc.robot.commands.automation.TuckCommand;
import frc.robot.subsystems.actuation.ActuationConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.AutoAlignController.allignmentMode;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.RobotState;

public class Coral1Algae4 implements AutoBase {
    @Override
    public AutoRoutine getAuto() {
        AutoRoutine routine = drive.autoFactory.newRoutine("Coral1Algae4");

        Transform2d place1Offset = new Transform2d(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)), new Rotation2d());
        
        var pickup1 = routine.trajectory("coral 1 algae 4", 0);
        var place1 = routine.trajectory("coral 1 algae 4", 1);
        var pickup2 = routine.trajectory("coral 1 algae 4", 2);
        var place2 = routine.trajectory("coral 1 algae 4", 3);  
        var pickup3 = routine.trajectory("coral 1 algae 4", 4);
        var place3 = routine.trajectory("coral 1 algae 4", 5);
        var pickup4 = routine.trajectory("coral 1 algae 4", 6);
        var place4 = routine.trajectory("coral 1 algae 4", 7);

        routine.active().onTrue(
                Commands.sequence(
                    pickup1.resetOdometry(),
                        Commands.runOnce(() -> drive.orientModules(Drive.getStraightOrientations()), drive),
                        elevator.resetPosition(),
                        Commands.parallel(
                            pickup1.cmd(),
                                Commands.sequence(
                                    elevator.runOnce(() -> elevator.setPos(ElevatorConstants.tuckPos)),
                                    Commands.waitSeconds(0.25),
                                    actuation.runOnce(() -> actuation.setPos(ActuationConstants.tuckPos))
                                )
                        )));
// Commands.runOnce(() -> System.out.println(DriverStation.getMatchTime()))));
pickup1.atTime("Pick up Algae 1").onTrue(
    Commands.sequence(
            //  Place Coral
            Commands.runOnce(
                    () -> drive.setAutoAlignGoal(
                            () -> AllianceFlipUtil.apply(Reef.placePoses[5].transformBy(place1Offset)), 
                            () -> new Translation2d(),
                            allignmentMode.SLOW),
                    drive),
            new SetPosition(l4),
            new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
            Commands.waitUntil(() -> elevator.isAtHeight(l4.elevatorHeight().getAsDouble(), 0.25) || !elevator.isMoving()),
            Commands.waitUntil(() -> actuation.isAtAngle()),
            actuation.runOnce(() -> actuation.setPos(L4Pos.getAsDouble() - 10)),
            Commands.waitUntil(() -> false).withTimeout(0.25),
            intake.runOnce(() -> intake.runVelocity(intakeVelocity.get())),
            Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
            //Pickup Algae
            Commands.runOnce(
                    () -> drive.setAutoAlignGoal(
                            () -> AllianceFlipUtil.apply(Reef.reefFaces[3].facePos()), 
                            () -> new Translation2d(),
                            allignmentMode.SLOW),
                    drive),
            actuation.runOnce(() -> actuation.setPos(ActuationConstants.pickUpPos.getAsDouble())),
            Commands.parallel(
                new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                place1.cmd()
            )));
            
            place1.atTime("place Algae 1").onTrue(
                Commands.sequence(
                        new SetPosition(barge),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.waitUntil(() -> elevator.isAtHeight(l4.elevatorHeight().getAsDouble(), 0.25) || !elevator.isMoving()),
                        Commands.waitUntil(() -> actuation.isAtAngle()),
                        actuation.runOnce(() -> actuation.setPos(L4Pos.getAsDouble() - 10)),
                        Commands.waitUntil(() -> false).withTimeout(0.25),
                        intake.runOnce(() -> intake.runVelocity(intakeVelocity.get())),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        //Pickup Algae
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil.apply(Reef.reefFaces[3].facePos()), 
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        actuation.runOnce(() -> actuation.setPos(ActuationConstants.pickUpPos.getAsDouble())),
                        Commands.parallel(
                            new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                            place1.cmd()
                        )));

        return routine;
    }
}
