package frc.robot.commands.autos;

import static frc.robot.Constants.l4;
import static frc.robot.subsystems.intake.IntakeConstants.intakeVelocity;
import static frc.robot.subsystems.intake.IntakeConstants.outtakeVelocity;
import static frc.robot.Subsystems.actuation;
import static frc.robot.Subsystems.drive;
import static frc.robot.Subsystems.elevator;
import static frc.robot.Subsystems.intake;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class Place3RightSide implements AutoBase {
    @Override
    public AutoRoutine getAuto() {
        AutoRoutine routine = drive.autoFactory.newRoutine("Place3RightSide");

        Transform2d place1Offset = new Transform2d(new Translation2d(5.5,2), new Rotation2d());
        Transform2d place2Offset = new Transform2d(new Translation2d(3.5, 1), new Rotation2d());
        Transform2d place3Offset = new Transform2d(new Translation2d(3.5, 1), new Rotation2d());
        
        var place1 = routine.trajectory("Place 3 right side", 0);
        var pickup2 = routine.trajectory("Place 3 right side", 1);
        var place3 = routine.trajectory("Place 3 right side", 2);
        var pickup4 = routine.trajectory("Place 3 right side", 3);
        var place5 = routine.trajectory("Place 3 right side", 4);

        routine.active().onTrue(
                Commands.sequence(
                        place1.resetOdometry(),
                        Commands.runOnce(() -> drive.orientModules(Drive.getStraightOrientations()), drive),
                        elevator.resetPosition(),
                        Commands.parallel(
                                place1.cmd(),
                                Commands.sequence(
                                    elevator.runOnce(() -> elevator.setPos(ElevatorConstants.tuckPos)),
                                    Commands.waitSeconds(0.25),
                                    actuation.runOnce(() -> actuation.setPos(ActuationConstants.tuckPos))
                                )
                        )));

        place1.atTime("Place Piece 1").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil.apply(Reef.placePoses[8].transformBy(place1Offset)), 
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        new SetPosition(l4),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.waitUntil(() -> elevator.isAtHeight(l4.elevatorHeight().getAsDouble(), 0.25) || !elevator.isMoving()),
                        Commands.waitUntil(() -> actuation.isAtAngle()),
                        intake.runOnce(() -> intake.runVelocity(outtakeVelocity.get())),
                        Commands.waitUntil(() -> false).withTimeout(0.25),
                        intake.runOnce(() -> intake.runVelocity(intakeVelocity.get())),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.parallel(
                            new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                            pickup2.cmd()
                        )));

        place3.atTime("Place Piece 2").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil.apply(Reef.placePoses[10].transformBy(place2Offset)), 
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        new SetPosition(l4),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.waitUntil(() -> elevator.isAtHeight(l4.elevatorHeight().getAsDouble(), 0.25) || !elevator.isMoving()),
                        Commands.waitUntil(() -> actuation.isAtAngle()),
                        intake.runOnce(() -> intake.runVelocity(outtakeVelocity.get())),
                        Commands.waitUntil(() -> false).withTimeout(0.25),
                        intake.runOnce(() -> intake.runVelocity(intakeVelocity.get())),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.parallel(
                            new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                            pickup4.cmd()
                            // Commands.runOnce(() -> System.out.println(DriverStation.getMatchTime()))
                        )));

        place5.atTime("Place Piece 3").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil.apply(Reef.placePoses[11].transformBy(place3Offset)), 
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        new SetPosition(l4),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.waitUntil(() -> elevator.isAtHeight(l4.elevatorHeight().getAsDouble(), 0.25) || !elevator.isMoving()),
                        Commands.waitUntil(() -> actuation.isAtAngle()),
                        intake.runOnce(() -> intake.runVelocity(outtakeVelocity.get())),
                        Commands.waitUntil(() -> false).withTimeout(0.25),
                        new TuckCommand(),
                        intake.runOnce(() -> intake.runVelocity(0)),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.runOnce(() -> System.out.println(DriverStation.getMatchTime()))));

        pickup2.atTime("pickup 1").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil.apply(CoralStations.pickupLocations[16]),
                                        () -> new Translation2d(),
                                        allignmentMode.NORMAL),
                                drive),
                        new SetPosition(ElevatorConstants.pickUpPos, ActuationConstants.pickUpPos),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted() || RobotState.getInstance().hasCoral()),
                        Commands.waitUntil(() -> elevator.isAtHeight(ElevatorConstants.pickUpPos.get(), 0.25)),
                        intake.runOnce(() -> intake.runVelocity(intakeVelocity.get())),
                        Commands.waitUntil(RobotState.getInstance()::hasCoral),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.parallel(
                            place3.cmd(),   
                            new TuckCommand().beforeStarting(Commands.waitSeconds(0.15))
                        )));

        pickup4.atTime("pickup 2").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil.apply(CoralStations.pickupLocations[16]),
                                        () -> new Translation2d(),
                                        allignmentMode.NORMAL),
                                drive),
                        new SetPosition(ElevatorConstants.pickUpPos, ActuationConstants.pickUpPos),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted() || RobotState.getInstance().hasCoral()),
                        Commands.waitUntil(() -> elevator.isAtHeight(ElevatorConstants.pickUpPos.get(), 0.25)),
                        intake.runOnce(() -> intake.runVelocity(intakeVelocity.get())),
                        Commands.waitUntil(RobotState.getInstance()::hasCoral),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.parallel(
                            place5.cmd(),   
                            new TuckCommand().beforeStarting(Commands.waitSeconds(0.15))
                        )));

        return routine;
    }
}
