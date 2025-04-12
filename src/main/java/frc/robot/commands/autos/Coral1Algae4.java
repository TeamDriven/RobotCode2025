package frc.robot.commands.autos;

import static frc.robot.Constants.barge;
import static frc.robot.Constants.highAlgae;
import static frc.robot.Constants.l4;
import static frc.robot.Constants.lowAlgae;
import static frc.robot.subsystems.intake.IntakeConstants.intakeVelocity;
import static frc.robot.subsystems.intake.IntakeConstants.outtakeVelocity;
import static frc.robot.Subsystems.actuation;
import static frc.robot.Subsystems.drive;
import static frc.robot.Subsystems.elevator;
import static frc.robot.Subsystems.intake;
import static frc.robot.subsystems.actuation.ActuationConstants.L4Pos;
import static frc.robot.subsystems.actuation.ActuationConstants.tuckPos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants.CoralStations;
import frc.robot.FieldConstants.Reef;
import frc.robot.commands.automation.SetPosition;
import frc.robot.commands.automation.TuckCommand;
import frc.robot.subsystems.actuation.ActuationConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.AutoAlignController.allignmentMode;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.RobotState;

public class Coral1Algae4 implements AutoBase {
    @Override
    public AutoRoutine getAuto() {
        AutoRoutine routine = drive.autoFactory.newRoutine("Coral1Algae4");

        Transform2d coral1Offset = new Transform2d(
                new Translation2d(Units.inchesToMeters(-15), Units.inchesToMeters(-1)),
                new Rotation2d());
        Transform2d pickup1Offset = new Transform2d(
                new Translation2d(Units.inchesToMeters(-15), Units.inchesToMeters(0)),
                new Rotation2d());
        Transform2d pickup2Offset = new Transform2d(
                new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(-7.5)),
                new Rotation2d());
        Transform2d pickup3Offset = new Transform2d(
                new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(7.5)),
                new Rotation2d());
        Transform2d pickup4Offset = new Transform2d(
                new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(7.5)),
                new Rotation2d());

        double waitTimePickup1 = 0.5;
        double waitTimePlace1 = 0.5;
        double waitTimePickup2 = 0.5;
        double waitTimePlace2 = 0.5;
        double waitTimePickup3 = 0.5;
        double waitTimePlace3 = 0.5;
        double waitTimePickup4 = 0.5;

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
                                // elevator.runOnce(() -> elevator.setPos(ElevatorConstants.tuckPos)),
                                // Commands.waitSeconds(0.25),
                                // actuation.runOnce(() -> actuation.setPos(ActuationConstants.tuckPos))
                                ))));

        // Commands.runOnce(() -> System.out.println(DriverStation.getMatchTime()))));
        pickup1.atTime("Pick up Algae 1").onTrue(
                Commands.sequence(
                        // // Place Coral
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil.apply(Reef.placePoses[5].transformBy(coral1Offset)),
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        new SetPosition(l4),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.waitUntil(() -> elevator.isAtHeight(l4.elevatorHeight().getAsDouble(), 0.25)
                                || !elevator.isMoving()),
                        Commands.waitUntil(() -> actuation.isAtAngle()),
                        intake.runOnce(() -> intake.runVelocity(outtakeVelocity.get())),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.runOnce(() -> System.out.println("Place Coral 1: " + DriverStation.getMatchTime())),
                        // // Pickup Algae
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil
                                                .apply(Reef.reefFaces[3].facePos().transformBy(pickup1Offset)),
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        new SetPosition(lowAlgae),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.waitUntil(() -> elevator.isAtHeight(lowAlgae.elevatorHeight().getAsDouble(), 0.25)
                                || !elevator.isMoving()),
                        Commands.waitUntil(() -> actuation.isAtAngle()),
                        intake.runOnce(() -> intake.runVelocity(intakeVelocity.get())),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.runOnce(() -> System.out.println("Pickup Algae 1: " + DriverStation.getMatchTime())),
                        Commands.waitSeconds(waitTimePickup1),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                place1.cmd())));

        place1.atTime("place Algae 1").onTrue(
                Commands.sequence(
                        elevator.runOnce(() -> elevator.setPos(barge.elevatorHeight().getAsDouble())),
                        Commands.waitUntil(() -> elevator.isAtHeight(barge.elevatorHeight().getAsDouble(), 0.25)
                                || !elevator.isMoving()),
                        actuation.runVoltageCommand(0.2),
                        Commands.waitUntil(() -> actuation.isAboveAngle(60)),
                        intake.runOnce(() -> intake.runVelocity(outtakeVelocity.get())),
                        new WaitCommand(0.1),
                        actuation.runOnce(() -> actuation.setPos(tuckPos)),
                        intake.runOnce(() -> intake.runVelocity(0)),
                        Commands.runOnce(() -> System.out.println("place Algae 1: " + DriverStation.getMatchTime())),
                        Commands.waitSeconds(waitTimePlace1),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                pickup2.cmd())));

        pickup2.atTime("Pick up Algae 2").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil
                                                .apply(Reef.reefFaces[4].facePos().transformBy(pickup1Offset)),
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        new SetPosition(highAlgae),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.waitUntil(() -> elevator.isAtHeight(highAlgae.elevatorHeight().getAsDouble(), 0.25)
                                || !elevator.isMoving()),
                        Commands.waitUntil(() -> actuation.isAtAngle()),
                        intake.runOnce(() -> intake.runVelocity(intakeVelocity.get())),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.runOnce(() -> System.out.println("Pickup Algae 2: " + DriverStation.getMatchTime())),
                        Commands.waitSeconds(waitTimePickup2),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                place2.cmd())));

        place2.atTime("place Algae 2").onTrue(
                Commands.sequence(
                        actuation.runVoltageCommand(0.2),
                        Commands.waitUntil(() -> actuation.isAboveAngle(60)),
                        intake.runOnce(() -> intake.runVelocity(outtakeVelocity.get())),
                        new WaitCommand(0.1),
                        actuation.runOnce(() -> actuation.setPos(tuckPos)),
                        intake.runOnce(() -> intake.runVelocity(0)),
                        Commands.runOnce(() -> System.out.println("place Algae 2: " + DriverStation.getMatchTime())),
                        Commands.waitSeconds(waitTimePlace2),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                pickup3.cmd())));

        pickup3.atTime("Pick up Algae 3").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil
                                                .apply(Reef.reefFaces[2].facePos().transformBy(pickup1Offset)),
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        new SetPosition(highAlgae),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.waitUntil(() -> elevator.isAtHeight(highAlgae.elevatorHeight().getAsDouble(), 0.25)
                                || !elevator.isMoving()),
                        Commands.waitUntil(() -> actuation.isAtAngle()),
                        intake.runOnce(() -> intake.runVelocity(intakeVelocity.get())),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.runOnce(() -> System.out.println("Pickup Algae 3: " + DriverStation.getMatchTime())),
                        Commands.waitSeconds(waitTimePickup3),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                place3.cmd())));

        place3.atTime("place Algae 3").onTrue(
                Commands.sequence(
                        actuation.runVoltageCommand(0.2),
                        Commands.waitUntil(() -> actuation.isAboveAngle(60)),
                        intake.runOnce(() -> intake.runVelocity(outtakeVelocity.get())),
                        new WaitCommand(0.1),
                        actuation.runOnce(() -> actuation.setPos(tuckPos)),
                        intake.runOnce(() -> intake.runVelocity(0)),
                        Commands.runOnce(() -> System.out.println("place Algae 3: " + DriverStation.getMatchTime())),
                        Commands.waitSeconds(waitTimePlace3),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                pickup4.cmd())));

        pickup4.atTime("Pick up Algae 4").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil
                                                .apply(Reef.reefFaces[1].facePos().transformBy(pickup1Offset)),
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        new SetPosition(lowAlgae),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.waitUntil(() -> elevator.isAtHeight(lowAlgae.elevatorHeight().getAsDouble(), 0.25)
                                || !elevator.isMoving()),
                        Commands.waitUntil(() -> actuation.isAtAngle()),
                        intake.runOnce(() -> intake.runVelocity(intakeVelocity.get())),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.runOnce(() -> System.out.println("Pickup Algae 4: " + DriverStation.getMatchTime())),
                        Commands.waitSeconds(waitTimePickup4),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                place4.cmd())));

        place4.atTime("place Algae 4").onTrue(
                Commands.sequence(
                        actuation.runVoltageCommand(0.2),
                        Commands.waitUntil(() -> actuation.isAboveAngle(60)),
                        intake.runOnce(() -> intake.runVelocity(outtakeVelocity.get())),
                        new WaitCommand(0.1),
                        actuation.runOnce(() -> actuation.setPos(tuckPos)),
                        intake.runOnce(() -> intake.runVelocity(0)),
                        Commands.runOnce(() -> System.out.println("place Algae 4: " + DriverStation.getMatchTime()))));
        // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15))));

        return routine;
    }
}
