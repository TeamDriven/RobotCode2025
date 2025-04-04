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

        Transform2d coral1Offset = new Transform2d(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
                new Rotation2d());
        Transform2d pickup1Offset = new Transform2d(new Translation2d(Units.inchesToMeters(-15), Units.inchesToMeters(0)),
                new Rotation2d());
        Transform2d pickup2Offset = new Transform2d(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
                new Rotation2d());
        Transform2d pickup3Offset = new Transform2d(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
                new Rotation2d());
        Transform2d pickup4Offset = new Transform2d(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
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
        
        // pickup1.done().onTrue(place1.cmd());
        // place1.done().onTrue(pickup2.cmd());
        // pickup2.done().onTrue(place2.cmd());
        // place2.done().onTrue(pickup3.cmd());
        // pickup3.done().onTrue(place3.cmd());
        // place3.done().onTrue(pickup4.cmd());
        // pickup4.done().onTrue(place4.cmd());

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
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        // // Pickup Algae
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil.apply(Reef.reefFaces[3].facePos().transformBy(pickup1Offset)),
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.waitSeconds(waitTimePickup1),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                place1.cmd())));

        place1.atTime("place Algae 1").onTrue(
                Commands.sequence(
                        Commands.waitSeconds(waitTimePlace1),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                pickup2.cmd())));

        pickup2.atTime("Pick up Algae 2").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil.apply(Reef.reefFaces[4].facePos().transformBy(pickup2Offset)),
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.waitSeconds(waitTimePickup2),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                place2.cmd())));

        place2.atTime("place Algae 2").onTrue(
                Commands.sequence(
                        Commands.waitSeconds(waitTimePlace2),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                pickup3.cmd())));

        pickup3.atTime("Pick up Algae 3").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil.apply(Reef.reefFaces[2].facePos().transformBy(pickup3Offset)),
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.waitSeconds(waitTimePickup3),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                place3.cmd())));

        place3.atTime("place Algae 3").onTrue(
                Commands.sequence(
                        Commands.waitSeconds(waitTimePlace3),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                pickup4.cmd())));

        pickup4.atTime("Pick up Algae 4").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(
                                        () -> AllianceFlipUtil.apply(Reef.reefFaces[1].facePos().transformBy(pickup4Offset)),
                                        () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        new WaitUntilCommand(() -> drive.isAutoAlignGoalCompleted()),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal(), drive),
                        Commands.waitSeconds(waitTimePickup4),
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                place4.cmd())));

        place4.atTime("place Algae 4").onTrue(
                Commands.sequence(
                        Commands.parallel(
                                // new TuckCommand().beforeStarting(Commands.waitSeconds(0.15)),
                                Commands.none())));

        return routine;
    }
}
