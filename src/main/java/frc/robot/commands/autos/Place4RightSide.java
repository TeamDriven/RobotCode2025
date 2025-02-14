package frc.robot.commands.autos;

import static frc.robot.Subsystems.drive;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants.Reef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.AutoAlignController.allignmentMode;

public class Place4RightSide implements AutoBase {
    @Override
    public AutoRoutine getAuto() {
        AutoRoutine routine = drive.autoFactory.newRoutine("Place4RightSide");

        var place1 = routine.trajectory("place 4 right side", 0);
        var pickup2 = routine.trajectory("place 4 right side", 1);
        var place3 = routine.trajectory("place 4 right side", 2);
        var pickup4 = routine.trajectory("place 4 right side", 3);
        var place5 = routine.trajectory("place 4 right side", 4);
        var pickup6 = routine.trajectory("place 4 right side", 5);
        var place7 = routine.trajectory("place 4 right side", 6);

        routine.active().onTrue(
                Commands.sequence(
                        place1.resetOdometry(),
                        Commands.runOnce(() -> drive.orientModules(Drive.getStraightOrientations()), drive),
                        place1.cmd()));

        place1.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        pickup2.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        place3.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        pickup4.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        place5.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        pickup6.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        place7.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));

        // place1.done().onTrue(pickup2.cmd().beforeStarting(new WaitCommand(1)));
        pickup2.done().onTrue(place3.cmd().beforeStarting(new WaitCommand(0.33)));
        // place3.done().onTrue(pickup4.cmd().beforeStarting(new WaitCommand(1)));
        pickup4.done().onTrue(place5.cmd().beforeStarting(new WaitCommand(0.33)));
        // place5.done().onTrue(pickup6.cmd().beforeStarting(new WaitCommand(1)));
        pickup6.done().onTrue(place7.cmd().beforeStarting(new WaitCommand(0.33)));

        place1.atTime("Place Piece 1").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(() -> Reef.placePoses[5], () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        Commands.waitSeconds(1),
                        pickup2.cmd()));

        place3.atTime("Place Piece 2").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(() -> Reef.placePoses[3], () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        Commands.waitSeconds(1),
                        pickup4.cmd()));

        place5.atTime("Place Piece 3").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(() -> Reef.placePoses[2], () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        Commands.waitSeconds(1),
                        pickup6.cmd()));

        place7.atTime("Place Piece 4").onTrue(
                Commands.sequence(
                        Commands.runOnce(
                                () -> drive.setAutoAlignGoal(() -> Reef.placePoses[4], () -> new Translation2d(),
                                        allignmentMode.SLOW),
                                drive),
                        Commands.waitSeconds(1),
                        Commands.runOnce(() -> System.out.println(DriverStation.getMatchTime()))));

        return routine;
    }
}
