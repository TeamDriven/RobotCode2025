package frc.robot.commands.autos;

import static frc.robot.Subsystems.drive;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;

public class Place4RightSide implements AutoBase{
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
                place1.cmd()
            )
        );

        place1.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        pickup2.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        place3.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        pickup4.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        place5.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        pickup6.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        place7.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));

        place1.done().onTrue(pickup2.cmd().beforeStarting(new WaitCommand(1)));
        pickup2.done().onTrue(place3.cmd().beforeStarting(new WaitCommand(0.33)));
        place3.done().onTrue(pickup4.cmd().beforeStarting(new WaitCommand(1)));
        pickup4.done().onTrue(place5.cmd().beforeStarting(new WaitCommand(0.33)));
        place5.done().onTrue(pickup6.cmd().beforeStarting(new WaitCommand(1)));
        pickup6.done().onTrue(place7.cmd().beforeStarting(new WaitCommand(0.33)));

        place7.done().onTrue(Commands.runOnce(() -> System.out.println(DriverStation.getMatchTime())).beforeStarting(new WaitCommand(1)));

        return routine;
    }
}
