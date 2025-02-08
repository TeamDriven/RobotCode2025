package frc.robot.commands.autos;

import static frc.robot.Subsystems.drive;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;

public class Place4RightSideForReal implements AutoBase{
    @Override
    public AutoRoutine getAuto() {
        AutoRoutine routine = drive.autoFactory.newRoutine("Place4RightSideForReal");

        var place1 = routine.trajectory("place 4 right for real", 0);
        var pickup2 = routine.trajectory("place 4 right for real", 1);

        routine.active().onTrue(
            Commands.sequence(
                place1.resetOdometry(),
                Commands.runOnce(() -> drive.orientModules(Drive.getStraightOrientations()), drive),
                place1.cmd()
            )
        );

        place1.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        pickup2.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));

        place1.done().onTrue(pickup2.cmd().beforeStarting(new WaitCommand(1)));

        return routine;
    }
}
