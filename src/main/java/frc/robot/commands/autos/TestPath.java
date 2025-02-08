package frc.robot.commands.autos;

import static frc.robot.Subsystems.drive;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;

public class TestPath implements AutoBase{
    @Override
    public AutoRoutine getAuto() {
        AutoRoutine routine = drive.autoFactory.newRoutine("TestPath");

        var path1 = routine.trajectory("TestPath", 0);
        var path2 = routine.trajectory("TestPath", 1);

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> drive.orientModules(Drive.getStraightOrientations()), drive),
                path1.cmd()
            )
        );

        path1.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));
        path2.done().onTrue(Commands.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false), drive));

        path1.done().onTrue(path2.cmd().beforeStarting(new WaitCommand(1)));

        return routine;
    }
}
