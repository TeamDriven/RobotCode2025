package frc.robot.commands.autos;

import static frc.robot.Subsystems.drive;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

public class TestAuto implements AutoBase {
    @Override
    public AutoRoutine getAuto() {
        AutoRoutine routine = drive.autoFactory.newRoutine("TestAuto");

        // TODO: check out split index
        var test1 = routine.trajectory("test1");
        var test2 = routine.trajectory("test2");

        // When the routine begins, reset odometry and start the first trajectory 
        routine.active().onTrue(
            Commands.sequence(
                test1.resetOdometry(),
                test1.cmd()
            )
        );

        test1.atTime(0.1).onTrue(new PrintCommand("0.1 has passed"));

        test1.done().onTrue(test2.cmd());

        test2.active().whileTrue(new RepeatCommand(new PrintCommand("Test2 is running")));

        test2.done().onTrue(Commands.runOnce(() -> {drive.acceptSimpleInput(0, 0, 0, false);}));

        return routine;
    }
}
