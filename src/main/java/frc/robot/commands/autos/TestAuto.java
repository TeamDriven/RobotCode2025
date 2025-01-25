package frc.robot.commands.autos;

import static frc.robot.Subsystems.drive;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TestAuto implements AutoBase {
    @Override
    public AutoRoutine getAuto() {
        AutoRoutine routine = drive.autoFactory.newRoutine("TestAuto");

        // TODO: check out split index
        var test1 = routine.trajectory("TestPath");

        // When the routine begins, reset odometry and start the first trajectory 
        routine.active().onTrue(
            Commands.sequence(
                new InstantCommand(() -> Logger.recordOutput("Debug/PathActive", 1)),
                test1.resetOdometry(),
                test1.cmd()
            )
        );

        // test1.atTime(0.1).onTrue(new PrintCommand("0.1 has passed"));

        test1.done().onTrue(Commands.runOnce(() -> {drive.acceptSimpleInput(0, 0, 0, false);}));

        return routine;
    }
}
