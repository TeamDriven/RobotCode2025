package frc.robot.commands.autos;

import static frc.robot.Subsystems.drive;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;

public class Place2RightSide implements AutoBase{
    @Override
    public AutoRoutine getAuto() {
        AutoRoutine routine = drive.autoFactory.newRoutine("Place2RightSide");

        var place1 = routine.trajectory("Place 2 right side", 0);
        var pickup2 = routine.trajectory("Place 2 right side", 1);
        var place3 = routine.trajectory("Place 2 right side", 2);

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

        place1.done().onTrue(pickup2.cmd().beforeStarting(new WaitCommand(1)));
        
        pickup2.done().onTrue(place3.cmd().beforeStarting(new WaitCommand(1)));

        return routine;
    }
}
