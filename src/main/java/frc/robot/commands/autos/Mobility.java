package frc.robot.commands.autos;

import static frc.robot.Subsystems.actuation;
import static frc.robot.Subsystems.drive;
import static frc.robot.Subsystems.elevator;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.actuation.ActuationConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class Mobility implements AutoBase {
    @Override
    public AutoRoutine getAuto() {
        AutoRoutine routine = drive.autoFactory.newRoutine("Mobility");

        var place1 = routine.trajectory("Place 1", 0);

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

        return routine;
    }
}
