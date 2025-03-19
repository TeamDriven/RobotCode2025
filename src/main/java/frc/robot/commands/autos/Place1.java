package frc.robot.commands.autos;

import static frc.robot.Constants.l4;
import static frc.robot.subsystems.intake.IntakeConstants.outtakeVelocity;
import static frc.robot.Subsystems.actuation;
import static frc.robot.Subsystems.drive;
import static frc.robot.Subsystems.elevator;
import static frc.robot.Subsystems.intake;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.automation.SetPosition;
import frc.robot.commands.automation.TuckCommand;
import frc.robot.subsystems.actuation.ActuationConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class Place1 implements AutoBase {
    @Override
    public AutoRoutine getAuto() {
        AutoRoutine routine = drive.autoFactory.newRoutine("Place1");

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
                                        actuation.runOnce(() -> actuation.setPos(ActuationConstants.tuckPos))))));

        place1.atTime("Place Piece 1").onTrue(
                Commands.sequence(
                        new SetPosition(l4)));

        place1.done().onTrue(
                Commands.sequence(
                        Commands.waitUntil(
                                () -> elevator.isAtHeight(l4.elevatorHeight(), 0.25) || !elevator.isMoving()),
                        Commands.waitUntil(() -> actuation.isAtAngle()),
                        intake.runOnce(() -> intake.runVelocity(outtakeVelocity.get())),
                        Commands.waitUntil(() -> false).withTimeout(0.25),
                        new TuckCommand(),
                        drive.runOnce(() -> drive.acceptSimpleInput(0, 0, 0, false))));

        return routine;
    }
}
