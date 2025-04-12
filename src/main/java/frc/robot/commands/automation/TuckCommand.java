package frc.robot.commands.automation;

import static frc.robot.Subsystems.actuation;
import static frc.robot.Subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.actuation.ActuationConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class TuckCommand extends ConditionalCommand {

    public TuckCommand() {
        // addCommands(
        // actuation.runOnce(() -> actuation.setPos(ActuationConstants.tuckPos)),
        // Commands.waitSeconds(0.25),
        // elevator.runOnce(() -> elevator.setPos(ElevatorConstants.tuckPos))
        // );
        super(
                Commands.sequence(
                        actuation.runOnce(() -> actuation.setPos(ActuationConstants.tuckPos)),
                        Commands.waitSeconds(0.25),
                        elevator.runOnce(() -> elevator.setPos(ElevatorConstants.tuckPos))),
                Commands.sequence(
                        actuation.runOnce(() -> actuation.setPos(ActuationConstants.tuckPos)),
                        elevator.runOnce(() -> elevator.setPos(ElevatorConstants.tuckPos))),
                RobotState.getInstance()::hasAlgae);
    }
}
