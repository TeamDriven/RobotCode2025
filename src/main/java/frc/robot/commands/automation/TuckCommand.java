package frc.robot.commands.automation;

import static frc.robot.Subsystems.actuation;
import static frc.robot.Subsystems.elevator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.actuation.ActuationConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class TuckCommand extends SequentialCommandGroup {

    public TuckCommand() {
        addCommands(
            elevator.runOnce(() -> elevator.setPos(ElevatorConstants.tuckPos)),
            actuation.runOnce(() -> actuation.setPos(ActuationConstants.tuckPos))
        );
    }
}
