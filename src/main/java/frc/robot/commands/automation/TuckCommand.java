package frc.robot.commands.automation;

import static frc.robot.Subsystems.coralActuation;
import static frc.robot.Subsystems.elevator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralActuation.CoralActuationConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class TuckCommand extends SequentialCommandGroup {

    public TuckCommand() {
        addCommands(
            elevator.runOnce(() -> elevator.setPos(ElevatorConstants.tuckPos)),
            coralActuation.runOnce(() -> coralActuation.setPos(CoralActuationConstants.tuckPos))
        );
    }
}
