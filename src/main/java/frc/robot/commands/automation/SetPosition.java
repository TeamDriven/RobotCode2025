package frc.robot.commands.automation;

import static frc.robot.Subsystems.coralActuation;
import static frc.robot.Subsystems.elevator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.placeLevel;

public class SetPosition extends SequentialCommandGroup {

    public SetPosition(double elevatorHeight, double angle) {
        addCommands(
            elevator.runOnce(() -> elevator.setPos(elevatorHeight)),
            coralActuation.runOnce(() -> coralActuation.setPos(angle))
        );
    }

    public SetPosition(placeLevel level) {
        this(level.elevatorHeight(), level.angle());
    }
}
