package frc.robot.commands.automation;

import static frc.robot.Subsystems.coralActuation;
import static frc.robot.Subsystems.elevator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.placeLevel;

public class PrepareForPlaceCoral extends SequentialCommandGroup {

    public PrepareForPlaceCoral(double elevatorHeight, double angle) {
        addCommands(
            elevator.run(() -> elevator.setPos(elevatorHeight)),
            coralActuation.run(() -> coralActuation.setPos(angle))
        );
    }

    public PrepareForPlaceCoral(placeLevel level) {
        this(level.elevatorHeight(), level.angle());
    }
}
