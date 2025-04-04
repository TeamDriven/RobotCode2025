package frc.robot.commands.automation;

import static frc.robot.Subsystems.actuation;
import static frc.robot.Subsystems.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.setLevel;

public class SetPosition extends SequentialCommandGroup {

    // public SetPosition(double elevatorHeight, double movementAngle, double
    // finalAngle) {
    // addCommands(
    // elevator.runOnce(() -> elevator.setPos(elevatorHeight)),
    // actuation.runOnce(() -> actuation.setPos(movementAngle)),
    // Commands.waitUntil(() -> elevator.isAtHeight(elevatorHeight, 2)),
    // actuation.runOnce(() -> actuation.setPos(finalAngle))
    // );
    // }

    public SetPosition(double elevatorHeight, double angle) {
        addCommands(
                elevator.runOnce(() -> elevator.setPos(elevatorHeight)),
                actuation.runOnce(() -> actuation.setPos(angle)));
    }

    public SetPosition(DoubleSupplier elevatorHeight, DoubleSupplier angle) {
        addCommands(elevator.runOnce(() -> elevator.setPos(elevatorHeight.getAsDouble())),
                actuation.runOnce(() -> actuation.setPos(angle.getAsDouble())));
    }

    public SetPosition(setLevel level) {
        this(level.elevatorHeight(), level.angle());
    }

    // public SetPosition(placeLevel level, double movementAngle) {
    // this(level.elevatorHeight(), movementAngle, level.angle());
    // }
}
