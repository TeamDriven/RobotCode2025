package frc.robot.commands.automation;

import static frc.robot.Subsystems.actuation;
import static frc.robot.Subsystems.intake;

import java.util.function.DoubleSupplier;

import static frc.robot.Subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.placeLevel;
import frc.robot.RobotState;
import frc.robot.subsystems.actuation.ActuationConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class PlaceCoral extends SequentialCommandGroup {

    public PlaceCoral(double elevatorHeight, double angle, double outtakeSpeed) {
        addCommands(
            Commands.runOnce(() -> elevator.setPos(elevatorHeight), elevator),
            Commands.runOnce(() -> actuation.setPos(angle), actuation),
            Commands.waitUntil(() -> elevator.isAtHeight(elevatorHeight, 0.25)),
            Commands.waitUntil(() -> actuation.isAtAngle()),
            Commands.runOnce(() -> intake.runVelocity(outtakeSpeed), actuation),
            // new WaitCommand(0.15),
            Commands.waitUntil(() -> !RobotState.getInstance().hasCoral()).withTimeout(0.15),
            Commands.runOnce(() -> intake.runVelocity(0), actuation),
            Commands.runOnce(() -> elevator.setPos(ElevatorConstants.tuckPos), elevator),
            Commands.runOnce(() -> actuation.setPos(ActuationConstants.tuckPos), actuation)
        );
    }

    public PlaceCoral(DoubleSupplier elevatorHeight, DoubleSupplier angle, double outtakeSpeed) {
        addCommands(
            Commands.runOnce(() -> elevator.setPos(elevatorHeight.getAsDouble()), elevator),
            Commands.runOnce(() -> actuation.setPos(angle.getAsDouble()), actuation),
            Commands.waitUntil(() -> elevator.isAtHeight(elevatorHeight.getAsDouble(), 0.25)),
            Commands.waitUntil(() -> actuation.isAtAngle()),
            Commands.runOnce(() -> intake.runVelocity(outtakeSpeed), actuation),
            // new WaitCommand(0.15),
            Commands.waitUntil(() -> !RobotState.getInstance().hasCoral()).withTimeout(0.15),
            Commands.runOnce(() -> intake.runVelocity(0), actuation),
            Commands.runOnce(() -> elevator.setPos(ElevatorConstants.tuckPos), elevator),
            Commands.runOnce(() -> actuation.setPos(ActuationConstants.tuckPos), actuation)
        );
    }

    public PlaceCoral(placeLevel level) {
        this(level.elevatorHeight(), level.angle(), level.outtakeSpeed());
    }
}
