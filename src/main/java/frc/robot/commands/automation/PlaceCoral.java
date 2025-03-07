package frc.robot.commands.automation;

import static frc.robot.Subsystems.coralActuation;
import static frc.robot.Subsystems.coralIntake;
import static frc.robot.Subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.placeLevel;
import frc.robot.RobotState;
import frc.robot.subsystems.coralActuation.CoralActuationConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class PlaceCoral extends SequentialCommandGroup {

    public PlaceCoral(double elevatorHeight, double angle, double outtakeSpeed) {
        addCommands(
            Commands.runOnce(() -> elevator.setPos(elevatorHeight), elevator),
            Commands.runOnce(() -> coralActuation.setPos(angle), coralActuation),
            Commands.waitUntil(() -> elevator.isAtHeight(elevatorHeight, 0.25)),
            Commands.waitUntil(() -> coralActuation.isAtAngle()),
            Commands.runOnce(() -> coralIntake.runVelocity(outtakeSpeed), coralActuation),
            // new WaitCommand(0.15),
            Commands.waitUntil(() -> !RobotState.getInstance().hasCoral()).withTimeout(0.15),
            Commands.runOnce(() -> coralIntake.runVelocity(0), coralActuation),
            Commands.runOnce(() -> elevator.setPos(ElevatorConstants.tuckPos), elevator),
            Commands.runOnce(() -> coralActuation.setPos(CoralActuationConstants.tuckPos), coralActuation)
        );
    }

    public PlaceCoral(placeLevel level) {
        this(level.elevatorHeight(), level.angle(), level.outtakeSpeed());
    }
}
