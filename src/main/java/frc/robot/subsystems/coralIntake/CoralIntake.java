package frc.robot.subsystems.coralIntake;

import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.detectionCurrent;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO coralIntakeIO;
    private CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();

    private double velocity = 0;

    public CoralIntake(CoralIntakeIO coralIntakeIO) {
        this.coralIntakeIO = coralIntakeIO;
    }

    @Override
    public void periodic() {
        coralIntakeIO.updateInputs(inputs);
        Logger.processInputs("CoralIntake", inputs);

        // Should report to robotState when it has a piece
        if (velocity > 0 && inputs.motorCurrent < detectionCurrent) {
            coralIntakeIO.runMotor(velocity);
        } else if (velocity < 0) {
            coralIntakeIO.runMotor(velocity);
        } else {
            coralIntakeIO.stopMotor();
        }
    }

    public void runVelocity(double velocity) {
        this.velocity = velocity;
    }

    public Command runVelocityCommand(double vel) {
        return Commands.startEnd(() -> runVelocity(vel), () -> runVelocity(0), this);
    }
}
