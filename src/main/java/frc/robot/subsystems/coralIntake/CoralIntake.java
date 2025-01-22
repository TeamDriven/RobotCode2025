package frc.robot.subsystems.coralIntake;

import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.detectionCurrent;

import org.littletonrobotics.junction.Logger;

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
        if (velocity > 0 && inputs.supplyCurrentAmps < detectionCurrent) {
            coralIntakeIO.runMotor(velocity);
        } else if (velocity < 0) {
            coralIntakeIO.runMotor(velocity);
        } else {
            coralIntakeIO.stopMotor();
        }
    }

    public void setMotorVelocity(double velocity) {
        this.velocity = velocity;
    }
}
