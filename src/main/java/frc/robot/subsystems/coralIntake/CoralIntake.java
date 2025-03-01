package frc.robot.subsystems.coralIntake;

import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.detectionCurrent;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO coralIntakeIO;
    private CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();

    private enum mode {
        VELOCITY,
        VOLTAGE;
    }

    private mode currentMode = mode.VELOCITY;

    private double value = 0;

    public CoralIntake(CoralIntakeIO coralIntakeIO) {
        this.coralIntakeIO = coralIntakeIO;
    }

    @Override
    public void periodic() {
        coralIntakeIO.updateInputs(inputs);
        Logger.processInputs("CoralIntake", inputs);

        // Should report to RobotState when piece status changes

        if (value == 0) {
            coralIntakeIO.stopMotor();
        } else if (currentMode == mode.VELOCITY) {
            coralIntakeIO.runMotor(value);
        } else if (currentMode == mode.VOLTAGE) {
            coralIntakeIO.runMotor(value);
        } else {
            throw new IllegalStateException();
        }
    }

    public void runVelocity(double velocity) {
        currentMode = mode.VELOCITY;
        this.value = velocity;
    }

    public void runVoltage(double volts) {
        currentMode = mode.VOLTAGE;
        this.value = volts;
    }

    public Command runVelocityCommand(double vel) {
        return Commands.startEnd(() -> runVelocity(vel), () -> runVelocity(0), this);
    }

    public Command runVelocityCommand(DoubleSupplier vel) {
        return Commands.startEnd(() -> runVelocity(vel.getAsDouble()), () -> runVelocity(0), this);
    }
}
