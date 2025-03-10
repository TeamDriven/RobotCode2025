package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Intake extends SubsystemBase {
    private IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private enum mode {
        VELOCITY,
        VOLTAGE;
    }

    private mode currentMode = mode.VELOCITY;

    private double value = 0;

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Should report to RobotState when piece status changes
        RobotState.getInstance().setGamePiece(inputs.gamePieceSensor);

        if (value == 0) {
            intakeIO.stopMotor();
        } else if (currentMode == mode.VELOCITY) {
            if (RobotState.getInstance().hasCoral() && Math.abs(inputs.motorVel) < 5) {
                value = Math.max(value, 0);
            }
            intakeIO.runMotor(value);
        } else if (currentMode == mode.VOLTAGE) {
            intakeIO.runMotor(value);
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
