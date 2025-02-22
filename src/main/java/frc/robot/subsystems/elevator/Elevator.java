package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    // TODO: remove voltage control
    private enum mode {
        POSITION,
        VELOCITY,
        VOLTAGE,
        STOPPED;
    }

    private mode currentMode = mode.STOPPED;

    private double value = 0;

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);

        Logger.recordOutput("Elevator/mode", currentMode);
        Logger.recordOutput("Elevator/value", value);

        switch (currentMode) {
            case POSITION:
                elevatorIO.moveToPos(value);
                break;
        
            case VELOCITY:
                elevatorIO.runVelocity(value);
                break;

            case VOLTAGE:
                elevatorIO.runVoltage(value);
                break;
            
            case STOPPED:
                elevatorIO.stopMotors();
                break;
        }
    }

    public void setPos(double pos) {
        currentMode = mode.POSITION;
        value = pos;
    }
    
    public void stop() {
        currentMode = mode.STOPPED;
        value = 0;
    }

    public void runVelocity(double vel) {
        currentMode = mode.VELOCITY;
        value = vel;
    }

    public void runVoltage(double volts) {
        currentMode = mode.VOLTAGE;
        value = volts;
    }

    public Command runVoltageCommand(double volts) {
        return Commands.startEnd(() -> runVoltage(volts), () -> stop(), this);
    }

    public Command runVoltageCommand(DoubleSupplier volts) {
        return Commands.startEnd(() -> runVoltage(volts.getAsDouble()), () -> stop(), this);
    }
}
