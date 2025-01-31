package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    private double position = startPos;
    private boolean stopped = false;
    private double velocity = 0;
    private boolean isPositionControl = false;

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);

        if (stopped == true) {
            elevatorIO.stopMotors();
            return;
        }

        if (isPositionControl == true) {
            elevatorIO.moveToPos(position);
        } else {
            // elevatorIO.runVelocity(velocity);
            elevatorIO.runVoltage(velocity);
        }
    }

    public void setPos(double pos) {
        stopped = false;
        isPositionControl = true;
        position = pos;
    }
    
    public void stop() {
        stopped = true;
    }

    public void runVelocity(double vel) {
        stopped = false;
        isPositionControl = false;
        velocity = vel;
    }
}
