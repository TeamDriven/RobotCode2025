package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.maxStableVelocity;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase{
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    private final LoggedTunableNumber upP = new LoggedTunableNumber("Elevator/upPID/kP", 3.3);
    private final LoggedTunableNumber upI = new LoggedTunableNumber("Elevator/upPID/kI", 0);
    private final LoggedTunableNumber upD = new LoggedTunableNumber("Elevator/upPID/kD", 0.02);
    private final LoggedTunableNumber upTolerance = new LoggedTunableNumber("Elevator/upPID/tolerance", 15);
    
    private PIDController upPid;

    private final LoggedTunableNumber downP = new LoggedTunableNumber("Elevator/downPID/kP", 3.15);
    private final LoggedTunableNumber downI = new LoggedTunableNumber("Elevator/downPID/kI", 0);
    private final LoggedTunableNumber downD = new LoggedTunableNumber("Elevator/downPID/kD", 0.02);
    private final LoggedTunableNumber downTolerance = new LoggedTunableNumber("Elevator/downPID/tolerance", 15);
    
    private PIDController downPid;

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

        setUpPID();
    }

    public void setUpPID() {
        upPid = new PIDController(upP.get(), upI.get(), upD.get());
        upPid.setTolerance(upTolerance.get());

        downPid = new PIDController(downP.get(), downI.get(), downD.get());
        downPid.setTolerance(downTolerance.get());
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);

        Logger.recordOutput("Elevator/mode", currentMode);
        Logger.recordOutput("Elevator/value", value);

        LoggedTunableNumber.ifChanged(hashCode(), this::setUpPID, upP, upI, upD, upTolerance, downP, downI, downD, downTolerance);

        switch (currentMode) {
            case POSITION:
                boolean isMovingUp = elevatorInputs.leftMotorPos < value;
                double velocity = isMovingUp
                    ? MathUtil.clamp(upPid.calculate(elevatorInputs.leftMotorPos, value), 0, maxStableVelocity)
                    : MathUtil.clamp(downPid.calculate(elevatorInputs.leftMotorPos, value), -maxStableVelocity, 0);
                Logger.recordOutput("Elevator/posVelocity", velocity);

                if ((isMovingUp && upPid.atSetpoint()) || (!isMovingUp && downPid.atSetpoint())) {
                    elevatorIO.moveToPos(value);
                } else {
                    elevatorIO.runVelocity(velocity);
                }
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

    public Command runVelocityCommand(double vel) {
        return Commands.startEnd(() -> runVelocity(vel), () -> stop(), this);
    }

    public Command runVelocityCommand(DoubleSupplier vel) {
        return Commands.startEnd(() -> runVelocity(vel.getAsDouble()), () -> stop(), this);
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

    public Command resetPosition() {
        return Commands.runOnce(elevatorIO::resetPosition, this);
    }
}
