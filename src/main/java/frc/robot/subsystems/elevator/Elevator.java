package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.tuckPos;
import static frc.robot.subsystems.elevator.ElevatorConstants.maxStableVelocity;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase{
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/posPID/kP", 6.5);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/posPID/kI", 0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/posPID/kD", 0.02);
    private final LoggedTunableNumber tolerance = new LoggedTunableNumber("Elevator/posPID/tolerance", 2.5);
    
    private PIDController posPid;

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
        posPid = new PIDController(kP.get(), kI.get(), kD.get());
        posPid.setTolerance(tolerance.get());
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);

        Logger.recordOutput("Elevator/mode", currentMode);
        Logger.recordOutput("Elevator/value", value);

        LoggedTunableNumber.ifChanged(hashCode(), this::setUpPID, kP, kI, kD, tolerance);

        switch (currentMode) {
            case POSITION:
                double velocity = MathUtil.clamp(posPid.calculate(elevatorInputs.leftMotorPos, value), -maxStableVelocity, maxStableVelocity);
                Logger.recordOutput("Elevator/posVelocity", velocity);

                if (posPid.atSetpoint()) {
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
        return new Command() {
            @Override
            public void execute() {
                runVoltage(volts.getAsDouble());
            }

            @Override
            public void end(boolean isInterrupted) {
                stop();
            }
        };
    }

    public Command resetPosition() {
        return new Command() {
            Timer timer = new Timer();

            @Override
            public void initialize() {
                timer.restart();
            }

            @Override
            public void execute() {
                runVoltage(-3);
            }

            @Override
            public void end(boolean interrupted) {
                timer.stop();
                elevatorIO.stopMotors();
                elevatorIO.resetPosition();
                stop();
                // setPos(tuckPos);
            }

            @Override
            public boolean isFinished() {
                return timer.hasElapsed(0.2) && !isMoving();
            }
        };
    }

    public void resetPos() {
        elevatorIO.resetPosition();
    }

    public boolean isAtHeight(double height, double tolerance) {
        Logger.recordOutput("Elevator/isAtHeight", MathUtil.isNear(height, elevatorInputs.leftMotorPos, tolerance));
        return MathUtil.isNear(height, elevatorInputs.leftMotorPos, tolerance);
    }

    public boolean isMoving() {
        Logger.recordOutput("Elevator/isMoving", Math.abs(elevatorInputs.leftMotorVel) > 1.5);
        return Math.abs(elevatorInputs.leftMotorVel) > 1.5;
    }
}
