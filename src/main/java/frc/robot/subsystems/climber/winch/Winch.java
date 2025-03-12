package frc.robot.subsystems.climber.winch;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The `Climber` class represents a subsystem that controls the climber
 * mechanism of the robot. It
 * provides methods to control the climber motor, set the position of the
 * climber, and run the
 * climber at a specified voltage.
 */
public class Winch extends SubsystemBase {
    private final WinchIO winchIO;
    private final WinchIOInputsAutoLogged winchInputs = new WinchIOInputsAutoLogged();

    private double voltage = 0;

    public Winch(WinchIO winchIO) {
        this.winchIO = winchIO;
    }

    @Override
    public void periodic() {
        winchIO.updateInputs(winchInputs);
        Logger.processInputs("Winch", winchInputs);

        if (voltage != 0) {
            winchIO.runClimberMotors(voltage);
        } else {
            winchIO.stopClimber();
        }
    }

    public void runVoltage(double volts) {
        voltage = volts;
    }

    public Command runVoltageCommand(double volts) {
        return Commands.startEnd(() -> runVoltage(volts), () -> runVoltage(0), this);
    }

}
