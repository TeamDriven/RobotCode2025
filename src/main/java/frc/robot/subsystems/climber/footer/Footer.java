package frc.robot.subsystems.climber.footer;

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
public class Footer extends SubsystemBase {
    private final FooterIO footerIO;
    private final FooterIOInputsAutoLogged footerInputs = new FooterIOInputsAutoLogged();

    private double voltage = 0;
    private double position = 0;

    // TODO: 0.2 volts in most of match

    public Footer(FooterIO footerIO) {
        this.footerIO = footerIO;
    }

    @Override
    public void periodic() {
        footerIO.updateInputs(footerInputs);
        Logger.processInputs("Footer", footerInputs);

        if (voltage != 0 && position == 0) {
            footerIO.runVoltage(voltage);
        } else if (voltage == 0 && position != 0) {
            footerIO.moveToPos(position);
        } else {
            footerIO.stopFooter();
        }
    }

    public void runVoltage(double volts) {
        position = 0;
        voltage = volts;
    }

    public Command runVoltageCommand(double volts) {
        return Commands.startEnd(() -> runVoltage(volts), () -> runVoltage(0), this);
    }

    public void setPos(double pos) {
        voltage = 0;
        position = pos;
    }

    public void stop() {
        voltage = 0;
        position = 0;
    }

}
