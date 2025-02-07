package frc.robot.subsystems.winch;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The `Climber` class represents a subsystem that controls the climber mechanism of the robot. It
 * provides methods to control the climber motor, set the position of the climber, and run the
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
  
    if(voltage != 0) {
      winchIO.runWinchMotors(voltage);
    } else {
      winchIO.stopWinch();
    }
  }

  public void runVoltage(double volts) {
    voltage = volts;
  }

}
