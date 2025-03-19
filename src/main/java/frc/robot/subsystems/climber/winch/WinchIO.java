package frc.robot.subsystems.climber.winch;

import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
  @AutoLog
  class WinchIOInputs {
    public double topMotorPos = 0;
    public double topMotorVoltage = 0;
    public double topMotorCurrent = 0;
    public double topTorqueCurrent = 0;
    public double topStallCurrent = 0; //find out what this is then check back

    public double bottomMotorPos = 0;
    public double bottomMotorVoltage = 0;
    public double bottomMotorCurrent = 0;
    public double bottomTorqueCurrent = 0;
    public double bottomStallCurrent = 0; //find out what this is then check back
  }

  default void updateInputs(WinchIOInputs inputs) {}

  default void runClimberMotors(double velocity) {}

  default void stopClimber() {}
}
