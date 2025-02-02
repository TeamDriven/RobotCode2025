package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
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

  default void updateInputs(ClimberIOInputs inputs) {}

  default void runClimberMotors(double velocity) {}

  default void stopClimber() {}
}
