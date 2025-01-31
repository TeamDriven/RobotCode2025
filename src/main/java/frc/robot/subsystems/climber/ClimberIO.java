package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double motorPos = 0;
    public double motorVoltage = 0;
    public double motorCurrent = 0;
    public double torqueCurrent = 0;
    public double stallCurrent = 0; //find out what this is then check back
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void runClimberMotors(double velocity) {}

  default void stopClimber() {}
}
