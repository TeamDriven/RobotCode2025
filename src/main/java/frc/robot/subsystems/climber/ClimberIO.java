package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double position;
    public double velocity;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void runClimberMotors(double velocity) {}

  default void stopClimber() {}
}
