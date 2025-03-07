package frc.robot.subsystems.climber.footer;

import org.littletonrobotics.junction.AutoLog;

public interface FooterIO {
  @AutoLog
  class FooterIOInputs {
    public double MotorPos = 0;
    public double MotorVoltage = 0;
    public double MotorCurrent = 0;
    public double TorqueCurrent = 0;
    public double StallCurrent = 0; //find out what this is then check back
  }

  default void updateInputs(FooterIOInputs inputs) {}

  default void runVoltage(double velocity) {}

  default void moveToPos(double pos) {}

  default void stopFooter() {}
}
