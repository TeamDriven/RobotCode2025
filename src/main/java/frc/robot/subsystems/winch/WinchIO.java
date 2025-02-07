package frc.robot.subsystems.winch;

import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
  @AutoLog
  class WinchIOInputs {
    public double leftMotorPos = 0;
    public double leftMotorVoltage = 0;
    public double leftMotorCurrent = 0;
    public double leftTorqueCurrent = 0;
    public double leftStallCurrent = 0; //find out what this is then check back

    public double rightMotorPos = 0;
    public double rightMotorVoltage = 0;
    public double rightMotorCurrent = 0;
    public double rightTorqueCurrent = 0;
    public double rightStallCurrent = 0; //find out what this is then check back
  }

  default void updateInputs(WinchIOInputs inputs) {}

  default void runWinchMotors(double velocity) {}

  default void stopWinch() {}
}
