package frc.robot.subsystems.climber.winch;

import frc.robot.util.LoggedTunableNumber;

public class WinchConstants {
  public static final double upPos = 0;
  public static final double downPos = 0;

  public static final LoggedTunableNumber winchTuningVoltage = new LoggedTunableNumber("Climber/Winch/tuningVoltage", 12);
}
