package frc.robot.subsystems.climber;

import frc.robot.util.LoggedTunableNumber;

public class ClimberConstants {
  public static final double upPos = 0;
  public static final double downPos = 0;

  public static final LoggedTunableNumber climberTuningVoltage = new LoggedTunableNumber("Climber/tuningVoltage", 12);
}
