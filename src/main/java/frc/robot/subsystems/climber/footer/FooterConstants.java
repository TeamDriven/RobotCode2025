package frc.robot.subsystems.climber.footer;

import frc.robot.util.LoggedTunableNumber;

public class FooterConstants {
  public static final double upPos = 0;
  public static final double downPos = 110;

  public static final double gearRatio = (60 / 14);

  public static final LoggedTunableNumber footerTuningVoltage = new LoggedTunableNumber("Climber/Footer/tuningVoltage", 12);
}
