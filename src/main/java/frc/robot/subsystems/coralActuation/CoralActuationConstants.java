package frc.robot.subsystems.coralActuation;

import frc.robot.util.LoggedTunableNumber;

public class CoralActuationConstants {
    public static final double gearRatio = 5 * 7 * (32.0 / 18);

    public static final double rotationsToDegrees = 360 * gearRatio;

    public static final double offset = 121.7;

    public static final double startPos = 0;

    public static final LoggedTunableNumber coralActuationTuningVoltage = new LoggedTunableNumber("CoralActuation/tuningVoltage", 1);
}
