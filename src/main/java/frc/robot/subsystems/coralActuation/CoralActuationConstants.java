package frc.robot.subsystems.coralActuation;

import frc.robot.util.LoggedTunableNumber;

public class CoralActuationConstants {
    public static final double gearRatio = 0;

    public static final double rotationsToDegrees = 360 * gearRatio;

    public static final double startPos = 0;

    public static final LoggedTunableNumber coralActuationTuningVoltage = new LoggedTunableNumber("CoralActuation/tuningVoltage", 4);
}
