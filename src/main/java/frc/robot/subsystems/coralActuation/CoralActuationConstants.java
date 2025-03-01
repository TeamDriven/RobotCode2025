package frc.robot.subsystems.coralActuation;

import frc.robot.util.LoggedTunableNumber;

public class CoralActuationConstants {
    public static final double gearRatio = 5 * 7 * (32.0 / 18);

    public static final double offset = 211.0;

    public static final double startPos = 0;

    public static final double tuckPos = -5;
    public static final double pickUpPos = 35;
    public static final double L1Pos = 0;
    public static final double L2Pos = -20;
    public static final double L3Pos = -20;
    public static final double L4Pos = -25;
    public static final double dealgifyPos = -15;

    public static final LoggedTunableNumber coralActuationTuningVoltage = new LoggedTunableNumber("CoralActuation/tuningVoltage", 1);
}
