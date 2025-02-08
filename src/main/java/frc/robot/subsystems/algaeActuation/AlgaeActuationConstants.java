package frc.robot.subsystems.algaeActuation;

import frc.robot.util.LoggedTunableNumber;

public class AlgaeActuationConstants {
    public static final double gearRatio = 36;

    public static final double rotationsToDegrees = 360 * gearRatio;

    public static final double downPos = 0;
    public static final double tuckPos = 0;

    public static final LoggedTunableNumber algaeActuationVoltage = new LoggedTunableNumber("AlgaeActuation/algaeActuationVoltage", 2);
}
