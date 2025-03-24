package frc.robot.subsystems.actuation;

import frc.robot.util.LoggedTunableNumber;

public class ActuationConstants {
    public static final double gearRatio = 5 * 7 * (36.0 / 15);

    public static final double offset = 0.619;

    public static final double startPos = 0;

    public static final double tuckPos = 0;
    public static final LoggedTunableNumber pickUpPos = new LoggedTunableNumber("Actuation/PickUp", 34.75);
    public static final LoggedTunableNumber L1Pos = new LoggedTunableNumber("Actuation/L1", 8);
    public static final LoggedTunableNumber L2Pos = new LoggedTunableNumber("Actuation/L2", -17.5);
    public static final LoggedTunableNumber L3Pos = new LoggedTunableNumber("Actuation/L3", -20); 
    public static final LoggedTunableNumber L4Pos = new LoggedTunableNumber("Actuation/L4", -32.5);  
    public static final double dealgifyPos = 0;
    public static final double processorPos = 0;

    public static final LoggedTunableNumber actuationTuningVoltage = new LoggedTunableNumber("Actuation/tuningVoltage", 1);
}
