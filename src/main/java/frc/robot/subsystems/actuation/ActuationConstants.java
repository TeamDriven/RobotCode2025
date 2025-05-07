package frc.robot.subsystems.actuation;

import frc.robot.util.LoggedTunableNumber;

public class ActuationConstants {
    public static final double gearRatio = (32 / 16) * (9) * (7);

    public static final double offset = 4.112;

    public static final double startPos = 0;

    public static final double tuckPos = 40;
    public static final LoggedTunableNumber L4Pos = new LoggedTunableNumber("Actuation/L4", -32.5); 
    public static final LoggedTunableNumber pickUpPos = new LoggedTunableNumber("Actuation/PickUp", -20);
    public static final LoggedTunableNumber bargePos = new LoggedTunableNumber("Actuation/Barge", 70); 
    public static final LoggedTunableNumber processorPos = new LoggedTunableNumber("Actuation/Processor", 0); 
    public static final LoggedTunableNumber turtlePos = new LoggedTunableNumber("Actuation/Turtle", 70); 
    public static final LoggedTunableNumber dealgifyPos = new LoggedTunableNumber("Actuation/dealgifyPos", -10);

    public static final LoggedTunableNumber actuationTuningVoltage = new LoggedTunableNumber("Actuation/tuningVoltage", 1);
}
