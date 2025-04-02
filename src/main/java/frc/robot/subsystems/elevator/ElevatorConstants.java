package frc.robot.subsystems.elevator;

import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants {
    public static final double gearRatio = (50.0 / 13) * (50.0 / 30); // Is subject to change

    private static final double pulleyRadius = 1.1;
    public static final double sensorToInches = 1 / (pulleyRadius * 2 * Math.PI);

    public static final double maxStableVelocity = 75;
    public static final double maxSafeHeight = 40;
    
    public static final double bottomPos = 0;
    public static final double topPos = 63.5;

    public static final double startPos = 0;
    public static final double tuckPos = 21;
    public static final LoggedTunableNumber pickUpPos = new LoggedTunableNumber("Elevator/PickUp", 14.75);
    public static final LoggedTunableNumber L1Pos = new LoggedTunableNumber("Elevator/L1", 3.8);
    public static final LoggedTunableNumber L2Pos = new LoggedTunableNumber("Elevator/L2", 21);
    public static final LoggedTunableNumber L3Pos = new LoggedTunableNumber("Elevator/L3", 36);
    public static final LoggedTunableNumber L4Pos = new LoggedTunableNumber("Elevator/L4", 63.75);
    public static final LoggedTunableNumber bargePos = new LoggedTunableNumber("Elevator/Barge", 0);
    public static final double lowDealgifyPos = 20;
    public static final double highDealgifyPos = 36;
    public static final double processorPos = 4;

    public static final LoggedTunableNumber elevatorTuningVoltage = new LoggedTunableNumber("Elevator/TuningVoltage", 4);
    public static final LoggedTunableNumber elevatorTuningVelocity = new LoggedTunableNumber("Elevator/TuningVelocity", 65);
}