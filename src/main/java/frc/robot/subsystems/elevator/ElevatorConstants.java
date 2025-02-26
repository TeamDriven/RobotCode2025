package frc.robot.subsystems.elevator;

import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants {
    public static final double gearRatio = (50.0 / 13) * (50.0 / 30); // Is subject to change

    private static final double pulleyRadius = 1.1;
    public static final double sensorToInches = 1 / (pulleyRadius * 2 * Math.PI);

    public static final double maxStableVelocity = 75;
    
    public static final double bottomPos = 0;
    public static final double topPos = 62.3;

    public static final double startPos = 0;
    public static final double restingPos = 0;
    public static final double pickUpPos = 0;
    public static final double L1Pos = 0;
    public static final double L2Pos = 0;
    public static final double L3Pos = 0;
    public static final double L4Pos = 0;

    public static final LoggedTunableNumber elevatorTuningVoltage = new LoggedTunableNumber("Elevator/TuningVoltage", 4);
    public static final LoggedTunableNumber elevatorTuningVelocity = new LoggedTunableNumber("Elevator/TuningVelocity", 65);
}