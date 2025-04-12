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
    public static final double tuckPos = 13;
    public static final LoggedTunableNumber pickUpPos = new LoggedTunableNumber("Elevator/PickUp", 2);
    public static final LoggedTunableNumber L4Pos = new LoggedTunableNumber("Elevator/L4", 44);
    public static final LoggedTunableNumber bargePos = new LoggedTunableNumber("Elevator/Barge", 52);
    public static final LoggedTunableNumber processorPos = new LoggedTunableNumber("Elevator/Processor", 1);
    public static final LoggedTunableNumber turtlePos = new LoggedTunableNumber("Elevator/Turtle", 2);
    public static final LoggedTunableNumber lowDealgifyPos = new LoggedTunableNumber("Elevator/lowDealigfy", 24);
    public static final LoggedTunableNumber highDealgifyPos = new LoggedTunableNumber("Elevator/highDealgify", 40);

    public static final LoggedTunableNumber elevatorTuningVoltage = new LoggedTunableNumber("Elevator/TuningVoltage", 4);
    public static final LoggedTunableNumber elevatorTuningVelocity = new LoggedTunableNumber("Elevator/TuningVelocity", 65);
}