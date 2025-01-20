package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    private static final double gearRatio = 7; // Is subject to change
    private static final double pulleyRadius = 0; // if 36t pulley 0.6765

    public static final double rotationsToInches = gearRatio / (pulleyRadius * 2 * Math.PI);
    
    public static final double startPos = 0;
    public static final double restingPos = 0;
    public static final double pickUpPos = 0;
    public static final double L1Pos = 0;
    public static final double L2Pos = 0;
    public static final double L3Pos = 0;
    public static final double L4Pos = 0;
}