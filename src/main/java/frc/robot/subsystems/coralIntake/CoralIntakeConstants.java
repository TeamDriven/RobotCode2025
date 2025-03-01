package frc.robot.subsystems.coralIntake;

import frc.robot.util.LoggedTunableNumber;

public class CoralIntakeConstants {
    // public static final int intakeVelocity = -40;
    // public static final int outtakeVelocity = 60;
    public static final int motorAcceleration = 60;

    public static final double detectionCurrent = 9999;

    public static final double L4Speed = 40;
    public static final double L3Speed = 40;
    public static final double L2Speed = 40;
    public static final double L1Speed = 40;

    public static final LoggedTunableNumber intakeVelocity = new LoggedTunableNumber("CoralIntake/intakeVelocity", -40);
    public static final LoggedTunableNumber outtakeVelocity = new LoggedTunableNumber("CoralIntake/outtakeVelocity", 40);
    public static final LoggedTunableNumber tuningVoltage = new LoggedTunableNumber("CoralIntake/tuningVoltage", -2);
}
