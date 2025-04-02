package frc.robot.subsystems.intake;

import frc.robot.util.LoggedTunableNumber;

public class IntakeConstants {
    // public static final int intakeVelocity = -40;
    // public static final int outtakeVelocity = 60;
    public static final int motorAcceleration = 60;

    public static final double L4Speed = 35;
    public static final double L3Speed = 38;
    public static final double L2Speed = 40;
    public static final double L1Speed = 0;

    public static final LoggedTunableNumber intakeVelocity = new LoggedTunableNumber("Intake/intakeVelocity", -40);
    public static final LoggedTunableNumber outtakeVelocity = new LoggedTunableNumber("Intake/outtakeVelocity", 36);
    public static final LoggedTunableNumber tuningVoltage = new LoggedTunableNumber("Intake/tuningVoltage", -2);
}
