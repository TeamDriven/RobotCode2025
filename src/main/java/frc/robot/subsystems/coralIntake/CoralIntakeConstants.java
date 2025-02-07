package frc.robot.subsystems.coralIntake;

import frc.robot.util.LoggedTunableNumber;

public class CoralIntakeConstants {
    // public static final int intakeVelocity = -40;
    // public static final int outtakeVelocity = 60;
    public static final int motorAcceleration = 60;

    public static final double detectionCurrent = 9999;

    public static final LoggedTunableNumber intakeVelocity = new LoggedTunableNumber("CoralIntake/intakeVelocity", -40);
    public static final LoggedTunableNumber outtakeVelocity = new LoggedTunableNumber("CoralIntake/outtakeVelocity", 60);
}
