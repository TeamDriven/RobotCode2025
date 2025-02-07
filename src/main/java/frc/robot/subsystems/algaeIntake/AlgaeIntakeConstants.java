package frc.robot.subsystems.algaeIntake;

import frc.robot.util.LoggedTunableNumber;

public class AlgaeIntakeConstants {
    public static final double gearRatio = 0;

    public static final double inSpeed = 40;
    public static final double outSpeed = -40;

    public static final LoggedTunableNumber algaeIntakeTuningVelocity = new LoggedTunableNumber("AlgaeIntake/intakeTuningVelocity", 40);
}
