package frc.robot.subsystems.algaeIntake;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {
    @AutoLog
    public class AlgaeIntakeIOInputs {
        public double motorPos = 0;
        public double motorVel = 0;
    }
    
    default void updateInputs(AlgaeIntakeIOInputs inputs) {}

    default void runVelocity(double speed) {}

    default void stopMotors() {}
}
