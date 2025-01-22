package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
    @AutoLog
    public class CoralIntakeIOInputs {
        public double velocity = 0;
        public double appliedVolts = 0;
        public double supplyCurrentAmps = 0;
    }

    default void updateInputs(CoralIntakeIOInputs inputs) {}

    default void runMotor(double velocity) {}

    default void stopMotor() {}
    
}
