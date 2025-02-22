package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
    @AutoLog
    public class CoralIntakeIOInputs {
        public double motorVoltage = 0;
        public double motorCurrent = 0;
        public double motorVel = 0;
        public double motorAccel = 0;
    }

    default void updateInputs(CoralIntakeIOInputs inputs) {}

    default void runMotor(double velocity) {}

    default void stopMotor() {}
    
}
