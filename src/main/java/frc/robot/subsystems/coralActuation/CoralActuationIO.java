package frc.robot.subsystems.coralActuation;

import org.littletonrobotics.junction.AutoLog;

public interface CoralActuationIO {
    @AutoLog
    public class CoralActuationIOInputs {
        public double motorPos = 0;
        public double motorVoltage = 0;
        public double motorCurrent = 0;
        public double motorVel = 0;
    }

    default void updateInputs(CoralActuationIOInputs inputs) {}

    default void moveToPos(double pos) {}

    default void runVoltage(double voltage) {}

    default void stopMotor() {}
}
