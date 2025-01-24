package frc.robot.subsystems.coralActuation;

import org.littletonrobotics.junction.AutoLog;

public interface CoralActuationIO {
    @AutoLog
    public class CoralActuationIOInputs {
        public double pos = 0;
    }

    default void updateInputs(CoralActuationIOInputs inputs) {}

    default void moveToPos(double pos) {}

    default void runVoltage(double voltage) {}

    default void stopMotor() {}
}
