package frc.robot.subsystems.algaeActuation;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeActuationIO {
    @AutoLog
    public class AlgaeActuationIOInputs {
        public double motorPos = 0;
        public double motorVel = 0;
    }
    
    default void updateInputs(AlgaeActuationIOInputs inputs) {}

    /**
     * Move the actuation to a position using motion magic
     * @param pos position in degrees
     */
    default void moveToPos(double pos) {}

    default void runVoltage(double voltage, double acceleration) {}

    default void stopMotors() {}
}
