package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double motorVoltage = 0;
        public double motorCurrent = 0;
        public double motorVel = 0;
        public double motorAccel = 0;
        public double motorTemp = 0;

        public boolean gamePieceSensor = false;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void runMotor(double velocity) {}

    default void runVoltage(double volts) {}

    default void stopMotor() {}
    
}
