package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInputs {
        public double leftMotorPos = 0;
        public double rightMotorPos = 0;
        public double leftMotorVel = 0;
        public double rightMotorVel = 0;
    }
    
    default void updateInputs(ElevatorIOInputs inputs) {}

    default void moveToPos(double pos) {}

    default void runVelocity(double speed) {}

    default void stopMotors() {}
}
