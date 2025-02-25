package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInputs {
        public double leftMotorPos = 0;
        public double leftMotorVoltage = 0;
        public double leftMotorCurrent = 0;
        public double leftMotorVel = 0;
        public double leftMotorAccel = 0;
        public double leftTemp = 0;
        public boolean leftIsMotionMagic = false;

        public double rightMotorPos = 0;
        public double rightMotorVoltage = 0;
        public double rightMotorCurrent = 0;
        public double rightMotorVel = 0;
        public double rightMotorAccel = 0;
        public double rightTemp = 0;
        public boolean rightIsMotionMagic = false;

        public double absoluteEncoderPos = 0;
        public double relativeEncoderPos = 0;
    }
    
    default void updateInputs(ElevatorIOInputs inputs) {}

    default void moveToPos(double pos) {}

    default void runVelocity(double speed) {}

    default void runVoltage(double volts) {}

    default void stopMotors() {}
}
