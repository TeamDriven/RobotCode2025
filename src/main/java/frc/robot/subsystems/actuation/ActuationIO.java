package frc.robot.subsystems.actuation;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ActuationIO {
    @AutoLog
    public class ActuationIOInputs {
        public double motorPos = 0;
        public double motorVoltage = 0;
        public double motorCurrent = 0;
        public double motorVel = 0;

        public boolean encoderConnected = false;

        public Rotation2d absoluteEncoderPos = new Rotation2d();
        public Rotation2d relativeEncoderPos = new Rotation2d();
    }

    default void updateInputs(ActuationIOInputs inputs) {}

    default void moveToPos(double pos) {}

    default void runVoltage(double voltage) {}

    default void setBrakeMode(boolean brakeMode) {}

    default void stopMotor() {}

    default void seedMotor(Rotation2d currentRot) {}
}
