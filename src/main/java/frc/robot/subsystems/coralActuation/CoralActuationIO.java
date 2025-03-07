package frc.robot.subsystems.coralActuation;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface CoralActuationIO {
    @AutoLog
    public class CoralActuationIOInputs {
        public double motorPos = 0;
        public double motorVoltage = 0;
        public double motorCurrent = 0;
        public double motorVel = 0;

        public Rotation2d absoluteEncoderPos = new Rotation2d();
        public Rotation2d relativeEncoderPos = new Rotation2d();
    }

    default void updateInputs(CoralActuationIOInputs inputs) {}

    default void moveToPos(double pos) {}

    default void runVoltage(double voltage) {}

    default void setBrakeMode(boolean brakeMode) {}

    default void stopMotor() {}

    default void seedMotor(Rotation2d currentRot) {}
}
