package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonFXUtil {
    public static class ConfigFactory {
        private final TalonFXConfiguration config;

        public ConfigFactory() {
            config = new TalonFXConfiguration();
        }

        public void setInverted(boolean shouldInvert) {
            config.MotorOutput.Inverted = shouldInvert ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;
        }

        public void setBrakeMode(boolean brakeMode) {
            config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        }

        public void setCurrentLimits(double currentLimit) {
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        }

        public void setGearRatio(double gearRatio) {
            config.Feedback.SensorToMechanismRatio = gearRatio;
        }

        public void setContinousWrap(boolean continousWrap) {
            config.ClosedLoopGeneral.ContinuousWrap = continousWrap;
        }

        public void setMotionMagic(double maxVelocity, double maxAccel, double jerk) {
            config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
            config.MotionMagic.MotionMagicAcceleration = maxAccel;
            config.MotionMagic.MotionMagicJerk = jerk;
        }

        public void setVoltageLimits(double maxVoltage) {
            config.Voltage.PeakForwardVoltage = maxVoltage;
            config.Voltage.PeakReverseVoltage = -maxVoltage;
        }

        public void setVoltageLimits(double maxForwardVoltage, double maxReverseVoltage) {
            config.Voltage.PeakForwardVoltage = maxForwardVoltage;
            config.Voltage.PeakReverseVoltage = maxReverseVoltage;
        }

        public void setSlot0(double kP, double kI, double kD, double kV) {
            config.Slot0.kP = kP;
            config.Slot0.kI = kI;
            config.Slot0.kD = kD;
            config.Slot0.kV = kV;
        }

        public void setSlot0(double kP, double kI, double kD) {
            setSlot0(kP, kI, kD, 0.12);
        }

        public void setSlot1(double kP, double kI, double kD, double kV) {
            config.Slot1.kP = kP;
            config.Slot1.kI = kI;
            config.Slot1.kD = kD;
            config.Slot1.kV = kV;
        }

        public void setSlot1(double kP, double kI, double kD) {
            setSlot1(kP, kI, kD, 0.12);
        }

        public void setSlot2(double kP, double kI, double kD, double kV) {
            config.Slot2.kP = kP;
            config.Slot2.kI = kI;
            config.Slot2.kD = kD;
            config.Slot2.kV = kV;
        }

        public void setSlot2(double kP, double kI, double kD) {
            setSlot2(kP, kI, kD, 0.12);
        }

        public TalonFXConfiguration getConfig() {
            return config;
        }
    }

    public static void applySettings(TalonFX motor, TalonFXConfiguration config) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    public static void applySettings(TalonFX motor1, TalonFX motor2, TalonFXConfiguration config,
            boolean shouldRunOpposite) {
        applySettings(motor1, config);

        // if the motors should run opposite directions of each other, we flip the inverted value
        if (shouldRunOpposite) {
            config.MotorOutput.Inverted = config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive;
        }
        applySettings(motor2, config);
    }
}
