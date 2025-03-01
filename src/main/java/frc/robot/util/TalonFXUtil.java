package frc.robot.util;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
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

        public void setSensorToOutputRatio(double setSensorToOutputRatio) {
            config.Feedback.SensorToMechanismRatio = setSensorToOutputRatio;
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

        public void setExternalEncoder(int encoderID, FeedbackSensorSourceValue encoderType) {
            config.Feedback.FeedbackRemoteSensorID = encoderID;
            config.Feedback.FeedbackSensorSource = encoderType;
            // config.Feedback.FeedbackRotorOffset
        }

        public void setExternalEncoder(int encoderID, FeedbackSensorSourceValue encoderType, double motorToSensorRatio) {
            config.Feedback.FeedbackRemoteSensorID = encoderID;
            config.Feedback.FeedbackSensorSource = encoderType;
            config.Feedback.RotorToSensorRatio = motorToSensorRatio;
            // config.Feedback.FeedbackRotorOffset
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

        public void setSlot0(double kS, double kG, GravityTypeValue gravityType) {
            config.Slot0.kS = kS;
            config.Slot0.kG = kG;
            config.Slot0.GravityType = gravityType;
        }

        public void setSlot0(double kS) {
            setSlot0(kS, 0, GravityTypeValue.Elevator_Static);
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

        public void setSlot1(double kS, double kG, GravityTypeValue gravityType) {
            config.Slot1.kS = kS;
            config.Slot1.kG = kG;
            config.Slot1.GravityType = gravityType;
        }

        public void setSlot1(double kS) {
            setSlot1(kS, 0, GravityTypeValue.Elevator_Static);
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

        public void setSlot2(double kS, double kG, GravityTypeValue gravityType) {
            config.Slot2.kS = kS;
            config.Slot2.kG = kG;
            config.Slot2.GravityType = gravityType;
        }

        public void setSlot2(double kS) {
            setSlot2(kS, 0, GravityTypeValue.Elevator_Static);
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

    public static class MotorFactory {
        private final TalonFX[] motors;
        private final ConfigFactory configFactory;
        private final String key;

        private boolean[] invertedValues = null;

        private ArrayList<LoggedTunableNumber> activeSettings = new ArrayList<>();

        private LoggedTunableNumber currentLimits = null;
        private LoggedTunableNumber forwardVoltageLimit = null;
        private LoggedTunableNumber reverseVoltageLimit = null;

        private LoggedTunableNumber mmVel = null;
        private LoggedTunableNumber mmAccel = null;
        private LoggedTunableNumber mmJerk = null;

        private LoggedTunableNumber slot0kP = null;
        private LoggedTunableNumber slot0kI = null;
        private LoggedTunableNumber slot0kD = null;
        private LoggedTunableNumber slot0kS = null;
        private LoggedTunableNumber slot0kG = null;
        private GravityTypeValue slot0GravityType;

        private LoggedTunableNumber slot1kP = null;
        private LoggedTunableNumber slot1kI = null;
        private LoggedTunableNumber slot1kD = null;
        private LoggedTunableNumber slot1kS = null;
        private LoggedTunableNumber slot1kG = null;
        private GravityTypeValue slot1GravityType;

        private LoggedTunableNumber slot2kP = null;
        private LoggedTunableNumber slot2kI = null;
        private LoggedTunableNumber slot2kD = null;
        private LoggedTunableNumber slot2kS = null;
        private LoggedTunableNumber slot2kG = null;
        private GravityTypeValue slot2GravityType;

        public MotorFactory(String key, int... ids) {
            this.key = key;

            motors = new TalonFX[ids.length];

            for (int i = 0; i < ids.length; i++) {
                motors[i] = new TalonFX(ids[i]);
            }

            configFactory = new ConfigFactory();
        }

        public void setInverted(boolean... shouldInvert) {
            if (shouldInvert.length != motors.length) throw new IllegalArgumentException();

            this.invertedValues = shouldInvert;
        }

        public void setBrakeMode(boolean brakeMode) {
            configFactory.setBrakeMode(brakeMode);
        }

        public void setCurrentLimits(double currentLimit) {
            configFactory.setCurrentLimits(currentLimit);
            this.currentLimits = new LoggedTunableNumber(key + "/CurrentLimit", currentLimit);
            activeSettings.add(this.currentLimits);
        }

        public void setSensorToOutputRatio(double gearRatio) {
            configFactory.setSensorToOutputRatio(gearRatio);
        }

        public void setContinousWrap(boolean continousWrap) {
            configFactory.setContinousWrap(continousWrap);
        }

        public void setMotionMagic(double maxVelocity, double maxAccel, double jerk) {
            configFactory.setMotionMagic(maxVelocity, maxAccel, jerk);

            this.mmVel = new LoggedTunableNumber(key + "/MotionMagic/MaxVelocity", maxVelocity);
            activeSettings.add(this.mmVel);

            this.mmAccel = new LoggedTunableNumber(key + "/MotionMagic/MaxAcceleration", maxAccel);
            activeSettings.add(this.mmAccel);

            this.mmJerk = new LoggedTunableNumber(key + "/MotionMagic/Jerk", jerk);
            activeSettings.add(this.mmJerk);
        }

        public void setVoltageLimits(double maxVoltage) {
            setVoltageLimits(maxVoltage, -maxVoltage);
        }

        public void setVoltageLimits(double maxForwardVoltage, double maxReverseVoltage) {
            configFactory.setVoltageLimits(maxForwardVoltage, maxReverseVoltage);

            this.forwardVoltageLimit = new LoggedTunableNumber(key + "/ForwardVoltageLimit", maxForwardVoltage);
            activeSettings.add(this.forwardVoltageLimit);

            this.reverseVoltageLimit = new LoggedTunableNumber(key + "/ReverseVoltageLimit", maxReverseVoltage);
            activeSettings.add(this.reverseVoltageLimit);
        }

        public void setExternalEncoder(int encoderID, FeedbackSensorSourceValue encoderType) {
            configFactory.setExternalEncoder(encoderID, encoderType);
        }

        public void setExternalEncoder(int encoderID, FeedbackSensorSourceValue encoderType, double motorToSensorRatio) {
            configFactory.setExternalEncoder(encoderID, encoderType, motorToSensorRatio);
        }
        
        public void setSlot0(double kP, double kI, double kD) {
            configFactory.setSlot0(kP, kI, kD);

            this.slot0kP = new LoggedTunableNumber(key + "/Slot0/kP", kP);
            activeSettings.add(this.slot0kP);

            this.slot0kI = new LoggedTunableNumber(key + "/Slot0/kI", kI);
            activeSettings.add(this.slot0kI);

            this.slot0kD = new LoggedTunableNumber(key + "/Slot0/kD", kD);
            activeSettings.add(this.slot0kD);
        }

        public void setSlot0(double kS, double kG, GravityTypeValue gravityType) {
            configFactory.setSlot0(kS, kG, gravityType);

            this.slot0kS = new LoggedTunableNumber(key + "/Slot0/kS", kS);
            activeSettings.add(this.slot0kS);

            this.slot0kG = new LoggedTunableNumber(key + "/Slot0/kG", kG);
            activeSettings.add(this.slot0kG);

            slot0GravityType = gravityType;
        }

        public void setSlot0(double kS) {
            configFactory.setSlot0(kS);

            this.slot0kS = new LoggedTunableNumber(key + "/Slot0/kS", kS);
            activeSettings.add(this.slot0kS);
        }

        public void setSlot1(double kP, double kI, double kD) {
            configFactory.setSlot1(kP, kI, kD);

            this.slot1kP = new LoggedTunableNumber(key + "/Slot1/kP", kP);
            activeSettings.add(this.slot1kP);

            this.slot1kI = new LoggedTunableNumber(key + "/Slot1/kI", kI);
            activeSettings.add(this.slot1kI);

            this.slot1kD = new LoggedTunableNumber(key + "/Slot1/kD", kD);
            activeSettings.add(this.slot1kD);
        }

        public void setSlot1(double kS, double kG, GravityTypeValue gravityType) {
            configFactory.setSlot1(kS, kG, gravityType);

            this.slot1kS = new LoggedTunableNumber(key + "/Slot1/kS", kS);
            activeSettings.add(this.slot1kS);

            this.slot1kG = new LoggedTunableNumber(key + "/Slot1/kG", kG);
            activeSettings.add(this.slot1kG);

            slot1GravityType = gravityType;
        }

        public void setSlot1(double kS) {
            configFactory.setSlot1(kS);

            this.slot1kS = new LoggedTunableNumber(key + "/Slot1/kS", kS);
            activeSettings.add(this.slot1kS);
        }

        public void setSlot2(double kP, double kI, double kD) {
            configFactory.setSlot2(kP, kI, kD);

            this.slot2kP = new LoggedTunableNumber(key + "/Slot2/kP", kP);
            activeSettings.add(this.slot2kP);

            this.slot2kI = new LoggedTunableNumber(key + "/Slot2/kI", kI);
            activeSettings.add(this.slot2kI);

            this.slot2kD = new LoggedTunableNumber(key + "/Slot2/kD", kD);
            activeSettings.add(this.slot2kD);
        }

        public void setSlot2(double kS, double kG, GravityTypeValue gravityType) {
            configFactory.setSlot2(kS, kG, gravityType);

            this.slot2kS = new LoggedTunableNumber(key + "/Slot2/kS", kS);
            activeSettings.add(this.slot2kS);

            this.slot2kG = new LoggedTunableNumber(key + "/Slot2/kG", kG);
            activeSettings.add(this.slot2kG);

            slot2GravityType = gravityType;
        }

        public void setSlot2(double kS) {
            configFactory.setSlot2(kS);

            this.slot2kS = new LoggedTunableNumber(key + "/Slot2/kS", kS);
            activeSettings.add(this.slot2kS);
        }

        public void configureMotors() {
            if (currentLimits != null) {
                configFactory.setCurrentLimits(currentLimits.get());
            }

            if (forwardVoltageLimit != null && reverseVoltageLimit != null) {
                configFactory.setVoltageLimits(forwardVoltageLimit.get(), reverseVoltageLimit.get());
            }

            if (mmVel != null && mmAccel != null && mmJerk != null) {
                configFactory.setMotionMagic(mmVel.get(), mmAccel.get(), mmJerk.get());
            }

            if (slot0kP != null && slot0kI != null && slot0kD != null) {
                configFactory.setSlot0(slot0kP.get(), slot0kI.get(), slot0kD.get());
            }

            if (slot0kS != null) {
                if (slot0kG != null && slot0GravityType != null) {
                    configFactory.setSlot0(slot0kS.get(), slot0kG.get(), slot0GravityType);
                } else {
                    configFactory.setSlot0(slot0kS.get());
                }
            }

            if (slot1kP != null && slot1kI != null && slot1kD != null) {
                configFactory.setSlot1(slot1kP.get(), slot1kI.get(), slot1kD.get());
            }

            if (slot1kS != null) {
                if (slot1kG != null && slot1GravityType != null) {
                    configFactory.setSlot1(slot1kS.get(), slot1kG.get(), slot1GravityType);
                } else {
                    configFactory.setSlot1(slot1kS.get());
                }
            }

            if (slot2kP != null && slot2kI != null && slot2kD != null) {
                configFactory.setSlot2(slot2kP.get(), slot2kI.get(), slot2kD.get());
            }

            if (slot2kS != null) {
                if (slot2kG != null && slot2GravityType != null) {
                    configFactory.setSlot2(slot2kS.get(), slot2kG.get(), slot2GravityType);
                } else {
                    configFactory.setSlot2(slot2kS.get());
                }
            }

            for (int i = 0; i < motors.length; i++) {
                if (invertedValues != null) {
                    configFactory.setInverted(invertedValues[i]);
                }

                applySettings(motors[i], configFactory.getConfig());
            }
        }

        public TalonFX[] getMotors() {
            return motors;
        }

        public void checkForUpdates() {
            LoggedTunableNumber.ifChanged(hashCode(), () -> configureMotors(), activeSettings.toArray(new LoggedTunableNumber[activeSettings.size()]));
        }
    }
}
