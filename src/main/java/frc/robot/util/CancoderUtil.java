package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class CancoderUtil {
    public static class ConfigFactory {
        private final CANcoderConfiguration config;

        public ConfigFactory() {
            config = new CANcoderConfiguration();
        }

        public void setInverted(boolean shouldInvert) {
            config.MagnetSensor.SensorDirection = shouldInvert ? SensorDirectionValue.Clockwise_Positive
                    : SensorDirectionValue.CounterClockwise_Positive;
        }

        public void setOffset(double offset) {
            config.MagnetSensor.MagnetOffset = offset;
        }

        public CANcoderConfiguration getConfig() {
            return config;
        }
    }

    public static void applySettings(CANcoder cancoder, CANcoderConfiguration config) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = cancoder.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }
}
