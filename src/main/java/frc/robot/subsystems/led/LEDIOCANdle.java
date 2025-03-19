package frc.robot.subsystems.led;

import static frc.robot.subsystems.led.LEDConstants.numLED;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class LEDIOCANdle implements LEDIO {
    private final CANdle candle;
    
    public LEDIOCANdle(int id) {
        candle = new CANdle(id);
        candle.configLEDType(LEDStripType.RGB);
        candle.configBrightnessScalar(0.3);
    }

    @Override
    public void animate(Animation animation) {
        candle.clearAnimation(0);
        candle.animate(animation);
    }

    @Override
    public void setColor(int r, int g, int b) {
        candle.clearAnimation(0);
        candle.setLEDs(r, g, b, 0, 8, numLED);
    }

    @Override
    public void setColor(int r, int g, int b, int startIndex, int count) {
        candle.clearAnimation(0);
        candle.setLEDs(r, g, b, 0, startIndex+8, count);
    }
}
