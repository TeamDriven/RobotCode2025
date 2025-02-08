package frc.robot.subsystems.led;

import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

public class LEDConstants {
    public static final int numLED = 146;

    public static final int[] teamDrivenYellow = { 235, 211, 17 };

    public static final RainbowAnimation RAINBOW_ANIMATION = new RainbowAnimation(1, 0.5, numLED, false, 8);

    public static final LarsonAnimation LARSON_ANIMATION = new LarsonAnimation(teamDrivenYellow[0], teamDrivenYellow[1],
            teamDrivenYellow[2], 0, 1, numLED, BounceMode.Front, 7, 8);

    public static final FireAnimation FIRE_ANIMATION = new FireAnimation(1, 0.5, numLED, 1, 0.25, false, 8);

    public static final RgbFadeAnimation RGB_FADE_ANIMATION = new RgbFadeAnimation(1, 0.5, numLED, 8);

    public static final TwinkleAnimation TWINKLE_ANIMATION = new TwinkleAnimation(teamDrivenYellow[0], teamDrivenYellow[1], teamDrivenYellow[2], 0, 0.5, numLED, TwinklePercent.Percent64, 8);
}
