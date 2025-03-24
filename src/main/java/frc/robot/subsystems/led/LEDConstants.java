package frc.robot.subsystems.led;

import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

public class LEDConstants {
    public static final int CANdleLEDs = 8; 
    public static final int numLED = 26 + CANdleLEDs;

    // public static final int[] teamDrivenYellow = { 235, 211, 17 };

    private static final int[] L4Color = {0, 255, 0};
    private static final int[] L3Color = {150, 20, 0};
    private static final int[] L2Color = {60, 120, 180};
    private static final int[] L1Color = {220, 206, 86};
    private static final int[] pickupColor = {172, 96, 184};
    private static final int[] highDealgifyColor = {230, 80, 40};
    private static final int[] lowDealgifyColor = {130, 50, 20};
    private static final int[] noneColor = {0, 0, 130};

    public static final RainbowAnimation IDLE_ANIMATION = new RainbowAnimation(1, 0.5, numLED, false, CANdleLEDs);

    public static final FireAnimation MANUAL_MODE_ANIMATION = new FireAnimation(1, 0.5, numLED, 1, 0.25, false, CANdleLEDs);

    private static double coralAnimationSpeed = 0.5;
    private static int coralAnimationSize = 8;
    private static BounceMode coralAnimationMode = BounceMode.Front;
    public static final LarsonAnimation[] hasCoralAnimations;

    public static final StrobeAnimation[] normalAnimations;

    static {
        hasCoralAnimations = new LarsonAnimation[] {
            new LarsonAnimation(L4Color[0], L4Color[1], L4Color[2], 0, coralAnimationSpeed, numLED, coralAnimationMode, coralAnimationSize, CANdleLEDs),
            new LarsonAnimation(L3Color[0], L3Color[1], L3Color[2], 0, coralAnimationSpeed, numLED, coralAnimationMode, coralAnimationSize, CANdleLEDs),
            new LarsonAnimation(L2Color[0], L2Color[1], L2Color[2], 0, coralAnimationSpeed, numLED, coralAnimationMode, coralAnimationSize, CANdleLEDs),
            new LarsonAnimation(L1Color[0], L1Color[1], L1Color[2], 0, coralAnimationSpeed, numLED, coralAnimationMode, coralAnimationSize, CANdleLEDs),
            new LarsonAnimation(pickupColor[0], pickupColor[1], pickupColor[2], 0, coralAnimationSpeed, numLED, coralAnimationMode, coralAnimationSize, CANdleLEDs),
            new LarsonAnimation(highDealgifyColor[0], highDealgifyColor[1], highDealgifyColor[2], 0, coralAnimationSpeed, numLED, coralAnimationMode, coralAnimationSize, CANdleLEDs),
            new LarsonAnimation(lowDealgifyColor[0], lowDealgifyColor[1], lowDealgifyColor[2], 0, coralAnimationSpeed, numLED, coralAnimationMode, coralAnimationSize, CANdleLEDs),
            new LarsonAnimation(noneColor[0], noneColor[1], noneColor[2], 0, coralAnimationSpeed, numLED, coralAnimationMode, coralAnimationSize, CANdleLEDs)
        };
        normalAnimations = new StrobeAnimation[] {
            new StrobeAnimation(L4Color[0], L4Color[1], L4Color[2], 0, 0, numLED, CANdleLEDs),
            new StrobeAnimation(L3Color[0], L3Color[1], L3Color[2], 0, 0, numLED, CANdleLEDs),
            new StrobeAnimation(L2Color[0], L2Color[1], L2Color[2], 0, 0, numLED, CANdleLEDs),
            new StrobeAnimation(L1Color[0], L1Color[1], L1Color[2], 0, 0, numLED, CANdleLEDs),
            new StrobeAnimation(pickupColor[0], pickupColor[1], pickupColor[2], 0, 0, numLED, CANdleLEDs),
            new StrobeAnimation(highDealgifyColor[0], highDealgifyColor[1], highDealgifyColor[2], 0, 0, numLED, CANdleLEDs),
            new StrobeAnimation(lowDealgifyColor[0], lowDealgifyColor[1], lowDealgifyColor[2], 0, 0, numLED, CANdleLEDs),
            new StrobeAnimation(noneColor[0], noneColor[1], noneColor[2], 0, 0, numLED, CANdleLEDs),
        };
    }

    // public static final LarsonAnimation LARSON_ANIMATION = new LarsonAnimation(teamDrivenYellow[0], teamDrivenYellow[1],
    //         teamDrivenYellow[2], 0, 1, numLED, BounceMode.Front, 7, 8);


    // public static final RgbFadeAnimation RGB_FADE_ANIMATION = new RgbFadeAnimation(1, 0.5, numLED, 8);

    // public static final TwinkleAnimation TWINKLE_ANIMATION = new TwinkleAnimation(teamDrivenYellow[0], teamDrivenYellow[1], teamDrivenYellow[2], 0, 0.5, numLED, TwinklePercent.Percent64, 8);
}
