package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;

public interface LEDIO {
    default void animate(Animation animation) {}

    default void setColor(int r, int g, int b) {}
    default void setColor(int r, int g, int b, int startIndex, int count) {}
}
