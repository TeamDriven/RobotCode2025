package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class LED extends SubsystemBase {
    private final LEDIO ledIO;

    private Animation curAnimation = LEDConstants.IDLE_ANIMATION;

    public LED(LEDIO ledIO) {
        this.ledIO = ledIO;
        ledIO.animate(curAnimation);
    }

    @Override
    public void periodic() {
        Animation desiredAnimation;

        if (!DriverStation.isTeleopEnabled()) {
            desiredAnimation = LEDConstants.IDLE_ANIMATION;
        } else if (RobotState.getInstance().isManualMode()) { // Fire
            desiredAnimation = LEDConstants.MANUAL_MODE_ANIMATION;
        } else if (RobotState.getInstance().isNoLimelightMode()) { // Twinkle
            desiredAnimation = curAnimation;
        } else if (RobotState.getInstance().hasCoral()) { // Larson
            desiredAnimation = switch (RobotState.getInstance().getDesiredAction()) {
                case L4 -> LEDConstants.hasCoralAnimations[0];
                case L3 -> LEDConstants.hasCoralAnimations[1];
                case L2 -> LEDConstants.hasCoralAnimations[2];
                case L1 -> LEDConstants.hasCoralAnimations[3];
                case PICKUP_CORAL -> LEDConstants.hasCoralAnimations[4];
                case DEALGIFY_HIGH -> LEDConstants.hasCoralAnimations[5];
                case DEALGIFY_LOW -> LEDConstants.hasCoralAnimations[6];
                case NONE -> LEDConstants.hasCoralAnimations[7];
                default -> curAnimation;
            };
        } else { // Solid
            desiredAnimation = switch (RobotState.getInstance().getDesiredAction()) {
                case L4 -> LEDConstants.normalAnimations[0];
                case L3 -> LEDConstants.normalAnimations[1];
                case L2 -> LEDConstants.normalAnimations[2];
                case L1 -> LEDConstants.normalAnimations[3];
                case PICKUP_CORAL -> LEDConstants.normalAnimations[4];
                case DEALGIFY_HIGH -> LEDConstants.normalAnimations[5];
                case DEALGIFY_LOW -> LEDConstants.normalAnimations[6];
                case NONE -> LEDConstants.IDLE_ANIMATION;
                default -> curAnimation;
            };
        }

        Logger.recordOutput("LEDs/animation", desiredAnimation.toString());

        if (!curAnimation.equals(desiredAnimation)) {
            curAnimation = desiredAnimation;
            ledIO.animate(curAnimation);
        }
    }

    public Command setColor(int r, int g, int b) {
        return Commands.runOnce(() -> ledIO.setColor(r, g, b), this);
    }

    public Command setColor(int r, int g, int b, int startIndex, int numLeds) {
        return Commands.runOnce(() -> ledIO.setColor(r, g, b, startIndex, numLeds), this);
    }

    public Command setAnimation(Animation animation) {
        return Commands.runOnce(() -> ledIO.animate(animation), this);
    }
}
