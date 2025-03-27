package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.actions;

public class LED extends SubsystemBase {
    private final LEDIO ledIO;

    private Animation curAnimation = LEDConstants.IDLE_ANIMATION;
    private int[] curColor = null;

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
            if (RobotState.getInstance().getDesiredAction().equals(actions.NONE)) {
                desiredAnimation = LEDConstants.IDLE_ANIMATION;
            } else {
                var desiredColor = switch (RobotState.getInstance().getDesiredAction()) {
                    case L4 -> LEDConstants.L4Color;
                    case L3 -> LEDConstants.L3Color;
                    case L2 -> LEDConstants.L2Color;
                    case L1 -> LEDConstants.L1Color;
                    case PICKUP_CORAL -> LEDConstants.pickupColor;
                    case DEALGIFY_HIGH -> LEDConstants.highDealgifyColor;
                    case DEALGIFY_LOW -> LEDConstants.lowDealgifyColor;
                    default -> LEDConstants.noneColor;
                };

                if (curColor == null || !desiredColor.equals(curColor)) {
                    curColor = desiredColor;
                    curAnimation = null;
                    ledIO.setColor(desiredColor[0], desiredColor[1], desiredColor[2]);
                }
                return;
            }
        }

        if (curAnimation == null || !curAnimation.equals(desiredAnimation)) {
            curAnimation = desiredAnimation;
            curColor = null;
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
