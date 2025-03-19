package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final LEDIO ledIO;

    public LED(LEDIO ledIO) {
        this.ledIO = ledIO;
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
