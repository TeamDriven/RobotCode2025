package frc.robot.commands.drivetrain;

import static frc.robot.Subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleAutoTurn extends Command {
    private static final double minStartTime = 0.25;
    private Timer startTimer = new Timer();

    private final Supplier<Rotation2d> supplier;

    public TeleAutoTurn(Supplier <Rotation2d> rotSupplier) {
        supplier = rotSupplier;
    }

    @Override
    public void initialize() {
        drive.setHeadingGoal(supplier);
        startTimer.restart();
    }

    @Override
    public void end(boolean isInterrupted) {
        drive.clearHeadingGoal();
        startTimer.stop();
    }

    @Override
    public boolean isFinished() {
        return drive.atHeadingGoal() && startTimer.hasElapsed(minStartTime);
    }
    
}
