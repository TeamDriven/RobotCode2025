package frc.robot.subsystems.climber;

import static frc.robot.Subsystems.footer;
import static frc.robot.Subsystems.winch;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class climberController {

    public static Command climberOut() {
        return winch.startEnd(() -> winch.runVoltage(12), () -> winch.runVoltage(0));
    }

    public static Command climb() {
        return Commands.parallel(
                winch.startEnd(() -> winch.runVoltage(-12), () -> winch.runVoltage(0)),
                footer.startEnd(() -> footer.runVoltage(-6), () -> footer.runVoltage(0)).withTimeout(0.75));
    }

    public static Command runClimbers(double winchVoltage, double FooterPos) {
        return new SequentialCommandGroup(
                footer.runOnce(() -> footer.setPos(FooterPos)),
                winch.runOnce(() -> winch.runVoltage(winchVoltage)));
    }

    public static Command StopClimbers() {
        return new Command() {
            @Override
            public void execute() {
                footer.runOnce(() -> footer.stop());
                winch.runOnce(() -> winch.runVoltage(0));
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }
}
