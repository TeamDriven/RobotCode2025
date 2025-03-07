package frc.robot.subsystems.climber;

import static frc.robot.Subsystems.footer;
import static frc.robot.Subsystems.winch;
import static frc.robot.subsystems.algaeActuation.AlgaeActuationConstants.downPos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class climberConroller{

    public static Command runClimbers(double winchVoltage, double FooterPos) {
        return new SequentialCommandGroup(
            footer.runOnce(() -> footer.setPos(FooterPos)),
            winch.runOnce(() -> winch.runVoltage(winchVoltage))
        );
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
