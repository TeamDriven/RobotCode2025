package frc.robot;

import static frc.robot.Constants.driver;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class Controls {
    private static final boolean rightStickDrive = false;

    // Drivetrain
    public static DoubleSupplier driveX = () -> rightStickDrive ? -driver.getRightY() : -driver.getLeftY();
    public static DoubleSupplier driveY = () -> rightStickDrive ? -driver.getRightX() : -driver.getLeftX();
    public static DoubleSupplier driveOmega = () -> rightStickDrive ? -driver.getLeftX() : -driver.getRightX();
    public static Trigger resetPose = driver.start();

    public static Trigger resetElevatorPosition = driver.back();

    public static Trigger manualMode = rightStickDrive ? driver.b() : driver.pov(90);

    // Actions
    class StandardMode {
        public static Trigger cancelAction = (rightStickDrive ? driver.leftStick() : driver.rightStick())
                .and(RobotState.getInstance()::isStandardMode);

        // public static Trigger placeL4 = (rightStickDrive ? driver.pov(0) :
        // driver.y())
        // .and(RobotState.getInstance()::isStandardMode);
        // public static Trigger placeL3 = (rightStickDrive ? driver.pov(90) :
        // driver.b())
        // .and(RobotState.getInstance()::isStandardMode);
        // public static Trigger placeL2 = (rightStickDrive ? driver.pov(270) :
        // driver.x())
        // .and(RobotState.getInstance()::isStandardMode);
        // public static Trigger placeL1 = (rightStickDrive ? driver.pov(180) :
        // driver.a())
        // .and(RobotState.getInstance()::isStandardMode);

        public static Trigger placeAlgae = (rightStickDrive ? driver.pov(0) : driver.y())
                .and(RobotState.getInstance()::isStandardMode);

        public static Trigger processor = (rightStickDrive ? driver.pov(90) : driver.b())
                .and(RobotState.getInstance()::isStandardMode);

        public static Trigger inttake = driver.rightBumper().and(RobotState.getInstance()::isStandardMode);
        public static Trigger outtake = driver.leftBumper().and(RobotState.getInstance()::isStandardMode);

        // public static Trigger maintainIntake =
        // driver.leftTrigger().and(RobotState.getInstance()::isStandardMode);

        public static Trigger highDealgify = driver.rightTrigger(0.1).and(RobotState.getInstance()::isStandardMode);
        public static Trigger lowDealgify = driver.leftTrigger(0.1).and(RobotState.getInstance()::isStandardMode);

        public static Trigger climb = (rightStickDrive ? driver.y() : driver.pov(0))
                .and(RobotState.getInstance()::isStandardMode);
        public static Trigger deployClimber = (rightStickDrive ? driver.a() : driver.pov(180))
                .and(RobotState.getInstance()::isStandardMode);

        public static Trigger turtleMode = (rightStickDrive ? driver.x() : driver.pov(270));
    }

    class NoLimelightMode {

    }

    class ManualMode {
        public static Trigger elevatorUp = driver.rightTrigger(0.1).and(RobotState.getInstance()::isManualMode);
        public static Trigger elevatorDown = driver.leftTrigger(0.1).and(RobotState.getInstance()::isManualMode);

        public static Trigger actuationUp = (rightStickDrive ? driver.pov(0) : driver.y())
                .and(RobotState.getInstance()::isManualMode);
        public static Trigger actuationDown = (rightStickDrive ? driver.pov(180) : driver.a())
                .and(RobotState.getInstance()::isManualMode);

        public static Trigger intakeIn = driver.rightBumper().and(RobotState.getInstance()::isManualMode);
        public static Trigger intakeOut = driver.leftBumper().and(RobotState.getInstance()::isManualMode);

        public static Trigger winchOut = (rightStickDrive ? driver.y() : driver.pov(0))
                .and(RobotState.getInstance()::isManualMode);
        public static Trigger winchIn = (rightStickDrive ? driver.a() : driver.pov(180))
                .and(RobotState.getInstance()::isManualMode);

        public static Trigger footerOut = (rightStickDrive ? driver.pov(90) : driver.b())
                .and(RobotState.getInstance()::isManualMode);
    }
}
