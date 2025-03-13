package frc.robot;

import static frc.robot.Constants.driver;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

public class Controls {
    private static final boolean rightStickDrive = true;

    // Drivetrain
    public static DoubleSupplier driveX = () -> rightStickDrive ? -driver.getRightY() : -driver.getLeftY();
    public static DoubleSupplier driveY = () -> rightStickDrive ? -driver.getRightX() : -driver.getLeftX();
    public static DoubleSupplier driveOmega = () -> rightStickDrive ? -driver.getLeftX() : -driver.getRightX();
    public static Trigger resetPose = driver.start();

    public static Trigger resetElevatorPosition = driver.back();

    public static Trigger noLimelightMode = driver.leftTrigger().and(rightStickDrive ? driver.x() : driver.pov(270));
    public static Trigger manualMode = driver.leftTrigger().and(rightStickDrive ? driver.b() : driver.pov(90));

    // Actions
    class StandardMode {
        public static Trigger cancelAction = rightStickDrive ? driver.leftStick() : driver.rightStick();

        public static Trigger placeL4 = rightStickDrive ? driver.pov(0) : driver.y();
        public static Trigger placeL3 = rightStickDrive ? driver.pov(90) : driver.b();
        public static Trigger placeL2 = rightStickDrive ? driver.pov(270) : driver.x();
        public static Trigger placeL1 = rightStickDrive ? driver.pov(180) : driver.a();

        public static Trigger inttake = driver.rightBumper();
        public static Trigger outtake = driver.leftBumper();

        public static Trigger dealgify = driver.rightTrigger(0.1);

        public static Trigger climb = rightStickDrive ? driver.y() : driver.pov(0);
        public static Trigger deployClimber = rightStickDrive ? driver.a() : driver.pov(180);
    }

    class NoLimelightMode {

    }

    class ManualMode {
        public static Trigger elevatorUp = driver.rightTrigger(0.1);
        public static Trigger elevatorDown = driver.leftTrigger(0.1)
                .and(
                        rightStickDrive ? driver.x().negate() : driver.pov(270).negate())
                .and(
                        rightStickDrive ? driver.b().negate() : driver.pov(90).negate());

        public static Trigger actuationUp = rightStickDrive ? driver.pov(0) : driver.y();
        public static Trigger actuationDown = rightStickDrive ? driver.pov(180) : driver.a();
            
        public static Trigger intakeIn = driver.rightBumper();
        public static Trigger intakeOut = driver.leftBumper();

        public static Trigger winchOut = rightStickDrive ? driver.y() : driver.pov(0);
        public static Trigger winchIn = rightStickDrive ? driver.a() : driver.pov(180);

        public static Trigger footerOut = rightStickDrive ? driver.pov(90) : driver.b();
    }
}
