package frc.robot;

import static frc.robot.Constants.driver;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class Controls {
  private static final boolean rightStickDrive = true;

  // Drivetrain
  public static DoubleSupplier driveX =
      () -> rightStickDrive ? -driver.getRightY() : -driver.getLeftY();
  public static DoubleSupplier driveY =
      () -> rightStickDrive ? -driver.getRightX() : -driver.getLeftX();
  public static DoubleSupplier driveOmega =
      () -> rightStickDrive ? -driver.getLeftX() : -driver.getRightX();
  public static Trigger resetPose = driver.start();

  // Algae Actuation
  public static Trigger algaeActuationUp = driver.leftBumper();
  public static Trigger algaeActuationDown = driver.rightBumper();

  // Algae Intake
  public static Trigger algaeIntakeIn = driver.x();
  public static Trigger algaeIntakeOut = driver.b();
}
