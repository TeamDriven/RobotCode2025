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

  //coralIntake
  public static Trigger intake = driver.x();
  public static Trigger outtake = driver.b();

  public static Trigger coralActuationUp = new Trigger(() -> false);
  public static Trigger coralActuationDown = new Trigger(() -> false);

  // Algae Actuation
  public static Trigger algaeActuationUp = new Trigger(() -> false);
  public static Trigger algaeActuationDown = new Trigger(() -> false);

  // Algae Intake
  public static Trigger algaeIntakeIn = new Trigger(() -> false);
  public static Trigger algaeIntakeOut = new Trigger(() -> false);
  
  public static Trigger elevatorUp = new Trigger(() -> false);
  public static Trigger elevatorDown = new Trigger(() -> false);

  // Climber
  public static Trigger climberUp = new Trigger(() -> false);
  public static Trigger climberDown = new Trigger(() -> false);
}
