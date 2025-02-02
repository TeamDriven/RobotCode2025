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
  public static Trigger intake = new Trigger(() -> false);
  public static Trigger outtake = new Trigger(() -> false);

  public static Trigger coralActuationUp = new Trigger(() -> false);
  public static Trigger coralActuationDown = new Trigger(() -> false);

  // Algae Actuation
  public static Trigger algaeActuationUp = driver.y();
  public static Trigger algaeActuationDown = driver.a();

  // Algae Intake
  public static Trigger algaeIntakeIn = driver.x();
  public static Trigger algaeIntakeOut = driver.b();
  
  public static Trigger elevatorUp = driver.pov(0);
  public static Trigger elevatorDown = driver.pov(180);

  // Climber
  public static Trigger climberUp = driver.pov(270);
  public static Trigger climberDown = driver.pov(90);
}
