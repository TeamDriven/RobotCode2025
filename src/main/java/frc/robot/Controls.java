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
  public static Trigger algaeActuationUp = new Trigger(() -> false);
  public static Trigger algaeActuationDown = new Trigger(() -> false);

  // Algae Intake
  public static Trigger algaeIntakeIn = new Trigger(() -> false);
  public static Trigger algaeIntakeOut = new Trigger(() -> false);

  // Coral Actuation
  public static Trigger coralActuationUp = new Trigger(() -> false);
  public static Trigger coralActuationDown = new Trigger(() -> false);

  // Coral Intake
  public static Trigger coralIntakeIn = driver.x();
  public static Trigger coralOuttakeOut = driver.b();
  
  // Elevator
  public static Trigger elevatorUp = driver.pov(0);
  public static Trigger elevatorDown = driver.pov(180);

  // Climber
  public static Trigger climberUp = driver.pov(270);
  public static Trigger climberDown = driver.pov(90);
}
