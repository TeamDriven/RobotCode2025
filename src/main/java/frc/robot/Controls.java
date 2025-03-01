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

  // Actions
  public static Trigger placeL4 = rightStickDrive ? driver.pov(0) : driver.y();
  public static Trigger placeL3 = rightStickDrive ? driver.pov(270) : driver.x();
  public static Trigger placeL2 = rightStickDrive ? driver.pov(90) : driver.b();
  public static Trigger placeL1 = rightStickDrive ? driver.pov(180) : driver.a();
  
  // Algae Actuation
  public static Trigger algaeActuationUp = new Trigger(() -> false);
  public static Trigger algaeActuationDown = new Trigger(() -> false);

  // Algae Intake
  public static Trigger algaeIntakeIn = new Trigger(() -> false);
  public static Trigger algaeIntakeOut = new Trigger(() -> false);

  // Coral Actuation
  public static Trigger coralActuationUp = driver.y();
  public static Trigger coralActuationDown = driver.a();

  // Coral Intake
  public static Trigger coralIntakeIn = driver.rightBumper();
  public static Trigger coralOuttakeOut = driver.leftBumper();
  
  // Elevator
  public static Trigger elevatorUp = driver.pov(0);
  public static Trigger elevatorDown = driver.pov(180);
  public static Trigger resetElevatorPosition = driver.back();

  // Climber
  public static Trigger climberUp = driver.pov(270);
  public static Trigger climberDown = driver.pov(90);
}
