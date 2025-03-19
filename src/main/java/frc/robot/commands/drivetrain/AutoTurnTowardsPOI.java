package frc.robot.commands.drivetrain;

import static frc.robot.Subsystems.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoTurnTowardsPOI extends Command {

  protected Supplier<Rotation2d> angleSupplier;
  private DoubleSupplier offset;
  private Supplier<Translation2d> POISupplier;

  /**
   * Turn the robot to face a point of interest
   * @param POISupplier The location of the point of interest
   * @param offset The offset from facing straight forward
   */
  public AutoTurnTowardsPOI(Supplier<Translation2d> POISupplier, DoubleSupplier offset) {
    this.offset = offset;
    this.POISupplier = POISupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Translation2d POILocation =
        AllianceFlipUtil.apply(POISupplier.get());
        
    // this.angleSupplier =
    //     () -> {
    //       Transform2d translation =
    //           new Transform2d(
    //               POILocation.getX() - RobotState.getInstance().getEstimatedPose().getX(),
    //               POILocation.getY() - RobotState.getInstance().getEstimatedPose().getY(),
    //               new Rotation2d());
    //       return new Rotation2d(
    //           Math.atan2(translation.getY(), translation.getX())
    //               + Units.degreesToRadians(offset.getAsDouble()));
    //     };

    this.angleSupplier =
        () -> {
          Translation2d translation =
              new Translation2d(
                  POILocation.getX() - RobotState.getInstance().getEstimatedPose().getX(),
                  POILocation.getY() - RobotState.getInstance().getEstimatedPose().getY());
          return translation.getAngle().plus(Rotation2d.fromDegrees(offset.getAsDouble()));
        };

    drive.setHeadingGoal(angleSupplier);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    drive.clearHeadingGoal();
  }

  @Override
  public boolean isFinished() {
    return drive.atHeadingGoal();
  }
}
