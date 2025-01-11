package frc.robot.util;

import java.util.Arrays;
import java.util.Objects;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Represents the wheel positions for a swerve drive drivetrain. */
public class SwerveDriveWheelPositions {
  /** The distances driven by the wheels. */
  public SwerveModulePosition[] positions;

  /**
   * Creates a new SwerveDriveWheelPositions instance.
   *
   * @param positions The swerve module positions. This will be deeply copied.
   */
  public SwerveDriveWheelPositions(SwerveModulePosition[] positions) {
    this.positions = new SwerveModulePosition[positions.length];
    for (int i = 0; i < positions.length; i++) {
      this.positions[i] = positions[i].copy();
    }
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof SwerveDriveWheelPositions) {
      SwerveDriveWheelPositions other = (SwerveDriveWheelPositions) obj;
      return Arrays.equals(this.positions, other.positions);
    }
    return false;
  }

  @Override
  public int hashCode() {
    // Cast to interpret positions as single argument, not array of the arguments
    return Objects.hash((Object) positions);
  }

  @Override
  public String toString() {
    return String.format("SwerveDriveWheelPositions(%s)", Arrays.toString(positions));
  }

  public SwerveDriveWheelPositions copy() {
    return new SwerveDriveWheelPositions(positions);
  }

  public SwerveDriveWheelPositions interpolate(SwerveDriveWheelPositions endValue, double t) {
    if (endValue.positions.length != positions.length) {
      throw new IllegalArgumentException("Inconsistent number of modules!");
    }
    var newPositions = new SwerveModulePosition[positions.length];
    for (int i = 0; i < positions.length; i++) {
      newPositions[i] = positions[i].interpolate(endValue.positions[i], t);
    }
    return new SwerveDriveWheelPositions(newPositions);
  }
}
