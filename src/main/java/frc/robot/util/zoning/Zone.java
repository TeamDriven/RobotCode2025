package frc.robot.util.zoning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface Zone {
    public boolean isInZone(Translation2d point);

    public boolean isRobotInZone(Pose2d robotPose);
}
