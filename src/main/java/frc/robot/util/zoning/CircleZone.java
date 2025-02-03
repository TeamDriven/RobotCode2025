package frc.robot.util.zoning;

import java.awt.geom.Ellipse2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;

public class CircleZone implements Zone {
    private final Translation2d centerPoint;
    private final double radius;

    public CircleZone(Translation2d centerPoint, double radius) {        
        this.centerPoint = centerPoint;
        this.radius = radius;
    }
    
    @Override
    public boolean isInZone(Translation2d point) {
        var circle = makeCircle();

        double x = point.getY();
        double y = point.getX();
        
        return circle.contains(x, y);
    }

    @Override
    public boolean isRobotInZone(Pose2d robotPose) {
        var circle = makeCircle();

        double xOffset = DriveConstants.driveConfig.bumperWidthX() / 2;
        double yOffset = DriveConstants.driveConfig.bumperWidthY() / 2;

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                double x = robotPose.getY() + (yOffset * i);
                double y = robotPose.getX() + (xOffset * j);

                if (circle.contains(x, y)) return true;
            }
        }

        return false;
    }

    private Ellipse2D makeCircle() {
        var center = AllianceFlipUtil.apply(centerPoint);

        double x = center.getY() - radius;
        double y = center.getX() - radius;

        return new Ellipse2D.Double(x, y, radius*2, radius*2);
    }
}
