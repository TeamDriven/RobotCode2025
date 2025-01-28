package frc.robot.util.zoning;

import java.awt.Polygon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;

public class PolygonZone implements Zone {
    private final int precision = 5;

    private final Translation2d[] vertexes;

    public PolygonZone(Translation2d... vertexes) {        
        this.vertexes = vertexes;
    }
    
    @Override
    public boolean isInZone(Translation2d point) {
        var polygon = makePolygon();

        int x = convertToCoords(point.getY());
        int y = convertToCoords(point.getX());
        
        return polygon.contains(x, y);
    }

    @Override
    public boolean isRobotInZone(Pose2d robotPose) {
        var polygon = makePolygon();

        double xOffset = DriveConstants.driveConfig.bumperWidthX() / 2;
        double yOffset = DriveConstants.driveConfig.bumperWidthY() / 2;

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                int x = convertToCoords(robotPose.getY() + (yOffset * i));
                int y = convertToCoords(robotPose.getX() + (xOffset * j));

                if (polygon.contains(x, y)) return true;
            }
        }

        return false;
    }

    private int convertToCoords(double num) {
        return (int) Math.round(num * Math.pow(10, precision));
    }

    private Polygon makePolygon() {
        int[] xPoints = new int[vertexes.length];
        int[] yPoints = new int[vertexes.length];
        
        for (int i = 0; i < vertexes.length; i++) {
            var vertex = AllianceFlipUtil.apply(vertexes[i]);

            xPoints[i] = convertToCoords(vertex.getY());
            yPoints[i] = convertToCoords(vertex.getX());
        }

        return new Polygon(xPoints, yPoints, vertexes.length);
    }
}
