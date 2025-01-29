// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.zoning.CircleZone;
import frc.robot.util.zoning.PolygonZone;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.875);
  public static final double fieldWidth = Units.inchesToMeters(317.0);

  public class coralMarkers {
    public static final Translation2d[] markerTranslations = new Translation2d[3]; // fill

    private static final double markerX = Units.inchesToMeters(48);

    private static final double[] markerY = { 86.5, 158.5, 230.5 };

    static {
      for (int i = 0; i < markerTranslations.length; i++) {
        markerTranslations[i] = new Translation2d(markerX, Units.inchesToMeters(markerY[i]));
      }
    }
  }

  public static final Translation2d processor = new Translation2d(Units.inchesToMeters(235.7261),
      Units.inchesToMeters(316.955)); // fill

  public class cageLocations {
    public static final Translation2d[] cageTranslations = new Translation2d[3];

    private static final double cageX = Units.inchesToMeters(fieldLength / 2);

    private static final double[] cageY = { 31.1875, 74.125, 117.0625 };

    static {
      for (int i = 0; i < cageTranslations.length; i++) {
        cageTranslations[i] = new Translation2d(cageX, Units.inchesToMeters(cageY[i]));
      }
    }
  }

  public class Reef {
    public static record ReefFace(Pose2d facePos, double algaeHeight,
        Pose3d[] L2Positions, Pose3d[] L3Positions, Pose3d[] L4Positions) {
    }

    public enum ReefHeight {
      L4(Units.inchesToMeters(72), -90),
      L3(Units.inchesToMeters(47.625), -35),
      L2(Units.inchesToMeters(31.875), -35),
      L1(Units.inchesToMeters(18), 0);

      ReefHeight(double height, double pitch) {
        this.height = height;
        this.pitch = pitch; // in degrees
      }

      public final double height;
      public final double pitch;
    }

    public static final Translation2d center = new Translation2d(Units.inchesToMeters(176.746),
        Units.inchesToMeters(158.501));
    public static final double faceToZoneLine = Units.inchesToMeters(12); // Side of the reef to the inside of the reef
                                                                          // zone line

    public static final ReefFace[] reefFaces = new ReefFace[6];

    static {
      Pose2d[] centerFaces = new Pose2d[] {
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180)),
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120)),
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60)),
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0)),
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60)),
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120))
      };

      for (int i = 0; i < centerFaces.length; i++) {
        Pose3d[] L2Poses = findCoralPoses(centerFaces[i], ReefHeight.L2);
        Pose3d[] L3Poses = findCoralPoses(centerFaces[i], ReefHeight.L3);
        Pose3d[] L4Poses = findCoralPoses(centerFaces[i], ReefHeight.L4);

        reefFaces[0] = new ReefFace(
            centerFaces[i],
            i % 2 == 0 ? Units.inchesToMeters(43.7) : Units.inchesToMeters(29.834),
            L2Poses,
            L3Poses,
            L4Poses);
      }
    }

    private static Pose3d[] findCoralPoses(Pose2d centerFace, ReefHeight level) {
      Pose3d[] poses = new Pose3d[2];

      Pose2d poseDirection = new Pose2d(center, centerFace.getRotation());
      double adjustX = Units.inchesToMeters(30.738);
      double adjustY = Units.inchesToMeters(6.469);

      poses[0] = new Pose3d(
          new Translation3d(
              poseDirection
                  .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                  .getX(),
              poseDirection
                  .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                  .getY(),
              level.height),
          new Rotation3d(
              0,
              Units.degreesToRadians(level.pitch),
              poseDirection.getRotation().getRadians()));

      poses[1] = new Pose3d(
          new Translation3d(
              poseDirection
                  .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                  .getX(),
              poseDirection
                  .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                  .getY(),
              level.height),
          new Rotation3d(
              0,
              Units.degreesToRadians(level.pitch),
              poseDirection.getRotation().getRadians()));

      return poses;
    }
  }

  public class CoralStations {
    private static final Pose3d[] leftChuteLocations = new Pose3d[9];
    private static final Pose3d[] rightChuteLocations = new Pose3d[9];

    private static final double chuteHeight = Units.inchesToMeters(37.5);

    private static final Rotation2d leftRot = Rotation2d.fromDegrees(-54);
    private static final Rotation2d rightRot = Rotation2d.fromDegrees(54);

    private static final double[] leftChuteX = { 7.2883, 13.7614, 20.2345, 26.7075, 33.1806, 39.6537, 46.1268, 52.5998,
        59.0729 };
    private static final double[] leftChuteY = { 44.1488, 39.4478, 34.7468, 30.0458, 25.3448, 20.6434, 15.9429, 11.2419,
        6.5409 };

    static {
      for (int i = 0; i < leftChuteLocations.length; i++) {
        leftChuteLocations[i] = new Pose3d(leftChuteX[i], leftChuteY[i], chuteHeight, new Rotation3d(leftRot));
      }
      for (int i = 0; i < rightChuteLocations.length; i++) {
        rightChuteLocations[i] = new Pose3d(leftChuteX[i], fieldLength - leftChuteY[i], chuteHeight,
            new Rotation3d(rightRot));
      }
    }
  }

  public class Zones {
    private static final double pickupZoneLength = 40;

    public static PolygonZone climbZone;

    public static CircleZone reefZone;

    public static PolygonZone leftPickupZone;
    public static PolygonZone rightPickupZone;

    static {
      climbZone = new PolygonZone(
          new Translation2d(Units.inchesToMeters(368.438), 0),
          new Translation2d(Units.inchesToMeters(368.438), fieldWidth),
          new Translation2d(Units.inchesToMeters(322.438), fieldWidth),
          new Translation2d(Units.inchesToMeters(322.438), 0));

      reefZone = new CircleZone(
          new Translation2d(
              Units.inchesToMeters(176.746),
              Units.inchesToMeters(158.499)),
          Units.inchesToMeters(55)); // 55

      leftPickupZone = new PolygonZone(
          new Translation2d(0, Units.inchesToMeters(48.7015)),
          new Translation2d(0, Units.inchesToMeters(48.7015 + pickupZoneLength)),
          new Translation2d(Units.inchesToMeters(66.6745 + pickupZoneLength), 0),
          new Translation2d(Units.inchesToMeters(66.6745), 0));

      rightPickupZone = new PolygonZone(
          new Translation2d(0, fieldWidth - Units.inchesToMeters(48.7015)),
          new Translation2d(0, fieldWidth - Units.inchesToMeters(48.7015 + pickupZoneLength)),
          new Translation2d(Units.inchesToMeters(66.6745 + pickupZoneLength), fieldWidth),
          new Translation2d(Units.inchesToMeters(66.6745), fieldWidth));
    }
  }
}
