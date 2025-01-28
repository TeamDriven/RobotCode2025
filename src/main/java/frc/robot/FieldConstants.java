// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.zoning.CircleZone;
import frc.robot.util.zoning.PolygonZone;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
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

    private static final double[] markerY = {86.5, 158.5, 230.5};

    static {
      for (int i = 0; i < markerTranslations.length; i++) {
        markerTranslations[i] = new Translation2d(markerX, Units.inchesToMeters(markerY[i]));
      }
    }
  }

  public static final Translation2d processor = new Translation2d(Units.inchesToMeters(235.7261), Units.inchesToMeters(316.955)); // fill

  public class cageLocations {
    public static final Translation2d[] cageTranslations = new Translation2d[3];

    private static final double cageX = Units.inchesToMeters(fieldLength/2);

    private static final double[] cageY = {31.1875, 74.125, 117.0625};

    static {
      for (int i = 0; i < cageTranslations.length; i++) {
        cageTranslations[i] = new Translation2d(cageX, Units.inchesToMeters(cageY[i]));
      }
    }
  }

  public class coralBranches {
    public static final Pose3d[] L2Branches = new Pose3d[12];
    public static final Pose3d[] L3Branches = new Pose3d[12];
    public static final Pose3d[] L4Branches = new Pose3d[12];

    public static final double L2Height = Units.inchesToMeters(31.875);
    public static final double L3Height = Units.inchesToMeters(47.625);
    public static final double L4Height = Units.inchesToMeters(72);

    private static final double[] branchX = {146.2073, 154.9968, 166.1786, 185.69, 196.9168, 206.6387, 206.6837, 196.8943, 185.7125, 166.2011, 154.9743, 145.222};
    private static final double[] branchY = {164.1706, 181.0476, 187.5554, 187.5181, 181.0866, 164.1806, 151.2783, 134.3940, 127.8464, 127.9089, 134.3532, 151.2331};
    private static final double[] branchRot = {180, 120, 120, 60, 60, 0, 0, 60, 60, 120, 120, 180};

    static {
      for (int i = 0; i < L2Branches.length; i++) {
        L2Branches[i] = new Pose3d(
          Units.inchesToMeters(branchX[i]), 
          Units.inchesToMeters(branchY[i]), 
          L2Height, 
          new Rotation3d(Rotation2d.fromDegrees(branchRot[i]))
        );
      }

      for (int i = 0; i < L3Branches.length; i++) {
        L3Branches[i] = new Pose3d(
          Units.inchesToMeters(branchX[i]), 
          Units.inchesToMeters(branchY[i]), 
          L3Height, 
          new Rotation3d(Rotation2d.fromDegrees(branchRot[i]))
        );
      }

      for (int i = 0; i < L4Branches.length; i++) {
        L4Branches[i] = new Pose3d(
          Units.inchesToMeters(branchX[i]), 
          Units.inchesToMeters(branchY[i]), 
          L4Height, 
          new Rotation3d(Rotation2d.fromDegrees(branchRot[i]))
        );
      }

      
    }
  }

  public class CoralStations {
    private static final Pose3d[] leftChuteLocations = new Pose3d[9];
    private static final Pose3d[] rightChuteLocations = new Pose3d[9];

    private static final double chuteHeight = Units.inchesToMeters(37.5);

    private static final Rotation2d leftRot = Rotation2d.fromDegrees(-54);
    private static final Rotation2d rightRot = Rotation2d.fromDegrees(54);

    private static final double[] leftChuteX = {7.2883, 13.7614, 20.2345, 26.7075, 33.1806, 39.6537, 46.1268, 52.5998, 59.0729};
    private static final double[] leftChuteY = {44.1488, 39.4478, 34.7468, 30.0458, 25.3448, 20.6434, 15.9429, 11.2419, 6.5409};

    static {
      for (int i = 0; i < leftChuteLocations.length; i++) {
        leftChuteLocations[i] = new Pose3d(leftChuteX[i], leftChuteY[i], chuteHeight, new Rotation3d(leftRot));
      }
      for (int i = 0; i < rightChuteLocations.length; i++) {
        rightChuteLocations[i] = new Pose3d(leftChuteX[i], fieldLength-leftChuteY[i], chuteHeight, new Rotation3d(rightRot));
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
        new Translation2d(Units.inchesToMeters(322.438), 0)
      );

      reefZone = new CircleZone(
        new Translation2d(
          Units.inchesToMeters(176.746),
          Units.inchesToMeters(158.499)
        ), 
        Units.inchesToMeters(55)); // 55

      leftPickupZone = new PolygonZone(
        new Translation2d(0, Units.inchesToMeters(48.7015)),
        new Translation2d(0, Units.inchesToMeters(48.7015 + pickupZoneLength)),
        new Translation2d(Units.inchesToMeters(66.6745 + pickupZoneLength), 0),
        new Translation2d(Units.inchesToMeters(66.6745), 0)
      );

      rightPickupZone = new PolygonZone(
        new Translation2d(0, fieldWidth - Units.inchesToMeters(48.7015)),
        new Translation2d(0, fieldWidth - Units.inchesToMeters(48.7015 + pickupZoneLength)),
        new Translation2d(Units.inchesToMeters(66.6745 + pickupZoneLength), fieldWidth),
        new Translation2d(Units.inchesToMeters(66.6745), fieldWidth)
      );
    }
  }
}
