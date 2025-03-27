// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

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

        private static final double[] markerY = { 230.5, 158.5, 86.5 };

        static {
            for (int i = 0; i < markerTranslations.length; i++) {
                markerTranslations[i] = new Translation2d(markerX, Units.inchesToMeters(markerY[i]));
            }
        }
    }

    public static final Translation2d processor = new Translation2d(Units.inchesToMeters(235.7261),
            Units.inchesToMeters(0)); // fill

    public class cageLocations {
        public static final Translation2d[] cageTranslations = new Translation2d[3];

        private static final double[] cageY = { 31.1875, 74.125, 117.0625 };

        static {
            for (int i = 0; i < cageTranslations.length; i++) {
                cageTranslations[i] = new Translation2d(fieldLength / 2, fieldWidth - Units.inchesToMeters(cageY[i]));
            }
        }
    }

    public class Reef {
        private static final double placeOffset = Units.inchesToMeters(5 + 5.625); // 3.625 is bumper touching

        public static record ReefFace(Pose2d facePos, boolean isAlgaeHigh,
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
        // Side of the reef to the inside of the reef zone line
        public static final double faceToZoneLine = Units.inchesToMeters(12);

        public static final ReefFace[] reefFaces = new ReefFace[6];

        public static List<Pose2d> facePoses;

        public static ReefFace findNearestReefFace(Pose2d robotPose) {
            Pose2d nearestFacePose = robotPose.nearest(facePoses);
            // Streams are too slow for this purpose
            // return Arrays.stream(reefFaces)
            //         .filter((ReefFace face) -> nearestFacePose.equals(face.facePos))
            //         .findFirst()
            //         .get();
            for (ReefFace face : reefFaces) {
                if (nearestFacePose.equals(face.facePos)) return face;
            }
            return null;
        }

        public static final Pose2d[] placePoses = new Pose2d[12];

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

            int placePoseIndex = 0;
            for (int i = 0; i < centerFaces.length; i++) {
                Pose3d[] L2Poses = findCoralPoses(centerFaces[i], ReefHeight.L2);
                Pose3d[] L3Poses = findCoralPoses(centerFaces[i], ReefHeight.L3);
                Pose3d[] L4Poses = findCoralPoses(centerFaces[i], ReefHeight.L4);

                reefFaces[i] = new ReefFace(
                        centerFaces[i],
                        i % 2 == 0,
                        L2Poses,
                        L3Poses,
                        L4Poses);

                for (int j = 0; j < 2; j++) {
                    Pose2d poseDirection = new Pose2d(center, centerFaces[i].getRotation());
                    double adjustX = Units.inchesToMeters(30.738) + (driveConfig.bumperWidthX() / 2) + placeOffset;
                    double adjustY = Units.inchesToMeters(6.469);

                    placePoses[placePoseIndex++] = new Pose2d(
                            poseDirection
                                    .transformBy(
                                            new Transform2d(adjustX, j % 2 == 0 ? adjustY : -adjustY, new Rotation2d()))
                                    .getX(),
                            poseDirection
                                    .transformBy(
                                            new Transform2d(adjustX, j % 2 == 0 ? adjustY : -adjustY, new Rotation2d()))
                                    .getY(),
                            poseDirection.getRotation().rotateBy(new Rotation2d(Math.PI)));
                }
            }

            facePoses = Arrays.stream(reefFaces).map((ReefFace face) -> face.facePos).toList();
        }
    }

    public class CoralStations {
        public static final Pose2d[] pickupLocations = new Pose2d[18];

        public static final Pose3d[] leftChuteLocations = new Pose3d[9];
        public static final Pose3d[] rightChuteLocations = new Pose3d[9];

        public static final double pickupOffset = Units.inchesToMeters(-13.5);

        private static final double chuteHeight = Units.inchesToMeters(37.5);

        private static final double[] rightChuteX = { 7.2883, 13.7614, 20.2345, 26.7075, 33.1806, 39.6537, 46.1268,
                52.5998,
                59.0729 };
        private static final double[] rightChuteY = { 44.1488, 39.4478, 34.7468, 30.0458, 25.3448, 20.6434, 15.9429,
                11.2419,
                6.5409 };

        static {
            for (int i = 0; i < leftChuteLocations.length; i++) {
                var x = Units.inchesToMeters(rightChuteX[i]);
                var y = fieldWidth - Units.inchesToMeters(rightChuteY[i]);

                leftChuteLocations[i] = new Pose3d(
                        x,
                        y,
                        chuteHeight,
                        new Rotation3d(
                                0,
                                Units.degreesToRadians(55),
                                Units.degreesToRadians(-55)));

                Pose2d chutePose = new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(-55)));
                pickupLocations[i] = new Pose2d(
                        chutePose
                                .transformBy(new Transform2d((driveConfig.bumperWidthX() / 2) + pickupOffset, 0,
                                        new Rotation2d()))
                                .getX(),
                        chutePose
                                .transformBy(new Transform2d((driveConfig.bumperWidthX() / 2) + pickupOffset, 0,
                                        new Rotation2d()))
                                .getY(),
                        chutePose.getRotation().rotateBy(new Rotation2d(Math.PI)));
            }

            for (int i = 0; i < rightChuteLocations.length; i++) {
                var x = Units.inchesToMeters(rightChuteX[i]);
                var y = Units.inchesToMeters(rightChuteY[i]);

                rightChuteLocations[i] = new Pose3d(
                        x,
                        y,
                        chuteHeight,
                        new Rotation3d(
                                0,
                                Units.degreesToRadians(55),
                                Units.degreesToRadians(55)));

                Pose2d chutePose = new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(55)));
                pickupLocations[i + 9] = new Pose2d(
                        chutePose
                                .transformBy(new Transform2d((driveConfig.bumperWidthX() / 2) + pickupOffset, 0,
                                        new Rotation2d()))
                                .getX(),
                        chutePose
                                .transformBy(new Transform2d((driveConfig.bumperWidthX() / 2) + pickupOffset, 0,
                                        new Rotation2d()))
                                .getY(),
                        chutePose.getRotation().rotateBy(new Rotation2d(Math.PI)));
            }
        }
    }

    public class Zones {
        private static final double pickupZoneLength = 150; // Not real units

        public static PolygonZone climbZone;

        public static CircleZone reefZone;

        public static PolygonZone leftPickupZone;
        public static PolygonZone rightPickupZone;

        public static Translation2d[] climbZoneCorners = new Translation2d[] {
                new Translation2d(Units.inchesToMeters(368.438 + 24), 0),
                new Translation2d(Units.inchesToMeters(368.438 + 24), fieldWidth),
                new Translation2d(Units.inchesToMeters(322.438 + 24), fieldWidth),
                new Translation2d(Units.inchesToMeters(322.438 + 24), 0) };

        public static Translation2d[] leftPickupZoneCorners = new Translation2d[] {
                new Translation2d(0, fieldWidth - Units.inchesToMeters(48.7015)),
                new Translation2d(0,
                        fieldWidth - Units
                                .inchesToMeters(48.7015 + pickupZoneLength * Math.sin(Units.degreesToRadians(35)))),
                new Translation2d(
                        Units.inchesToMeters(66.6745 + pickupZoneLength * Math.cos(Units.degreesToRadians(35))),
                        fieldWidth),
                new Translation2d(Units.inchesToMeters(66.6745), fieldWidth)
        };

        public static Translation2d[] rightPickupZoneCorners = new Translation2d[] {
                new Translation2d(0, Units.inchesToMeters(48.7015)),
                new Translation2d(0,
                        Units.inchesToMeters(48.7015 + pickupZoneLength * Math.sin(Units.degreesToRadians(35)))),
                new Translation2d(
                        Units.inchesToMeters(66.6745 + pickupZoneLength * Math.cos(Units.degreesToRadians(35))), 0),
                new Translation2d(Units.inchesToMeters(66.6745), 0)
        };

        static {
            climbZone = new PolygonZone(climbZoneCorners);

            reefZone = new CircleZone(
                    Reef.center,
                    Units.inchesToMeters(67));

            leftPickupZone = new PolygonZone(
                    leftPickupZoneCorners);

            rightPickupZone = new PolygonZone(
                    rightPickupZoneCorners);
        }
    }

    public static void logFieldConstants() {
        Logger.recordOutput("FieldConstants/coralMarkers", coralMarkers.markerTranslations);

        Logger.recordOutput("FieldConstants/processor", processor);

        Logger.recordOutput("FieldConstants/cageTranslations", cageLocations.cageTranslations);

        Logger.recordOutput("FieldConstants/reefCenter", Reef.center);

        Logger.recordOutput("FieldConstants/placePoses", Reef.placePoses);

        Logger.recordOutput("FieldConstants/pickupPoses", CoralStations.pickupLocations);

        // Logger.recordOutput(String.format("FieldConstants/reefFaces/%d/L2Branches",
        // i), new Pose3d());

        for (int i = 0; i < Reef.reefFaces.length; i++) {
            Logger.recordOutput(String.format("FieldConstants/reefFaces/%d/facePose", i), Reef.reefFaces[i].facePos);
            Logger.recordOutput(String.format("FieldConstants/reefFaces/%d/isAlgaeHigh", i),
                    Reef.reefFaces[i].isAlgaeHigh);
            Logger.recordOutput(String.format("FieldConstants/reefFaces/%d/L2Positions", i),
                    Reef.reefFaces[i].L2Positions);
            Logger.recordOutput(String.format("FieldConstants/reefFaces/%d/L3Positions", i),
                    Reef.reefFaces[i].L3Positions);
            Logger.recordOutput(String.format("FieldConstants/reefFaces/%d/L4Positions", i),
                    Reef.reefFaces[i].L4Positions);
        }

        Logger.recordOutput("FieldConstants/leftChuteLocations", CoralStations.leftChuteLocations);
        Logger.recordOutput("FieldConstants/rightChuteLocations", CoralStations.rightChuteLocations);

        Logger.recordOutput("FieldConstants/Zones/climbZoneCorners", Zones.climbZoneCorners);
        Logger.recordOutput("FieldConstants/Zones/leftPickupZoneCorners", Zones.leftPickupZoneCorners);
        Logger.recordOutput("FieldConstants/Zones/rightPickupZoneCorners", Zones.rightPickupZoneCorners);
    }
}
