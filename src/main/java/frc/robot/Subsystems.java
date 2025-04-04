// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOKrakenFOC;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.climber.winch.WinchIO;
import frc.robot.subsystems.climber.winch.WinchIOKraken;
import frc.robot.subsystems.actuation.Actuation;
import frc.robot.subsystems.actuation.ActuationIO;
import frc.robot.subsystems.actuation.ActuationIOKraken;
import frc.robot.subsystems.climber.footer.Footer;
import frc.robot.subsystems.climber.footer.FooterIO;
import frc.robot.subsystems.climber.footer.FooterIOKraken;
import frc.robot.subsystems.climber.winch.Winch;

/**
 * The Subsystems class represents the collection of subsystems used in the
 * robot. It provides
 * static references to various subsystem objects that are used in the robot.
 */
public final class Subsystems {
    public static final Drive drive;

    public static final Vision bottomVision;
    public static final Vision backVision;
    public static final Vision topVision;
    // public static final Vision rightVision;

    public static final Intake intake;
    public static final Actuation actuation;
    public static final Elevator elevator;
    public static final Winch winch;
    public static final Footer footer;

    // public static final LED leds;

    static {
        // Create subsystems
        if (Constants.getMode() != Constants.Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case COMPBOT -> {
                    drive = new Drive(
                            new GyroIOPigeon2(true, "DriveBus"),
                            new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[0], "DriveBus"),
                            new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[1], "DriveBus"),
                            new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[2], "DriveBus"),
                            new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[3], "DriveBus"));

                    bottomVision = new Vision("Bottom Vision", new VisionIOLimelight("limelight-bottom"),
                            drive::getSpeeds);
                    backVision = new Vision("Back Vision", new VisionIOLimelight("limelight-back"), drive::getSpeeds);
                    topVision = new Vision("Top Vision", new VisionIOLimelight("limelight-top"), drive::getSpeeds);
                    // bottomVision = new Vision("v1", new VisionIO() {}, drive::getSpeeds);
                    // backVision = new Vision("v2", new VisionIO() {}, drive::getSpeeds);
                    // topVision = new Vision("v3", new VisionIO() {}, drive::getSpeeds);
                    // rightVision = new Vision("Right Vision", new
                    // VisionIOLimelight("limelight-right"), drive::getSpeeds);

                    intake = new Intake(new IntakeIOKraken(13, 0));
                    actuation = new Actuation(new ActuationIO() {});
                    elevator = new Elevator(new ElevatorIO() {});
                    winch = new Winch(new WinchIOKraken(20, 21));
                    footer = new Footer(new FooterIOKraken(19));

                    // intake = new Intake(new IntakeIO() {});
                    // actuation = new Actuation(new ActuationIO() {});
                    // elevator = new Elevator(new ElevatorIO() {});
                    // winch = new Winch(new WinchIO() {});
                    // footer = new Footer(new FooterIO() {});

                    // leds = new LED(new LEDIOCANdle(60));
                }
                case DEVBOT -> {
                    // drive = new Drive(
                    // new GyroIOPigeon2(true),
                    // new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[0], "DriveBus"),
                    // new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[1], "DriveBus"),
                    // new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[2], "DriveBus"),
                    // new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[3], "DriveBus"));

                    drive = new Drive(
                            new GyroIO() {
                            },
                            new ModuleIO() {
                            },
                            new ModuleIO() {
                            },
                            new ModuleIO() {
                            },
                            new ModuleIO() {
                            });

                    bottomVision = new Vision("Bottom Vision", new VisionIOLimelight("limelight-bottom"),
                            drive::getSpeeds);
                    backVision = new Vision("Back Vision", new VisionIOLimelight("limelight-back"), drive::getSpeeds);
                    topVision = new Vision("Top Vision", new VisionIOLimelight("limelight-top"), drive::getSpeeds);

                    // rightVision = new Vision("Right Vision", new
                    // VisionIOLimelight("limelight-right"), drive::getSpeeds);

                    intake = new Intake(new IntakeIO() {
                    });
                    actuation = new Actuation(new ActuationIO() {});
                    elevator = new Elevator(new ElevatorIO() {});
                    winch = new Winch(new WinchIOKraken(20, 21));
                    footer = new Footer(new FooterIOKraken(19));

                    // leds = new LED(new LEDIOCANdle(60));
                }
                case SIMBOT -> {
                    throw new IllegalStateException("SIMBOT is not currently implemented on this robot");
                }
                default -> {
                    throw new IllegalStateException("Robot type not selected");
                }
            }
        } else {
            drive = new Drive(
                    new GyroIO() {
                    },
                    new ModuleIO() {
                    },
                    new ModuleIO() {
                    },
                    new ModuleIO() {
                    },
                    new ModuleIO() {
                    });

            bottomVision = new Vision("Bottom Vision", new VisionIO() {
            }, drive::getSpeeds);
            backVision = new Vision("Back Vision", new VisionIO() {
            }, drive::getSpeeds);
            topVision = new Vision("Top Vision", new VisionIO() {
            }, drive::getSpeeds);
            // rightVision = new Vision("Right Vision", new VisionIO() {},
            // drive::getSpeeds);

            intake = new Intake(new IntakeIO() {
            });
            actuation = new Actuation(new ActuationIO() {
            });
            elevator = new Elevator(new ElevatorIO() {
            });
            winch = new Winch(new WinchIO() {
            });
            footer = new Footer(new FooterIO() {
            });

            // leds = new LED(new LEDIO() {});
        }
    }
}
