// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.*;
import static frc.robot.Controls.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls.ManualMode;
import frc.robot.Controls.StandardMode;
import frc.robot.FieldConstants.Reef;
import frc.robot.RobotState.actions;
import frc.robot.commands.automation.Dealgify;
import frc.robot.commands.automation.SetPosition;
import frc.robot.commands.automation.TuckCommand;
import frc.robot.commands.autos.Mobility;
import frc.robot.commands.autos.Place1;
import frc.robot.commands.autos.Place3LeftSide;
import frc.robot.commands.autos.Place3RightSide;
import frc.robot.commands.drivetrain.TeleAutoTurn;
import frc.robot.subsystems.actuation.ActuationConstants;
import frc.robot.subsystems.climber.climberController;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.*;
import frc.robot.util.Alert.AlertType;

public class RobotContainer {
    private final RobotState robotState = RobotState.getInstance();
    private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).",
            AlertType.WARNING);

    private static final double maxDriveSpeed = 1;
    private static final double maxTurnSpeed = 0.9;

    private static double driveMult = maxDriveSpeed;
    private static double turnMult = maxTurnSpeed;

    private static void setDriveMults(double drive, double turn) {
        driveMult = drive;
        turnMult = turn;
    }

    private static SendableChooser<Command> autoChooser = new SendableChooser<>();

    public static boolean shouldUseZones = true;

    public double time = Double.NaN;

    public LoggedTunableNumber elevatorPos = new LoggedTunableNumber("Debug/elevatorPos", 10);
    public LoggedTunableNumber actuationPos = new LoggedTunableNumber("Debug/actuationPos", 10);

    private DigitalInput driverDistanceSensor = new DigitalInput(3);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        FieldConstants.logFieldConstants();

        // Configure autos and buttons
        setupAutos();
        universalControls();
        configureStandardMode();
        configureNoLimelightMode();
        configureManualMode();

        // Alerts for constants
        if (Constants.tuningMode) {
            new Alert("Tuning mode enabled", AlertType.INFO).set(true);
        }
    }

    private void setupAutos() {
        autoChooser.setDefaultOption("Place 3 Left Side", new Place3LeftSide().getAuto().cmd());
        autoChooser.addOption("Place 3 Right Side", new Place3RightSide().getAuto().cmd());
        autoChooser.addOption("Mobility", new Mobility().getAuto().cmd());
        autoChooser.addOption("Place 1", new Place1().getAuto().cmd());

        SmartDashboard.putData(autoChooser);
    }

    private Command driveCommand = drive.run(
            () -> drive.acceptTeleopInput(
                    driveX.getAsDouble() * driveMult, driveY.getAsDouble() * driveMult,
                    driveOmega.getAsDouble() * turnMult,
                    false))
            .withName("Drive Teleop Input");

    private Command setDesiredAction(actions desiredAction) {
        return Commands.runOnce(() -> RobotState.getInstance().setDesiredAction(desiredAction));
    }

    private BooleanSupplier isDesiredAction(actions desiredAction) {
        return () -> RobotState.getInstance().getDesiredAction() == desiredAction;
    }

    private boolean isTryingToPlace() {
        return switch (RobotState.getInstance().getDesiredAction()) {
            case L4, L3, L2, L1 -> true;
            default -> false;
        };
    }

    private void universalControls() {
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        drive.clearAutoAlignGoal();
        drive.clearHeadingGoal();

        RobotState.getInstance().setDesiredAction(actions.NONE);

        elevator.stop();
        intake.runVelocity(0);
        actuation.stop();

        drive.setDefaultCommand(driveCommand);

        resetPose.onTrue(
                Commands.runOnce(
                        () -> robotState.resetPose(
                                new Pose2d(
                                        robotState.getEstimatedPose()
                                                .getTranslation(),
                                        AllianceFlipUtil.apply(
                                                new Rotation2d()))))
                        .ignoringDisable(true));

        new Trigger(() -> !driverDistanceSensor.get())
                .onTrue(Commands.runOnce(() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.5)))
                .onFalse(Commands.runOnce(() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0)));

        resetElevatorPosition.onTrue(elevator.resetPosition());

        noLimelightMode.onTrue(Commands.runOnce(() -> RobotState.getInstance().setNoLimelightMode()));
        manualMode.onTrue(Commands.runOnce(() -> RobotState.getInstance().setManualMode()));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick}
     * or {@link
     * XboxController}), and then passing it to a {@link JoystickButton}.
     */
    private void configureStandardMode() {
        new Trigger(isDesiredAction(actions.L4))
                .and(RobotState.getInstance()::isStandardMode)
                .onTrue(Commands.runOnce(() -> setDriveMults(maxDriveSpeed * 0.5, maxDriveSpeed * 0.5)))
                .onFalse(Commands.runOnce(() -> setDriveMults(maxDriveSpeed, maxTurnSpeed)));

        new Trigger(isDesiredAction(actions.L3))
                .or(isDesiredAction(actions.L2))
                .and(RobotState.getInstance()::isStandardMode)
                .onTrue(Commands.runOnce(() -> setDriveMults(maxDriveSpeed * 0.75, maxTurnSpeed * 0.75)))
                .onFalse(Commands.runOnce(() -> setDriveMults(maxDriveSpeed, maxTurnSpeed)));

        // Drivetrain
        new Trigger(isDesiredAction(actions.NONE))
                .and(RobotState.getInstance()::isStandardMode)
                .onTrue(Commands.parallel(
                        new TuckCommand(),
                        intake.runVelocityCommand(0),
                        Commands.runOnce(() -> drive.clearAutoAlignGoal()),
                        Commands.runOnce(() -> drive.clearHeadingGoal())));

        StandardMode.cancelAction.onTrue(setDesiredAction(actions.NONE));

        // Placing
        StandardMode.placeL4.onTrue(setDesiredAction(actions.L4));
        StandardMode.placeL3.onTrue(setDesiredAction(actions.L3));
        StandardMode.placeL2.onTrue(setDesiredAction(actions.L2));
        StandardMode.placeL1.onTrue(setDesiredAction(actions.L1));

        StandardMode.inttake.onTrue(setDesiredAction(actions.PICKUP_CORAL));

        StandardMode.outtake.whileTrue(intake.runVelocityCommand(outtakeVelocity))
                .onFalse(Commands.either(setDesiredAction(actions.NONE), Commands.none(),
                        this::isTryingToPlace));

        StandardMode.dealgify.onTrue(setDesiredAction(actions.DEALGIFY));

        StandardMode.climb.whileTrue(climberController.climb());
        StandardMode.deployClimber.whileTrue(climberController.climberOut());

        // Auto align if trying to place L2+
        // new Trigger(isDesiredAction(actions.L4))
        // .and(RobotState.getInstance()::isStandardMode)
        // .or(isDesiredAction(actions.L3))
        // .or(isDesiredAction(actions.L2))
        // .whileTrue(
        // new AutoMoveToNearestPOI(allignmentMode.TWO_STAGE, Reef.placePoses)
        // .until(isDesiredAction(actions.NONE)));

        new Trigger(isDesiredAction(actions.L4))
                .or(isDesiredAction(actions.L3))
                .or(isDesiredAction(actions.L2))
                .and(RobotState.getInstance()::isStandardMode)
                .onTrue(new TeleAutoTurn(() -> Reef.findNearestReefFace(RobotState.getInstance().getEstimatedPose())
                                .facePos().getRotation().rotateBy(new Rotation2d(Math.PI))))
                .onFalse(Commands.runOnce(() -> drive.clearHeadingGoal()));

        new Trigger(isDesiredAction(actions.L4))
                .and(RobotState.getInstance()::isStandardMode)
                .and(RobotState.getInstance()::isInReefZone)
                .onTrue(new SetPosition(Constants.l4));

        new Trigger(isDesiredAction(actions.L3))
                .and(RobotState.getInstance()::isStandardMode)
                .and(RobotState.getInstance()::isInReefZone)
                .onTrue(new SetPosition(Constants.l3));

        new Trigger(isDesiredAction(actions.L2))
                .and(RobotState.getInstance()::isStandardMode)
                .and(RobotState.getInstance()::isInReefZone)
                .onTrue(new SetPosition(Constants.l2));

        new Trigger(isDesiredAction(actions.L1)).and(RobotState.getInstance()::isStandardMode)
                .onTrue(new SetPosition(Constants.l1));

        // Pickup Coral
        new Trigger(isDesiredAction(actions.PICKUP_CORAL))
                .and(RobotState.getInstance()::isStandardMode)
                .and(RobotState.getInstance()::isInLeftPickupZone)
                .onTrue(Commands.parallel(
                        new TeleAutoTurn(() -> Rotation2d.fromDegrees(125)),
                        new SetPosition(ElevatorConstants.pickUpPos,
                                ActuationConstants.pickUpPos),
                        intake.runVelocityCommand(intakeVelocity)));

        new Trigger(isDesiredAction(actions.PICKUP_CORAL))
                .and(RobotState.getInstance()::isStandardMode)
                .and(RobotState.getInstance()::isInRightPickupZone)
                .onTrue(Commands.parallel(
                        new TeleAutoTurn(() -> Rotation2d.fromDegrees(-125)),
                        new SetPosition(ElevatorConstants.pickUpPos,
                                ActuationConstants.pickUpPos),
                        intake.runVelocityCommand(intakeVelocity)));

        new Trigger(isDesiredAction(actions.PICKUP_CORAL))
                .and(RobotState.getInstance()::isStandardMode)
                .and(RobotState.getInstance()::hasCoral)
                .onTrue(Commands.runOnce(() -> drive.clearHeadingGoal()));

        new Trigger(isDesiredAction(actions.PICKUP_CORAL))
                .and(RobotState.getInstance()::isStandardMode)
                .and(RobotState.getInstance()::hasCoral)
                .and(() -> !RobotState.getInstance().isInPickupZone())
                .onTrue(setDesiredAction(actions.NONE));

        // Algae
        new Trigger(isDesiredAction(actions.DEALGIFY))
                .and(RobotState.getInstance()::isStandardMode)
                .and(RobotState.getInstance()::isInReefZone)
                .whileTrue(new Dealgify());

        new Trigger(isDesiredAction(actions.DEALGIFY))
                .and(RobotState.getInstance()::isStandardMode)
                .and(() -> !RobotState.getInstance().isInReefZone())
                .onTrue(setDesiredAction(actions.NONE));

        new Trigger(RobotState.getInstance()::isInClimbZone)
                .and(RobotState.getInstance()::isStandardMode)
                .onTrue(elevator.runOnce(() -> elevator.setPos(2)));
    }

    private void configureNoLimelightMode() {
        new Trigger(isDesiredAction(actions.L4))
                .and(RobotState.getInstance()::isNoLimelightMode)
                .onTrue(Commands.runOnce(() -> setDriveMults(maxDriveSpeed * 0.5, maxDriveSpeed * 0.5)))
                .onFalse(Commands.runOnce(() -> setDriveMults(maxDriveSpeed, maxTurnSpeed)));

        new Trigger(isDesiredAction(actions.L3))
                .or(isDesiredAction(actions.L2))
                .and(RobotState.getInstance()::isNoLimelightMode)
                .onTrue(Commands.runOnce(() -> setDriveMults(maxDriveSpeed * 0.75, maxTurnSpeed * 0.75)))
                .onFalse(Commands.runOnce(() -> setDriveMults(maxDriveSpeed, maxTurnSpeed)));

        // Drivetrain
        new Trigger(isDesiredAction(actions.NONE))
                .and(RobotState.getInstance()::isNoLimelightMode)
                .onTrue(Commands.parallel(
                        new TuckCommand(),
                        intake.runVelocityCommand(0),
                        Commands.runOnce(() -> drive.clearHeadingGoal())));
    }

    private void configureManualMode() {
        ManualMode.elevatorUp.whileTrue(
                elevator.runVoltageCommand(
                        () -> MathUtil.applyDeadband(driver.getRightTriggerAxis(), 0.1) * 6));
        ManualMode.elevatorDown.whileTrue(
                elevator.runVoltageCommand(
                        () -> MathUtil.applyDeadband(driver.getLeftTriggerAxis(), 0.1) * -6));

        ManualMode.actuationUp.whileTrue(actuation.runVoltageCommand(1));
        ManualMode.actuationDown.whileTrue(actuation.runVoltageCommand(-1));

        ManualMode.intakeIn.whileTrue(intake.runVelocityCommand(intakeVelocity.get()));
        ManualMode.intakeOut.whileTrue(intake.runVelocityCommand(outtakeVelocity.get()));

        ManualMode.winchIn.whileTrue(winch.runVoltageCommand(12));
        ManualMode.winchOut.whileTrue(winch.runVoltageCommand(-12));

        ManualMode.footerOut.whileTrue(footer.runVoltageCommand(-2));
    }

    /** Updates the alerts for disconnected controllers. */
    public void checkControllers() {
        driverDisconnected.set(
                !DriverStation.isJoystickConnected(driver.getHID().getPort())
                        || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    }

    /** Updates dashboard data. */
    public void updateDashboardOutputs() {
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // Drive Static
        // Characterization
        // return new StaticCharacterization(
        // drive, drive::runCharacterization, drive::getCharacterizationVelocity)
        // .finallyDo(drive::endCharacterization);

        // Drive FF Characterization
        // return new FeedForwardCharacterization(
        // drive, drive::runCharacterization, drive::getCharacterizationVelocity)
        // .finallyDo(drive::endCharacterization);

        // Drive Wheel Radius Characterization
        // return drive
        // .orientModules(Drive.getCircleOrientations())
        // .andThen(
        // new WheelRadiusCharacterization(
        // drive, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE))
        // .withName("Drive Wheel Radius Characterization");

        // Slippage Calculator
        // return Commands.runOnce(
        // () ->
        // robotState.resetPose(
        // new Pose2d(
        // // robotState.getEstimatedPose().getTranslation(),
        // new Translation2d(), AllianceFlipUtil.apply(new Rotation2d()))))
        // .andThen(new SlippageCalculator(drive))
        // .withName("Slippage Calculator");

        return autoChooser.getSelected();
    }
}
