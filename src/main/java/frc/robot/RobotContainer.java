// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.*;
import static frc.robot.subsystems.actuation.ActuationConstants.dealgifyPos;
import static frc.robot.subsystems.elevator.ElevatorConstants.highDealgifyPos;
import static frc.robot.subsystems.elevator.ElevatorConstants.lowDealgifyPos;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import java.nio.charset.StandardCharsets;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls.ManualMode;
import frc.robot.Controls.StandardMode;
import frc.robot.FieldConstants.Reef;
import frc.robot.RobotState.actions;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.automation.Dealgify;
import frc.robot.commands.automation.SetPosition;
import frc.robot.commands.automation.TuckCommand;
import frc.robot.commands.autos.Coral1Algae2Left;
import frc.robot.commands.autos.Mobility;
import frc.robot.commands.autos.Place1;
import frc.robot.commands.drivetrain.TeleAutoTurn;
import frc.robot.subsystems.actuation.ActuationConstants;
import frc.robot.subsystems.climber.climberController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
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
        configureManualMode();

        // Alerts for constants
        if (Constants.tuningMode) {
            new Alert("Tuning mode enabled", AlertType.INFO).set(true);
        }
    }

    private void setupAutos() {
        autoChooser.setDefaultOption("Coral 1 Algae 2 Left", new Coral1Algae2Left().getAuto().cmd());
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
        // new Trigger(isDesiredAction(actions.L4))
        // .and(RobotState.getInstance()::isStandardMode)
        // .onTrue(Commands.runOnce(() -> setDriveMults(maxDriveSpeed * 0.5,
        // maxDriveSpeed * 0.5)))
        // .onFalse(Commands.runOnce(() -> setDriveMults(maxDriveSpeed, maxTurnSpeed)));

        // new Trigger(isDesiredAction(actions.L3))
        // .or(isDesiredAction(actions.L2))
        // .and(RobotState.getInstance()::isStandardMode)
        // .onTrue(Commands.runOnce(() -> setDriveMults(maxDriveSpeed * 0.75,
        // maxTurnSpeed * 0.75)))
        // .onFalse(Commands.runOnce(() -> setDriveMults(maxDriveSpeed, maxTurnSpeed)));

        // new Trigger(isDesiredAction(actions.DEALGIFY_LOW))
        // .or(isDesiredAction(actions.DEALGIFY_HIGH))
        // .and(RobotState.getInstance()::isStandardMode)
        // .onTrue(Commands.runOnce(() -> setDriveMults(maxDriveSpeed * 0.75,
        // maxTurnSpeed * 0.75)))
        // .onFalse(Commands.runOnce(() -> setDriveMults(maxDriveSpeed, maxTurnSpeed)));

        new Trigger(isDesiredAction(actions.PLACE_ALGAE))
                .and(RobotState.getInstance()::isStandardMode)
                .onTrue(Commands.runOnce(() -> setDriveMults(maxDriveSpeed * 0.5, maxTurnSpeed * 0.5)))
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

        StandardMode.inttake.onTrue(setDesiredAction(actions.FLOOR_PICKUP));

        StandardMode.outtake.whileTrue(intake.runVelocityCommand(outtakeVelocity))
                .onFalse(setDesiredAction(actions.NONE));

        StandardMode.highDealgify.onTrue(setDesiredAction(actions.DEALGIFY_HIGH));
        StandardMode.lowDealgify.onTrue(setDesiredAction(actions.DEALGIFY_LOW));

        StandardMode.placeAlgae.onTrue(setDesiredAction(actions.PLACE_ALGAE));
        StandardMode.processor.onTrue(setDesiredAction(actions.PROCESSOR));

        StandardMode.climb.whileTrue(climberController.climb());
        StandardMode.deployClimber.whileTrue(climberController.climberOut());

        StandardMode.turtleMode.onTrue(setDesiredAction(actions.TURTLE));

        new Trigger(isDesiredAction(actions.FLOOR_PICKUP))
                .and(RobotState.getInstance()::isStandardMode)
                .onTrue(
                        new SequentialCommandGroup(
                                new SetPosition(pickup),
                                intake.runOnce(() -> intake.runVelocity(intakeVelocity.getAsDouble())),
                                Commands.waitUntil(RobotState.getInstance()::hasAlgae),
                                Commands.waitSeconds(0.25),
                                setDesiredAction(actions.NONE)));

        // Dealgify
        new Trigger(isDesiredAction(actions.DEALGIFY_HIGH))
                .and(RobotState.getInstance()::isStandardMode)
                .whileTrue(new SequentialCommandGroup(
                        new SetPosition(highAlgae),
                        intake.runVelocityCommand(intakeVelocity)));

        new Trigger(isDesiredAction(actions.DEALGIFY_LOW))
                .and(RobotState.getInstance()::isStandardMode)
                .whileTrue(new SequentialCommandGroup(
                        new SetPosition(lowAlgae),
                        intake.runVelocityCommand(intakeVelocity)));

        new Trigger(() -> isDesiredAction(actions.DEALGIFY_HIGH).getAsBoolean()
                || isDesiredAction(actions.DEALGIFY_LOW).getAsBoolean())
                .and(RobotState.getInstance()::isStandardMode)
                .and(RobotState.getInstance()::hasAlgae)
                .and(() -> !RobotState.getInstance().isInAReefZone())
                .whileTrue(setDesiredAction(actions.NONE));

        // Barge
        new Trigger(isDesiredAction(actions.PLACE_ALGAE))
                .and(RobotState.getInstance()::isStandardMode)
                .whileTrue(new SetPosition(Constants.barge));

        // Processor
        new Trigger(isDesiredAction(actions.PROCESSOR))
                .and(RobotState.getInstance()::isStandardMode)
                .whileTrue(new SetPosition(Constants.processor));

        new Trigger(isDesiredAction(actions.TURTLE))
                .and(RobotState.getInstance()::isStandardMode)
                .whileTrue(new SetPosition(Constants.turtle));
    }

    private void configureManualMode() {
        ManualMode.elevatorUp.whileTrue(
                elevator.runVoltageCommand(
                        () -> MathUtil.applyDeadband(driver.getRightTriggerAxis(), 0.1) * 6));
        ManualMode.elevatorDown.whileTrue(
                elevator.runVoltageCommand(
                        () -> MathUtil.applyDeadband(driver.getLeftTriggerAxis(), 0.1) * -6));

        ManualMode.actuationUp.whileTrue(actuation.runVoltageCommand(3));
        ManualMode.actuationDown.whileTrue(actuation.runVoltageCommand(-3));

        ManualMode.intakeIn.whileTrue(intake.runVelocityCommand(intakeVelocity));
        // ManualMode.intakeIn.whileTrue(Commands.startEnd(() -> intake.runVoltage(-12),
        // () -> intake.runVoltage(0), intake));
        ManualMode.intakeOut.whileTrue(intake.runVelocityCommand(outtakeVelocity));

        // driver.x().onTrue(elevator.runOnce(() -> elevator.setPos(15)));
        // driver.y().onTrue(elevator.runOnce(() -> elevator.setPos(35)));
        // driver.a().onTrue(actuation.runOnce(() -> actuation.setPos(-20)));
        // driver.y().onTrue(actuation.runOnce(() -> actuation.setPos(20)));
        // driver.pov(90).whileTrue(Commands.startEnd(() -> intake.runVoltage(-1.0), ()
        // -> intake.runVoltage(0), intake));

        // driver.a().whileTrue(Commands.startEnd(() -> intake.runVoltage(-1.0), () ->
        // intake.runVoltage(0), intake));

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
