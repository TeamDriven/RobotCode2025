// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;
import frc.robot.RobotState;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.swerve.SwerveSetpoint;
import frc.robot.util.swerve.SwerveSetpointGenerator;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

public class Drive extends SubsystemBase {
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);

    private static final LoggedTunableNumber coastWaitTime = new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
    private static final LoggedTunableNumber coastMetersPerSecondThreshold = new LoggedTunableNumber(
            "Drive/CoastMetersPerSecThreshold", .05);

    private final Timer lastMovementTimer = new Timer();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);

    @AutoLogOutput
    private boolean velocityMode = false;
    @AutoLogOutput
    private boolean brakeModeEnabled = true;

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
            });
    private final SwerveSetpointGenerator swerveSetpointGenerator;

    private final AutonomousController autoController = new AutonomousController();
    public final AutoFactory autoFactory;

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);
        lastMovementTimer.start();
        setBrakeMode(true);

        swerveSetpointGenerator = new SwerveSetpointGenerator(kinematics, DriveConstants.moduleTranslations);

        autoFactory = new AutoFactory(
                RobotState.getInstance()::getEstimatedPose,
                RobotState.getInstance()::resetPose,
                this::acceptAutoInput,
                true,
                this);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();
    }

    public enum CoastRequest {
        AUTOMATIC,
        ALWAYS_BREAK,
        ALWAYS_COAST
    }

    @AutoLogOutput
    private CoastRequest coastRequest = CoastRequest.AUTOMATIC;

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.updateInputs();
        }
        odometryLock.unlock();
        LoggedTracer.record("Drive/Inputs");

        // Call periodic on modules
        for (var module : modules) {
            module.periodic();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", new SwerveModuleState[] {});
        }

        // Send odometry updates to robot state
        double[] sampleTimestamps = gyroInputs.odometryYawTimestamps; // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
            for (int j = 0; j < 4; j++) {
                wheelPositions[j] = modules[j].getOdometryPositions()[i];
            }
            RobotState.getInstance()
                    .addOdometryObservation(
                            new RobotState.OdometryObservation(
                                    wheelPositions,
                                    gyroInputs.odometryYawPositions[i],
                                    sampleTimestamps[i]));
        }

        RobotState.getInstance().addDriveSpeeds(getChassisSpeeds());

        // Update brake mode
        // Reset movement timer if velocity above threshold
        if (Arrays.stream(modules)
                .anyMatch(
                        (module) -> Math.abs(module.getVelocityMetersPerSec()) > coastMetersPerSecondThreshold.get())) {
            lastMovementTimer.reset();
        }

        if (DriverStation.isEnabled()) {
            coastRequest = CoastRequest.AUTOMATIC;
        }

        switch (coastRequest) {
            case AUTOMATIC -> {
                if (DriverStation.isEnabled()) {
                    setBrakeMode(true);
                } else if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
                    setBrakeMode(false);
                }
            }
            case ALWAYS_BREAK -> {
                setBrakeMode(true);
            }
            case ALWAYS_COAST -> {
                setBrakeMode(false);
            }
        }

        // Update current setpoint if not in velocity mode
        if (!velocityMode) {
            currentSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates());
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(
                !gyroInputs.data.connected() && Constants.getMode() != Mode.SIM);

        if (DriveConstants.shouldPrintZeros) {
            printZeros();
        }

        // Record cycle time
        LoggedTracer.record("Drive/Periodic");
    }

    /**
     * Set brake mode to {@code enabled} doesn't change brake mode if already set.
     */
    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled != enabled) {
            Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
        }
        brakeModeEnabled = enabled;
    }

    private void acceptAutoInput(SwerveSample swerveSample) {
        runVelocity(autoController.update(swerveSample));
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        velocityMode = true;
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
        SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);
        currentSetpoint = swerveSetpointGenerator.generateSetpoint(
                DriveConstants.moduleLimitsFree,
                currentSetpoint,
                discreteSpeeds,
                Constants.loopPeriodSecs);
        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
        Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }
    }

    /**
     * Runs the drive at the desired velocity with setpoint module forces.
     *
     * @param speeds       Speeds in meters/sec
     * @param moduleForces The forces applied to each module
     */
    public void runVelocity(ChassisSpeeds speeds, List<Vector<N2>> moduleForces) {
        velocityMode = true;
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
        SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);
        currentSetpoint = swerveSetpointGenerator.generateSetpoint(
                DriveConstants.moduleLimitsFree,
                currentSetpoint,
                discreteSpeeds,
                Constants.loopPeriodSecs);
        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
        Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

        // Save module forces to swerve states for logging
        SwerveModuleState[] wheelForces = new SwerveModuleState[4];
        // Send setpoints to modules
        SwerveModuleState[] moduleStates = getModuleStates();
        for (int i = 0; i < 4; i++) {
            // Optimize state
            Rotation2d wheelAngle = moduleStates[i].angle;
            setpointStates[i].optimize(wheelAngle);
            setpointStates[i].cosineScale(wheelAngle);

            // Calculate wheel torque in direction
            var wheelForce = moduleForces.get(i);
            Vector<N2> wheelDirection = VecBuilder.fill(wheelAngle.getCos(), wheelAngle.getSin());
            double wheelTorqueNm = wheelForce.dot(wheelDirection) * DriveConstants.wheelRadius;
            modules[i].runSetpoint(setpointStates[i], wheelTorqueNm);

            // Save to array for logging
            wheelForces[i] = new SwerveModuleState(wheelTorqueNm, setpointStates[i].angle);
        }
        Logger.recordOutput("Drive/SwerveStates/ModuleForces", wheelForces);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        velocityMode = false;
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = DriveConstants.moduleTranslations[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all the
     * modules.
     */
    @AutoLogOutput(key = "Drive/SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "Drive/SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native
     * units).
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the raw gyro rotation read by the IMU */
    public Rotation2d getGyroRotation() {
        return gyroInputs.data.yawPosition();
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return DriveConstants.maxLinearSpeed;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DriveConstants.driveBaseRadius;
    }

    public void printZeros() {
        ModuleConfig[] configs = DriveConstants.moduleConfigs;
        for (int i = 0; i < configs.length; i++) {
            System.out.printf(
                    "new ModuleConfig(%d, %d, %d, new Rotation2d(%f), %b, %b)",
                    configs[i].driveMotorId(),
                    configs[i].turnMotorId(),
                    configs[i].encoderChannel(),
                    modules[i].getAbsoulteAngle().getRadians(),
                    configs[i].turnInverted(),
                    configs[i].encoderInverted());
            if (i == configs.length - 1) {
                System.out.println();
            } else {
                System.out.println(",");
            }
        }
    }
}