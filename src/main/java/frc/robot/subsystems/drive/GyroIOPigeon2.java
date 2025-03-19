// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
    private static final int id = 0;

    private final boolean phoenixDrive;
    private final String CANbus;

    private Pigeon2 pigeon;
    private StatusSignal<Angle> yaw;
    private Queue<Double> yawPositionQueue;
    private StatusSignal<AngularVelocity> yawVelocity;

    public GyroIOPigeon2(boolean phoenixDrive, String CANbus) {
        this.phoenixDrive = phoenixDrive;
        this.CANbus = CANbus;
        
        constructPigeon();
    }

    private void constructPigeon() {
        pigeon = new Pigeon2(id, CANbus);
        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = pigeon.getConfigurator().apply(new Pigeon2Configuration());
            pigeon.getConfigurator().setYaw(0.0);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        yaw.setUpdateFrequency(DriveConstants.odometryFrequency);
        yawVelocity.setUpdateFrequency(100.0);
        pigeon.optimizeBusUtilization();
        if (phoenixDrive) {
            yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon, pigeon.getYaw());
        } else {
            yawPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
        }
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();

        if (!inputs.connected && !DriverStation.isEnabled()) {
            constructPigeon();
        }

        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        inputs.odometryYawPositions = yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        yawPositionQueue.clear();
    }
}
