package frc.robot.subsystems.elevator;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

import static frc.robot.subsystems.elevator.ElevatorConstants.gearRatio;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorIOSim implements ElevatorIO {
    private static final DCMotor leftMotorModel = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor rightMotorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim leftMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(leftMotorModel, 0.025, ElevatorConstants.gearRatio),
            leftMotorModel);
    private final DCMotorSim rightMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rightMotorModel, 0.004, ElevatorConstants.gearRatio),
            rightMotorModel);

    private PIDController posPid = new PIDController(6.5, 0, 0.02);

    private double LeftAppliedVolts = 0.0;
    private double RightAppliedVolts = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (DriverStation.isDisabled()) {
        }

        leftMotorSim.update(Constants.loopPeriodSecs);
        rightMotorSim.update(Constants.loopPeriodSecs);

        inputs.leftMotorPos = leftMotorSim.getAngularPositionRotations() * gearRatio;
        inputs.leftMotorVoltage = LeftAppliedVolts;
        inputs.leftMotorCurrent = Math.abs(leftMotorSim.getCurrentDrawAmps());
        inputs.leftMotorVel = leftMotorSim.getAngularVelocityRPM() / gearRatio;
        inputs.leftMotorAccel = leftMotorSim.getAngularAccelerationRadPerSecSq() / gearRatio;
        inputs.leftTemp = 0;
        // inputs.leftIsMotionMagic = false;

        inputs.rightMotorPos = leftMotorSim.getAngularPositionRotations() * gearRatio;
        inputs.rightMotorVoltage = LeftAppliedVolts;
        inputs.rightMotorCurrent = Math.abs(leftMotorSim.getCurrentDrawAmps());
        inputs.rightMotorVel = leftMotorSim.getAngularVelocityRPM() / gearRatio;
        inputs.rightMotorAccel = leftMotorSim.getAngularAccelerationRadPerSecSq() / gearRatio;
        inputs.rightTemp = 0;
        // inputs.rightIsMotionMagic = false;

        // inputs.absoluteEncoderPos = 0;
        // inputs.relativeEncoderPos = 0;
    }

    @Override
    public void moveToPos(double pos) {
        posPid.setSetpoint(pos);
    }

}
