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

    private PIDController pid = new PIDController(6.5, 0, 0.02);

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double leftFFVolts = 0.0;
    private double rightFFVolts = 0.0;

    public ElevatorIOSim() {
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (DriverStation.isDisabled()) {
        }

        leftAppliedVolts = leftFFVolts + pid.calculate(leftMotorSim.getAngularVelocityRadPerSec());
        rightAppliedVolts = rightFFVolts + pid.calculate(leftMotorSim.getAngularVelocityRadPerSec());

        leftMotorSim.update(Constants.loopPeriodSecs);
        rightMotorSim.update(Constants.loopPeriodSecs);

        inputs.leftMotorPos = leftMotorSim.getAngularPositionRotations() * gearRatio;
        inputs.leftMotorVoltage = leftAppliedVolts;
        inputs.leftMotorCurrent = Math.abs(leftMotorSim.getCurrentDrawAmps());
        inputs.leftMotorVel = leftMotorSim.getAngularVelocityRPM() / gearRatio;
        inputs.leftMotorAccel = leftMotorSim.getAngularAccelerationRadPerSecSq() / gearRatio;
        inputs.leftTemp = 0;
        // inputs.leftIsMotionMagic = false;

        inputs.rightMotorPos = leftMotorSim.getAngularPositionRotations() * gearRatio;
        inputs.rightMotorVoltage = leftAppliedVolts;
        inputs.rightMotorCurrent = Math.abs(leftMotorSim.getCurrentDrawAmps());
        inputs.rightMotorVel = leftMotorSim.getAngularVelocityRPM() / gearRatio;
        inputs.rightMotorAccel = leftMotorSim.getAngularAccelerationRadPerSecSq() / gearRatio;
        inputs.rightTemp = 0;
        // inputs.rightIsMotionMagic = false;

        // inputs.absoluteEncoderPos = 0;
        // inputs.relativeEncoderPos = 0;

        leftMotorSim.setInputVoltage(MathUtil.clamp(leftAppliedVolts, -12.0, 12.0));
        rightMotorSim.setInputVoltage(MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));

        leftMotorSim.update(Constants.loopPeriodSecs);
        rightMotorSim.update(Constants.loopPeriodSecs);
    }

    @Override
    public void moveToPos(double pos) {
        pid.setSetpoint(pos);
    }

    @Override
    public void runVelocity(double speed) {
        pid.setSetpoint(speed);
    }

    @Override
    public void runVoltage(double volts) {
        leftAppliedVolts = MathUtil.clamp(volts, -12, 12);
        leftMotorSim.setInputVoltage(leftAppliedVolts);

        rightAppliedVolts = MathUtil.clamp(volts, -12, 12);
        rightMotorSim.setInputVoltage(rightAppliedVolts);
    }

    @Override
    public void stopMotors() {
        runVoltage(0.0);
    }

    @Override
    public void resetPosition() {
        pid.setSetpoint(0);
    }
}
