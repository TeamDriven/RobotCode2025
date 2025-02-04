package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.rotationsToInches;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;

import frc.robot.util.TalonFXUtil;
import frc.robot.util.TalonFXUtil.ConfigFactory;

public class ElevatorIOKraken implements ElevatorIO {
  private TalonFX elevatorLeftMotor;
  private TalonFX elevatorRightMotor;

  private MotionMagicVoltage motionMagicControl;
  private VelocityVoltage velocityControl;
  private VoltageOut voltageOut;
  private NeutralOut StopMode;

  public ElevatorIOKraken(int leftMotorID, int rightMotorID) {
    elevatorLeftMotor = new TalonFX(leftMotorID);
    elevatorRightMotor = new TalonFX(rightMotorID);

    motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    velocityControl = new VelocityVoltage(0).withEnableFOC(true).withSlot(1);
    voltageOut = new VoltageOut(0).withEnableFOC(true);
    StopMode = new NeutralOut();

    ConfigFactory configFactory = new ConfigFactory();
    configFactory.setInverted(false);
    configFactory.setBrakeMode(true);
    configFactory.setVoltageLimits(12);
    configFactory.setCurrentLimits(80);

    configFactory.setSlot0(0.01, 0, 0);

    configFactory.setMotionMagic(10, 25, 35);

    TalonFXUtil.applySettings(elevatorLeftMotor, elevatorRightMotor, configFactory.getConfig(), true);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorPos = elevatorLeftMotor.getPosition().getValueAsDouble();
    inputs.leftMotorCurrent = elevatorLeftMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leftMotorVel = elevatorLeftMotor.getVelocity().getValueAsDouble();
    inputs.leftMotorVoltage = elevatorLeftMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftMotorAccel = elevatorLeftMotor.getAcceleration().getValueAsDouble();
    inputs.leftIsMotionMagic = elevatorLeftMotor.getMotionMagicIsRunning().getValue() == MotionMagicIsRunningValue.Enabled;
    inputs.leftTemp = elevatorLeftMotor.getDeviceTemp().getValueAsDouble();

    inputs.rightMotorPos = elevatorRightMotor.getPosition().getValueAsDouble();
    inputs.rightMotorCurrent = elevatorRightMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightMotorVel = elevatorRightMotor.getVelocity().getValueAsDouble();
    inputs.rightMotorVoltage = elevatorRightMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightMotorAccel = elevatorRightMotor.getAcceleration().getValueAsDouble();
    inputs.rightIsMotionMagic = elevatorRightMotor.getMotionMagicIsRunning().getValue() == MotionMagicIsRunningValue.Enabled;
    inputs.leftTemp = elevatorLeftMotor.getDeviceTemp().getValueAsDouble();

  }

  @Override
  public void moveToPos(double pos) {
    elevatorLeftMotor.setControl(motionMagicControl.withPosition(pos / rotationsToInches));
    elevatorRightMotor.setControl(motionMagicControl.withPosition(pos / rotationsToInches));
  }

  @Override
  public void runVelocity(double speed) {
    elevatorLeftMotor.setControl(velocityControl.withVelocity(speed));
    elevatorRightMotor.setControl(velocityControl.withVelocity(speed));
  }

  @Override
  public void runVoltage(double volts) {
    elevatorLeftMotor.setControl(voltageOut.withOutput(volts));
    elevatorRightMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stopMotors() {
    elevatorLeftMotor.setControl(StopMode);
    elevatorRightMotor.setControl(StopMode);
  }
}
