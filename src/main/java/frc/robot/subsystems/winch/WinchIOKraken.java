package frc.robot.subsystems.winch;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.TalonFXUtil;
import frc.robot.util.TalonFXUtil.ConfigFactory;
import frc.robot.util.TalonFXUtil.MotorFactory;

public class WinchIOKraken implements WinchIO {
  private MotorFactory motorFactory;

  private TalonFX leftWinchMotor;
  private TalonFX rightWinchMotor;

  private VoltageOut voltageControl;
  private NeutralOut StopMode;

  public WinchIOKraken(int leftMotorID, int rightMotorID) {
    motorFactory = new MotorFactory("winch", leftMotorID, rightMotorID);

    motorFactory.setBrakeMode(true);
    motorFactory.setInverted(false, false);
    motorFactory.setCurrentLimits(80);
    motorFactory.setVoltageLimits(12);

    motorFactory.configureMotors();

    var motors = motorFactory.getMotors();
    leftWinchMotor = motors[0];
    rightWinchMotor = motors[1];


    voltageControl = new VoltageOut(0).withEnableFOC(true);
    StopMode = new NeutralOut();
  }
  
  @Override
  public void updateInputs(WinchIOInputs inputs) {
    inputs.leftMotorPos = leftWinchMotor.getPosition().getValueAsDouble();
    inputs.leftMotorCurrent = leftWinchMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leftMotorVoltage = leftWinchMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftStallCurrent = leftWinchMotor.getMotorStallCurrent().getValueAsDouble();
    inputs.leftTorqueCurrent = leftWinchMotor.getTorqueCurrent().getValueAsDouble();
    
    inputs.rightMotorPos = rightWinchMotor.getPosition().getValueAsDouble();
    inputs.rightMotorCurrent = rightWinchMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightMotorVoltage = rightWinchMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightStallCurrent = rightWinchMotor.getMotorStallCurrent().getValueAsDouble();
    inputs.rightTorqueCurrent = rightWinchMotor.getTorqueCurrent().getValueAsDouble();

    motorFactory.checkForUpdates();

  }

  @Override
  public void runWinchMotors(double voltage) {
    leftWinchMotor.setControl(voltageControl.withOutput(voltage));
    rightWinchMotor.setControl(voltageControl.withOutput(voltage));
  }

  @Override
  public void stopWinch() {
    leftWinchMotor.setControl(StopMode);
    rightWinchMotor.setControl(StopMode);
  }
}
