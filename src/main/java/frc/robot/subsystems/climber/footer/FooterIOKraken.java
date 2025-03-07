package frc.robot.subsystems.climber.footer;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.util.TalonFXUtil.MotorFactory;

public class FooterIOKraken implements FooterIO {
  private MotorFactory motorFactory;

  private TalonFX footerMotor;

  private VoltageOut voltageControl;
  private MotionMagicVoltage motionMagicControl;
  private NeutralOut StopMode;

  public FooterIOKraken(int motorID) {
    motorFactory = new MotorFactory("footer", motorID);

    motorFactory.setBrakeMode(true);
    motorFactory.setInverted(false);
    motorFactory.setCurrentLimits(80);
    motorFactory.setVoltageLimits(12);

    motorFactory.setSlot0(0.1, 0, 0);
    motorFactory.setMotionMagic(100, 60, 120);

    motorFactory.configureMotors();

    footerMotor = motorFactory.getMotors()[0];

    motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    voltageControl = new VoltageOut(0).withEnableFOC(true);
    StopMode = new NeutralOut();
  }
  
  @Override
  public void updateInputs(FooterIOInputs inputs) {
    inputs.MotorPos = footerMotor.getPosition().getValueAsDouble();
    inputs.MotorCurrent = footerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.MotorVoltage = footerMotor.getMotorVoltage().getValueAsDouble();
    inputs.StallCurrent = footerMotor.getMotorStallCurrent().getValueAsDouble();
    inputs.TorqueCurrent = footerMotor.getTorqueCurrent().getValueAsDouble();
    // motorFactory.checkForUpdates();

  }

  @Override
  public void runVoltage(double voltage) {
    footerMotor.setControl(voltageControl.withOutput(voltage));
  }

  @Override
  public void stopFooter() {
    footerMotor.setControl(StopMode);
  }

  @Override
  public void moveToPos(double pos) {
    footerMotor.setControl(motionMagicControl.withPosition(pos));
  }
}
