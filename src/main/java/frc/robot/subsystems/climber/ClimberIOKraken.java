package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.TalonFXUtil;
import frc.robot.util.TalonFXUtil.ConfigFactory;

public class ClimberIOKraken implements ClimberIO {
  

  public TalonFX topClimberMotor;
  public TalonFX bottomClimberMotor;

  private VoltageOut voltageControl;
  private NeutralOut StopMode;

  public ClimberIOKraken(int tMotorID, int bMotorID) {
    topClimberMotor = new TalonFX(tMotorID);
    bottomClimberMotor = new TalonFX(bMotorID);

    voltageControl = new VoltageOut(0).withEnableFOC(true);
    StopMode = new NeutralOut();

    ConfigFactory configFactory = new ConfigFactory();

    configFactory.setBrakeMode(true);
    configFactory.setInverted(false);
    configFactory.setCurrentLimits(80);
    configFactory.setVoltageLimits(12);

    TalonFXUtil.applySettings(topClimberMotor, bottomClimberMotor, configFactory.getConfig(), false);
  }
  
  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.topMotorPos = topClimberMotor.getPosition().getValueAsDouble();
    inputs.topMotorCurrent = topClimberMotor.getSupplyCurrent().getValueAsDouble();
    inputs.topMotorVoltage = topClimberMotor.getMotorVoltage().getValueAsDouble();
    inputs.topStallCurrent = topClimberMotor.getMotorStallCurrent().getValueAsDouble();
    inputs.topTorqueCurrent = topClimberMotor.getTorqueCurrent().getValueAsDouble();
    
    inputs.bottomMotorPos = bottomClimberMotor.getPosition().getValueAsDouble();
    inputs.bottomMotorCurrent = bottomClimberMotor.getSupplyCurrent().getValueAsDouble();
    inputs.bottomMotorVoltage = bottomClimberMotor.getMotorVoltage().getValueAsDouble();
    inputs.bottomStallCurrent = bottomClimberMotor.getMotorStallCurrent().getValueAsDouble();
    inputs.bottomTorqueCurrent = bottomClimberMotor.getTorqueCurrent().getValueAsDouble();
  }

  @Override
  public void runClimberMotors(double voltage) {
    topClimberMotor.setControl(voltageControl.withOutput(voltage));
    bottomClimberMotor.setControl(voltageControl.withOutput(voltage));
  }

  @Override
  public void stopClimber() {
    topClimberMotor.setControl(StopMode);
    bottomClimberMotor.setControl(StopMode);
  }
}
