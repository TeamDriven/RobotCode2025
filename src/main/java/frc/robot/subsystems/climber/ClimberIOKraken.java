package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = topClimberMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = bottomClimberMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
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
