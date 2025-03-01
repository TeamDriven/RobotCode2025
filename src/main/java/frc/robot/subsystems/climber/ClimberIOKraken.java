package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.TalonFXUtil.MotorFactory;

public class ClimberIOKraken implements ClimberIO {
  private MotorFactory motorFactory;

  private TalonFX topClimberMotor;
  private TalonFX bottomClimberMotor;

  private VoltageOut voltageControl;
  private NeutralOut StopMode;

  public ClimberIOKraken(int tMotorID, int bMotorID) {
    motorFactory = new MotorFactory("climber", tMotorID, bMotorID);

    motorFactory.setBrakeMode(true);
    motorFactory.setInverted(true, true);
    motorFactory.setCurrentLimits(80);
    motorFactory.setVoltageLimits(12);

    motorFactory.configureMotors();

    var motors = motorFactory.getMotors();
    topClimberMotor = motors[0];
    bottomClimberMotor = motors[1];


    voltageControl = new VoltageOut(0).withEnableFOC(true);
    StopMode = new NeutralOut();
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

    // motorFactory.checkForUpdates();

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
