package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOKraken implements ClimberIO {
  public TalonFX leftMotor;
  public TalonFX rightMotor; 

  private VelocityTorqueCurrentFOC VelMode;
  private NeutralOut StopMode;

  public ClimberIOKraken(int lMotorID, int rMotorID) {
    leftMotor = new TalonFX(lMotorID);
    rightMotor = new TalonFX(rMotorID);

    VelMode = new VelocityTorqueCurrentFOC(0).withSlot(0);
    StopMode = new NeutralOut();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 10; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leftMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = rightMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.position = leftMotor.getPosition().getValueAsDouble();
    inputs.velocity = leftMotor.getVelocity().getValueAsDouble();
  }

  public void runClimberMotors(double velocity) {
    leftMotor.setControl(VelMode.withVelocity(velocity).withAcceleration(60));
    rightMotor.setControl(VelMode.withVelocity(velocity).withAcceleration(60));
  }

  public void stopMotor() {
    leftMotor.setControl(StopMode);
    rightMotor.setControl(StopMode);
  }
}
