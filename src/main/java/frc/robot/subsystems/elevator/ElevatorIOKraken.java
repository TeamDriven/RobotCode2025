package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.rotationsToInches;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOKraken implements ElevatorIO {
  private TalonFX elevatorLeftMotor;
  private TalonFX elevatorRightMotor;

  private MotionMagicVoltage motionMagicControl;
  private VelocityVoltage velocityControl;
  private NeutralOut StopMode;

  public ElevatorIOKraken(int leftMotorID, int rightMotorID) {
    elevatorLeftMotor = new TalonFX(leftMotorID);
    elevatorRightMotor = new TalonFX(rightMotorID);

    motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    velocityControl = new VelocityVoltage(0).withEnableFOC(true).withSlot(1);
    StopMode = new NeutralOut();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    configs.MotionMagic.MotionMagicCruiseVelocity = 100;
    configs.MotionMagic.MotionMagicAcceleration = 250;
    configs.MotionMagic.MotionMagicJerk = 350;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 10; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second

    // Peak output of 12 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = elevatorLeftMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = elevatorRightMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorPos = elevatorLeftMotor.getPosition().getValueAsDouble();
    inputs.leftMotorCurrent = elevatorLeftMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leftMotorVel = elevatorLeftMotor.getVelocity().getValueAsDouble();
    inputs.leftMotorVoltage = elevatorLeftMotor.getSupplyVoltage().getValueAsDouble();
    inputs.leftMotorAccel = elevatorLeftMotor.getAcceleration().getValueAsDouble();
    inputs.leftIsMotionMagic = elevatorLeftMotor.getMotionMagicIsRunning().getValue() == MotionMagicIsRunningValue.Enabled;
    inputs.leftTemp = elevatorLeftMotor.getDeviceTemp().getValueAsDouble();

    inputs.rightMotorPos = elevatorRightMotor.getPosition().getValueAsDouble();
    inputs.rightMotorCurrent = elevatorRightMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightMotorVel = elevatorRightMotor.getVelocity().getValueAsDouble();
    inputs.rightMotorVoltage = elevatorRightMotor.getSupplyVoltage().getValueAsDouble();
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
  public void stopMotors() {
    elevatorLeftMotor.setControl(StopMode);
    elevatorRightMotor.setControl(StopMode);
  }
}
