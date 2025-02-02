package frc.robot.subsystems.algaeActuation;

import static frc.robot.subsystems.algaeActuation.AlgaeActuationConstants.rotationsToDegrees;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class AlgaeActuationIOKraken implements AlgaeActuationIO{
    public TalonFX actuationMotor;

    private MotionMagicVoltage motionMagicControl;
    private VoltageOut voltageControl;
    private NeutralOut StopMode;

    public AlgaeActuationIOKraken(int motorID) {
        actuationMotor = new TalonFX(motorID);

      motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
      voltageControl = new VoltageOut(0).withEnableFOC(true);
      StopMode = new NeutralOut();

      TalonFXConfiguration configs = new TalonFXConfiguration();

      configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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
        status = actuationMotor.getConfigurator().apply(configs);
        if (status.isOK()) break;
      }
      if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
      }
    }
    
    @Override
    public void updateInputs(AlgaeActuationIOInputs inputs) {
        inputs.motorPos = actuationMotor.getPosition().getValueAsDouble();
        inputs.motorCurrent = actuationMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = actuationMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = actuationMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void moveToPos(double pos) {
        actuationMotor.setControl(motionMagicControl.withPosition(pos / rotationsToDegrees));
    }

    @Override
    public void runVoltage(double voltage) {
        actuationMotor.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void stopMotors() {
        actuationMotor.setControl(StopMode);
    }
}
