package frc.robot.subsystems.algaeIntake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class AlgaeIntakeIOKraken implements AlgaeIntakeIO{
    private TalonFX intakeMotor;

    private VelocityVoltage velocityControl;
    private NeutralOut StopMode;

    public AlgaeIntakeIOKraken(int motorID) {
        intakeMotor = new TalonFX(motorID);

        velocityControl = new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
        StopMode = new NeutralOut();
        
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        configs.Slot0.kP = 4; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI =
            0.0; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD =
            0.0; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV =
            0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
        // Rotation per second
        
        // Peak output of 12 volts
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;
        
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          status = intakeMotor.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        inputs.motorCurrent = intakeMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = intakeMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = intakeMotor.getSupplyVoltage().getValueAsDouble();
        inputs.motorAccel = intakeMotor.getAcceleration().getValueAsDouble();
    }

    @Override
    public void runVelocity(double velocity) {
        intakeMotor.setControl(velocityControl.withVelocity(velocity));
    }

    @Override
    public void stopMotors() {
        intakeMotor.setControl(StopMode);
    }
}
