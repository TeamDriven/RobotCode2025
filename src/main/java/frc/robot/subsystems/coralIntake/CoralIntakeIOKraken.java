package frc.robot.subsystems.coralIntake;

import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CoralIntakeIOKraken implements CoralIntakeIO {
    private final TalonFX intakeMotor; 

    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true).withSlot(0).withAcceleration(motorAcceleration);
    private final NeutralOut stopMode = new NeutralOut();

    public CoralIntakeIOKraken(int motorID) {
        intakeMotor = new TalonFX(motorID);

        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = 40;

        configs.Slot0.kP = 4;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kV =
            0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
        // Rotation per second

        // Peak output of 6 volts
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
    public void updateInputs(CoralIntakeIOInputs inputs) {
        inputs.motorCurrent = intakeMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = intakeMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = intakeMotor.getSupplyVoltage().getValueAsDouble();
        inputs.motorAccel = intakeMotor.getAcceleration().getValueAsDouble();
    }

    @Override
    public void runMotor(double velocity) {
        intakeMotor.setControl(velocityControl.withVelocity(velocity));
    }

    @Override
    public void stopMotor() {
        intakeMotor.setControl(stopMode);
    }
}
