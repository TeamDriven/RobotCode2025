package frc.robot.subsystems.coralActuation;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CoralActuationIOKraken implements CoralActuationIO {
    public TalonFX coralActuationMotor;

    private MotionMagicVoltage motionMagicControl;
    private VoltageOut voltageControl;
    private NeutralOut stopMode;

    public CoralActuationIOKraken(int motorID) {
        coralActuationMotor = new TalonFX(motorID);

        motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
        voltageControl = new VoltageOut(0).withEnableFOC(true);
        stopMode = new NeutralOut();

        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = 40;

        configs.MotionMagic.MotionMagicCruiseVelocity = 100;
        configs.MotionMagic.MotionMagicAcceleration = 250;
        configs.MotionMagic.MotionMagicJerk = 350;

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
          status = coralActuationMotor.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    @Override
    public void updateInputs(CoralActuationIOInputs inputs) {
        inputs.motorPos = coralActuationMotor.getPosition().getValueAsDouble();
        inputs.motorCurrent = coralActuationMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = coralActuationMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = coralActuationMotor.getSupplyVoltage().getValueAsDouble();
    }

    @Override
    public void moveToPos(double pos) {
        coralActuationMotor.setControl(motionMagicControl.withPosition(pos));
    }

    @Override
    public void runVoltage(double voltage) {
        coralActuationMotor.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void stopMotor() {
        coralActuationMotor.setControl(stopMode);
    }
}

