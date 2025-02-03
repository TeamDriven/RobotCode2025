package frc.robot.subsystems.coralActuation;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.TalonFXUtil;
import frc.robot.util.TalonFXUtil.ConfigFactory;

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

        ConfigFactory configFactory = new ConfigFactory();
        configFactory.setBrakeMode(true);
        configFactory.setInverted(false);
        configFactory.setCurrentLimits(40);
        configFactory.setVoltageLimits(8);

        configFactory.setSlot0(0.01, 0, 0);
        configFactory.setMotionMagic(10, 25, 35);

        TalonFXUtil.applySettings(coralActuationMotor, configFactory.getConfig());
    }

    @Override
    public void updateInputs(CoralActuationIOInputs inputs) {
        inputs.motorPos = coralActuationMotor.getPosition().getValueAsDouble();
        inputs.motorCurrent = coralActuationMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = coralActuationMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = coralActuationMotor.getMotorVoltage().getValueAsDouble();
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

