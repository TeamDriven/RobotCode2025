package frc.robot.subsystems.algaeIntake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.TalonFXUtil;
import frc.robot.util.TalonFXUtil.ConfigFactory;

public class AlgaeIntakeIOKraken implements AlgaeIntakeIO{
    private TalonFX intakeMotor;

    private VelocityVoltage velocityControl;
    private NeutralOut StopMode;

    public AlgaeIntakeIOKraken(int motorID) {
        intakeMotor = new TalonFX(motorID);

        velocityControl = new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
        StopMode = new NeutralOut();

        ConfigFactory configFactory = new ConfigFactory();
        configFactory.setBrakeMode(true);
        configFactory.setInverted(true);
        configFactory.setCurrentLimits(50);
        configFactory.setVoltageLimits(8);
        configFactory.setSlot0(0.025, 0, 0);
        
        TalonFXUtil.applySettings(intakeMotor, configFactory.getConfig());
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        inputs.motorCurrent = intakeMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = intakeMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
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
