package frc.robot.subsystems.coralIntake;

import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.*;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.TalonFXUtil;
import frc.robot.util.TalonFXUtil.ConfigFactory;

public class CoralIntakeIOKraken implements CoralIntakeIO {
    private final TalonFX intakeMotor; 

    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true).withSlot(0).withAcceleration(motorAcceleration);
    private final NeutralOut stopMode = new NeutralOut();

    public CoralIntakeIOKraken(int motorID) {
        intakeMotor = new TalonFX(motorID);

        ConfigFactory configFactory = new ConfigFactory();
        configFactory.setBrakeMode(true);
        configFactory.setInverted(false);
        configFactory.setCurrentLimits(40);
        configFactory.setVoltageLimits(8);
        configFactory.setSlot0(0.01, 0, 0);

        TalonFXUtil.applySettings(intakeMotor, configFactory.getConfig());
    }

    @Override
    public void updateInputs(CoralIntakeIOInputs inputs) {
        inputs.motorCurrent = intakeMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = intakeMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
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
